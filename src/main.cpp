#include "config.h"
#include "globals.h"
#include "display.h"
#include "regulator.h"
#include "pushstart.h"
#include "can_comm.h"
#include "fuel.h"
#include "security.h"
#include <Fonts/FreeSansBold24pt7b.h>
#include <esp_heap_caps.h>

// Short label for abnormal resets (panic / watchdog / brownout), nullptr for normal boots.
// Shown on screen for 30 s after boot so field reboots can be diagnosed without a laptop.
static const char *abnormalResetLabel = nullptr;

static const char *getAbnormalResetLabel(esp_reset_reason_t reason)
{
  switch (reason)
  {
  case ESP_RST_PANIC:    return "RST:PANIC";
  case ESP_RST_INT_WDT:  return "RST:INT_WDT";
  case ESP_RST_TASK_WDT: return "RST:TASK_WDT";
  case ESP_RST_WDT:      return "RST:WDT";
  case ESP_RST_BROWNOUT: return "RST:BROWNOUT";
  case ESP_RST_SW:       return "RST:SW";
  case ESP_RST_UNKNOWN:  return "RST:UNKNOWN";
  default:               return nullptr; // POWERON, DEEPSLEEP, EXT, SDIO
  }
}

//=================== setup ===============//
void setup()
{
  // Release any GPIO holds from previous deep sleep FIRST so pins can be driven
  gpio_hold_dis((gpio_num_t)PIN_RELAY_ACC);
  gpio_hold_dis((gpio_num_t)PIN_RELAY_IGN);
  gpio_hold_dis((gpio_num_t)PIN_RELAY_START);
  gpio_hold_dis((gpio_num_t)PIN_5V_GATE);
  gpio_hold_dis((gpio_num_t)field_relay_pin);
  gpio_hold_dis((gpio_num_t)buzzer_pin);
  gpio_hold_dis((gpio_num_t)PIN_3V3_DIGITAL_GATE);
  gpio_hold_dis((gpio_num_t)PIN_RELAY_LOCK);
  gpio_hold_dis((gpio_num_t)PIN_CAN_TX);
  gpio_deep_sleep_hold_dis();

  // Park CAN TX recessive immediately (while PIN_CAN_RS is still held HIGH in Standby on wake)
  pinMode(PIN_CAN_TX, INPUT_PULLUP);

  // 0. Crash recovery: restore ACC/IGN immediately (before any delay) if the MCU was reset
  // unexpectedly (panic / task WDT / brownout) while the engine was running.
  // Power-on: RTC_NOINIT memory holds garbage. Deep-sleep wake: always an intentional stop.
  esp_reset_reason_t resetReason = esp_reset_reason();
  bool crashWhileRunning = (resetReason != ESP_RST_POWERON &&
                            resetReason != ESP_RST_DEEPSLEEP &&
                            rtc_run_marker == RTC_RUN_MARKER_MAGIC &&
                            rtc_run_marker_inv == ~RTC_RUN_MARKER_MAGIC);

  setupPushStartPins(crashWhileRunning);

  if (crashWhileRunning)
  {
    currentState = STATE_RUNNING; // RTC_DATA_ATTR was re-initialized by the bootloader
    lastEngineStartTime = millis();
  }
  else
  {
    // Initialize system state to Standby with 2-minute sleep timeout on normal boot/wake
    currentState = STATE_STANDBY;
    rtc_run_marker = 0;
    rtc_run_marker_inv = 0;
  }
  standbyStartTime = millis();
  regulatorTaskRunning = true;
  stoppedToAcc = false;

  // 1. Turn on the switched rails
  pinMode(PIN_5V_GATE, OUTPUT);
  digitalWrite(PIN_5V_GATE, HIGH); // Enable Relay

  pinMode(PIN_3V3_DIGITAL_GATE, OUTPUT);
  digitalWrite(PIN_3V3_DIGITAL_GATE, HIGH); // Power ADS1115, I2C pull-ups, and IR2110 VDD (~1-3mA)

  // Release RS sleep hold AFTER CTX is already pulled HIGH -> High-Z (INPUT) lets the module's
  // on-board 10k resistor to GND set Slope Control Mode (~15V/µs, low EMI) with zero TX glitch
  gpio_hold_dis((gpio_num_t)PIN_CAN_RS);
  pinMode(PIN_CAN_RS, INPUT);

  delay(30); // Allow voltage rails to stabilize
  Serial.begin(250000);
  delay(20); // Let UART stabilize

  if (crashWhileRunning)
  {
    Serial.printf("[SAFETY] Unexpected reset (reason %d) while engine running — ACC/IGN restored.\n", (int)resetReason);
  }
  abnormalResetLabel = getAbnormalResetLabel(resetReason);
  if (abnormalResetLabel != nullptr)
  {
    Serial.printf("[BOOT] Abnormal reset: %s (reason %d)\n", abnormalResetLabel, (int)resetReason);
  }

  recoverI2CBus(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.begin();
  Wire.setClock(100000); // Slower clock for better noise immunity in engine bay
  Wire.setTimeOut(20);   // Abort I2C transaction if it takes > 20ms
  bool adcReady = adc.begin();
  if (adcReady)
  {
    adc.setGain(GAIN_ONE); // 1x gain for ±4.096V range, adjust if your input
                           // exceeds this
    adc.setDataRate(RATE_ADS1115_250SPS);
  }
  setupBLESecurity(); // Initialize BLE scanner to detect phone (auto-shuts down upon detection)
  tv.begin();
  tv.copyAfterSwap = true;

  if (rtc_trip_magic != RTC_TRIP_MAGIC_KEY || isnan(total_fuel_liters) ||
      isnan(total_distance_km) || isnan(total_fuel_saved_liters))
  {
    loadTripDataFromNVS(); // Reads double keys (migrates legacy float keys) and sanitizes values
    rtc_trip_magic = RTC_TRIP_MAGIC_KEY;
  }

  pinModeFast(buzzer_pin, OUTPUT);
  pinModeFast(coolant_level_pin, INPUT);
  pinMode(field_relay_pin, OUTPUT);
  digitalWriteFast(field_relay_pin, LOW); // Start with field relay off

  esp_task_wdt_deinit();      // De-init default core WDT config
  esp_task_wdt_init(1, true); // 1s timeout with panic=true
  esp_task_wdt_add(nullptr);  // Add current loop task
  ledcSetup(0, 400, 10);      // channel, freq, resolution
  ledcAttachPin(FIELD_PIN, 0);

  lastPacketTime = 0;     // 0 = no Front MCU packet received yet (grace handled per-state)
  lastFastPacketTime = 0; // 0 = no 0x02 fast engine/speed frame received yet
  lastVacPacketTime = 0;  // 0 = no 0x05 manifold vacuum/baro frame received yet
  lastEnvPacketTime = 0;  // 0 = no 0x06 thermal/aux frame received yet
  startCanRxTask();       // Install TWAI & drain CAN RX on Core 0 (zero video V-Sync lag)

  // Configure FreeRTOS task for regulator loop instead of hardware timer
  if (adcReady)
  {
    xTaskCreatePinnedToCore(
        regulatorTask,   // Task function
        "RegulatorTask", // Name of task
        8192,            // Stack size of task
        nullptr,         // Parameter of the task
        configMAX_PRIORITIES -
            2,                // Priority: high but not max, to avoid starving idle task
        &regulatorTaskHandle, // Task handle
        0);                   // Pin to Core 0 (Isolate from UI on Core 1)
  }
}

// ===================== main loop ======================//
void loop()
{
  // uint32_t start = micros();
  unsigned long now = millis();

  //============= send health signal to front MCU (only when POS2/IGN is active) ==================
  static unsigned long lastCanSendTimeMs = 0;
  if (isCanActiveState(currentState) &&
      now - lastCanSendTimeMs >= CAN_HEALTH_SEND_INTERVAL_MS)
  {
    sendCanHealthFrame(now);
    lastCanSendTimeMs = now;
  }

  if (last_clear < 6)
  {
    tv.fillScreen(0x00);
    drawStaticGauge();
    last_clear++;
  }

  // Show abnormal reset reason (if any) for 30 s once the initial screen clears are done
  static unsigned long resetLabelShownAt = 0;
  if (abnormalResetLabel != nullptr && last_clear >= 6)
  {
    if (resetLabelShownAt == 0)
    {
      tv.setCursor(5, 70);
      tv.setTextColor(0xE0, 0x00);
      tv.print(abnormalResetLabel);
      resetLabelShownAt = (now != 0) ? now : 1;
    }
    else if (now - resetLabelShownAt >= 30000)
    {
      tv.fillRect(5, 70, 6 * 12, 8, 0x00);
      abnormalResetLabel = nullptr;
    }
  }

  // Heap health log (Serial): a steadily falling "min" points to a leak
  static unsigned long lastHeapLogMs = 0;
  if (now - lastHeapLogMs >= 30000)
  {
    lastHeapLogMs = now;
    Serial.printf("[HEAP] up=%lus free=%u min=%u largest=%u\n", now / 1000UL,
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                  (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
                  (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
  }

  // Clear reset message after 5 seconds
  if (resetPrintTime && now - resetPrintTime >= 5000)
  {
    tv.fillRect(WARNING_X + 40, WARNING_Y + 50, 66, 8, 0x00);
    resetPrintTime = 0;
  }

  tv.waitForFrame();
  now = millis(); // Refresh timestamp after V-Sync wait; CAN RX is drained continuously on Core 0

  if (now - lastBlinkTime >= blinkInterval)
  {
    lowBlinkState = !lowBlinkState;

    if (lowBlinkState == true)
    {
      fuel_run = true;
      cool_run = true;
      oil_on = true;
      temp_on = true;
    }
    lastBlinkTime = now;
  }
  int over_speed_blink_interval =
      lowBlinkState2 ? over_speed_on : over_speed_off;
  if (now - lastBlinkTime2 >= over_speed_blink_interval)
  {
    lowBlinkState2 = !lowBlinkState2;
    if (lowBlinkState2 == true && counter < 10)
    {
      counter++;
    }
    lastBlinkTime2 = now;
  }

  portENTER_CRITICAL(&dataMux);
  int local_ads_fuel = ads_fuel;
  float local_voltage_filtered = voltage_filtered;
  float local_current_A_filtered = current_A_filtered;
  portEXIT_CRITICAL(&dataMux);

  fuel_in_temporary = local_ads_fuel; // Pre-fetched value from regulatorTask
  if (fuel_in_temporary < 22000 && fuel_in_temporary > 1200)
  {
    raw = fuel_in_temporary;
    if (!fuel_level_valid)
    {
      smoothVal = (float)raw;
      filtered = raw;
      lastValue = raw;
      fuel_level_valid = true;
    }
  }

  if (fuel_level_valid)
  {
    int delta = abs(raw - lastValue);
    if (delta <= 2000)
    { // sample accepted
      filtered = raw;
      goodSamples++;
    }
    else
    {
      filtered = lastValue; // sample rejected, set it to previous good value
      badSamples++;
    }
    if (now - last_fuel_correction >= 15000)
    { // sample error correction
      if (goodSamples < badSamples)
      {
        filtered = raw;
      }
      goodSamples = 0;
      badSamples = 0;
      last_fuel_correction = now;
    }
    lastValue = filtered;
    smoothVal =
        0.001f * filtered +
        (1.0f - 0.001f) * smoothVal; // Exponential moving average for smoothing
    fuel_percent_f = constrain(getFuelPercent(smoothVal), 0.0f, 100.0f);
    percent = (int)lroundf(fuel_percent_f);
  }

  //-------------------- Coolant level
  coolant_level = digitalReadFast(coolant_level_pin);

  //================================ Read data from engine MCU
  //==============================================//
  // CAN frames (0x02 @ 50Hz, 0x05 @ 50Hz, 0x04 @ 20Hz, 0x06 @ 10Hz) are drained continuously on Core 0 by CanRxTask.
  // Front MCU is only powered in POS2 (IGN) and driving states; force 0 km/h when unpowered or timed out.
  bool frontMcuSpeedValid = isCanActiveState(currentState) &&
                            isCanPacketFresh(now, lastFastPacketTime, FRONT_MCU_CAN_TIMEOUT_MS);
  spd_l = frontMcuSpeedValid ? constrain((int)spd_t, 0, 220) : 0;
  int spd_in = spd_l;

  // --------------- filter SPD ----------------
  if (lastTime == 0 || !frontMcuSpeedValid)
  {
    spd = spd_in;
  }
  else
  {
    if (abs(spd_in - last_spd) <= 10)
    { // sample accepted
      spd = spd_in;
      goodSamples2++;
    }
    else
    {
      spd = last_spd; // sample rejected, set it to previous good value
      badSamples2++;
    }
    if (now - last_spd_correction >= 5000)
    { // sample error correction
      if (goodSamples2 < badSamples2)
      {
        spd = spd_in;
      }
      goodSamples2 = 0;
      badSamples2 = 0;
      last_spd_correction = now;
    }
  }

  if (spd != last_spd || lastTime == 0)
  {
    // Clear previous speed number area (prevents character overlap with custom fonts)
    tv.fillRect(80, 149, 85, 38, 0x00);

    tv.setFont(&FreeSansBold24pt7b);
    tv.setTextColor(0xFF);
    tv.setTextSize(1);
    char spdStr[6];
    snprintf(spdStr, sizeof(spdStr), "%d", spd);

    int16_t x1, y1;
    uint16_t w, h;
    tv.getTextBounds(spdStr, 0, 0, &x1, &y1, &w, &h);
    int xPos = 160 - w;      // Right-align against x = 160
    tv.setCursor(xPos, 184); // Baseline at y = 184
    tv.print(spdStr);

    tv.setFont(); // Reset to default font for remaining UI elements
    last_spd = spd;
  }

  // --- Display voltage & current ---
  static unsigned long lastMetricsUpdateTime = 0;
  if (now - lastMetricsUpdateTime >= DISPLAY_METRICS_UPDATE_MS)
  {
    // Voltage
    float volts = local_voltage_filtered;
    char bufV[10];
    snprintf(bufV, sizeof(bufV), "%5.1fV", volts);
    static char last_bufV[10] = "";
    if (strcmp(bufV, last_bufV) != 0)
    {
      tv.setCursor(5, 40);
      tv.setTextColor(0xFF, 0x00);
      tv.print(bufV);
      strcpy(last_bufV, bufV);
    }

    // Current
    float current_d = local_current_A_filtered;
    char bufI[10];
    snprintf(bufI, sizeof(bufI), "%5.0fA", current_d);
    static char last_bufI[10] = "";
    if (strcmp(bufI, last_bufI) != 0)
    {
      tv.setCursor(5, 55);
      tv.setTextColor(0xFF, 0x00);
      tv.print(bufI);
      strcpy(last_bufI, bufI);
    }

    // Injector Duty Cycle (%)
    char bufInj[16];
    snprintf(bufInj, sizeof(bufInj), "INJ:%4.1f%% ", live_inj_duty_cycle);
    static char last_bufInj[16] = "";
    if (strcmp(bufInj, last_bufInj) != 0)
    {
      tv.setCursor(FUEL_X + FUEL_WIDTH + 150, FUEL_Y + 20);
      tv.setTextColor(0xFF, 0x00);
      tv.print(bufInj);
      strcpy(last_bufInj, bufInj);
    }

    lastMetricsUpdateTime = now;
  }

  // --- Fast Vacuum (PSI) Display & Instant ECO Warning Indicator ---
  // Vacuum is constantly visible and NEVER disappears.
  // Red "ECO" warning comes and goes instantly when vacuum drops below threshold without delays.
  static bool eco_warning_drawn = false;
  static bool vac_drawn = false;
  static char last_bufVac[16] = "";

  if (last_clear < 6)
  {
    vac_drawn = false;
    eco_warning_drawn = false;
    last_bufVac[0] = '\0';
  }

  unsigned long local_last_vac = lastVacPacketTime;
  bool vac_valid = isCanActiveState(currentState) &&
                   (local_last_vac != 0) &&
                   isCanPacketFresh(now, local_last_vac, FRONT_MCU_TIMEOUT_MS) &&
                   !map_sensor_fault && isBaroValid();

  // Vacuum PSI readout: constantly visible, shows "VAC: --- " if disconnected or invalid
  char bufVac[16];
  if (vac_valid)
  {
    snprintf(bufVac, sizeof(bufVac), "VAC:%4.1fpsi", vacuum_psi);
  }
  else
  {
    snprintf(bufVac, sizeof(bufVac), "VAC: --- ");
  }
  if (strcmp(bufVac, last_bufVac) != 0 || !vac_drawn)
  {
    tv.setCursor(VACUUM_DISPLAY_X, VACUUM_DISPLAY_Y);
    tv.setTextColor(0xFF, 0x00);
    tv.print(bufVac);
    strcpy(last_bufVac, bufVac);
    vac_drawn = true;
  }

  // Red "ECO" warning with hysteresis to eliminate flickering near threshold
  static bool eco_warning_latched = false;
  float eff_eco_thresh = getEcoWarningVacPsi();
  float eff_eco_clear = getEcoClearVacPsi();

  if (MAP_SENSOR_ENABLED && vac_valid && (currentState == STATE_RUNNING || rpm >= ENGINE_STARTED_RPM))
  {
    if (!eco_warning_latched && vacuum_psi < eff_eco_thresh)
    {
      eco_warning_latched = true;
    }
    else if (eco_warning_latched && vacuum_psi >= eff_eco_clear)
    {
      eco_warning_latched = false;
    }
  }
  else
  {
    eco_warning_latched = false;
  }
  eco_warning_active = eco_warning_latched;
  if (eco_warning_active != eco_warning_drawn)
  {
    if (eco_warning_active)
    {
      tv.setCursor(ECO_INDICATOR_X, ECO_INDICATOR_Y);
      tv.setTextColor(0xE0, 0x00); // Red
      tv.setTextSize(2);
      tv.print("ECO");
      tv.setTextSize(1);
      eco_warning_drawn = true;
    }
    else
    {
      tv.fillRect(ECO_INDICATOR_X, ECO_INDICATOR_Y, 38, 16, 0x00);
      eco_warning_drawn = false;
    }
  }

  // --- Display cranking amps & battery internal resistance briefly after
  // starting ---
  static float peak_crank_current = 0.0f;
  static SystemState lastStateDisplay = STATE_SLEEP;
  static unsigned long startedRunningTime = 0;
  static bool was_cranking_amps_drawn = false;

  static float resting_voltage = 12.6f;
  static float v_rest_frozen = 12.6f;
  static float crank_r_sum = 0.0f;
  static int crank_r_count = 0;

  // Track resting voltage prior to cranking when current load is low
  if (currentState != STATE_CRANKING && currentState != STATE_RUNNING)
  {
    if (abs(local_current_A_filtered) < 15.0f &&
        local_voltage_filtered > 10.0f)
    {
      resting_voltage =
          0.05f * local_voltage_filtered + 0.95f * resting_voltage;
    }
  }

  if (currentState == STATE_CRANKING && lastStateDisplay != STATE_CRANKING)
  {
    peak_crank_current = 0.0f;
    v_rest_frozen =
        (resting_voltage >= 10.0f) ? resting_voltage : local_voltage_filtered;
    crank_r_sum = 0.0f;
    crank_r_count = 0;
  }
  if (currentState == STATE_CRANKING)
  {
    if (local_current_A_filtered < peak_crank_current)
    {
      peak_crank_current = local_current_A_filtered;
    }

    // Accumulate internal resistance samples during heavy discharge (> 30A)
    float discharge_current = -local_current_A_filtered;
    if (discharge_current > 30.0f)
    {
      float v_drop = v_rest_frozen - local_voltage_filtered;
      if (v_drop > 0.05f)
      {
        float r_inst_mOhm = (v_drop / discharge_current) * 1000.0f;
        if (r_inst_mOhm >= 0.5f && r_inst_mOhm <= 100.0f)
        {
          crank_r_sum += r_inst_mOhm;
          crank_r_count++;
        }
      }
    }
  }

  if (currentState == STATE_RUNNING && lastStateDisplay == STATE_CRANKING)
  {
    startedRunningTime = now;

    // Calculate this crank event's average internal resistance
    float r_event = 0.0f;
    if (crank_r_count > 0)
    {
      r_event = crank_r_sum / (float)crank_r_count;
    }
    else if (peak_crank_current < -30.0f)
    {
      float v_drop = v_rest_frozen - local_voltage_filtered;
      if (v_drop > 0.05f)
      {
        r_event = (v_drop / (-peak_crank_current)) * 1000.0f;
      }
    }

    // Compound over time using Exponential Moving Average across crank events
    if (r_event >= 0.5f && r_event <= 100.0f)
    {
      if (compounded_r_int < 0.5f || isnan(compounded_r_int))
      {
        compounded_r_int = r_event;
      }
      else
      {
        compounded_r_int = 0.30f * r_event + 0.70f * compounded_r_int;
      }

      // Preferences write removed, saved in enterPowerDownSleep()
    }
  }

  bool show_crank_amps =
      (currentState == STATE_RUNNING && startedRunningTime != 0 &&
       (now - startedRunningTime < 5000));

  if (show_crank_amps)
  {
    if (!was_cranking_amps_drawn)
    {
      tv.setCursor(5, 70);
      tv.setTextColor(0xFF, 0x00);
      char bufCA[24];
      float ca_val = (peak_crank_current < 0.0f) ? -peak_crank_current : 0.0f;
      if (compounded_r_int > 0.1f)
      {
        snprintf(bufCA, sizeof(bufCA), "CRK %4.0fA %4.1fm", ca_val,
                 compounded_r_int);
      }
      else
      {
        snprintf(bufCA, sizeof(bufCA), "CRK %4.0fA", ca_val);
      }
      tv.print(bufCA);
      was_cranking_amps_drawn = true;
    }
  }
  else if (was_cranking_amps_drawn)
  {
    tv.setCursor(5, 70);
    tv.setTextColor(0xFF, 0x00);
    tv.print("                   "); // Erase with spaces
    was_cranking_amps_drawn = false;
  }

  lastStateDisplay = currentState;

  // display rpm
  int current_rpm = (int)rpm;
  static int last_drawn_rpm = -1;
  static unsigned long lastRpmUpdateTime = 0;
  if (now - lastRpmUpdateTime >= RPM_UPDATE_INTERVAL_MS)
  {
    if (current_rpm != last_drawn_rpm)
    {

      tv.setCursor(FUEL_X + FUEL_WIDTH + 150, FUEL_Y + 60);
      tv.setTextColor(0xFF, 0x00);
      tv.setTextSize(2);
      char rpmStr[6];
      snprintf(rpmStr, sizeof(rpmStr), "%5d", current_rpm);
      tv.print(rpmStr);
      tv.setTextSize(1);
      last_drawn_rpm = current_rpm;
    }
    lastRpmUpdateTime = now;
  }

  // `temp_out` is converted from `raw2` directly inside CanRxTask on Core 0 upon 0x06 arrival,
  // and `rpm` is zeroed atomically on Core 0 when CAN times out.
  if (!isCanPacketFresh(now, lastFastPacketTime, FRONT_MCU_CAN_TIMEOUT_MS))
  {
    spd = 0;
  }

  // =========================== Send Dynamic data to display
  // ============================
  if (now - lastTime >= FUEL_UPDATE_INTERVAL_MS)
  {
    fill = map(percent, 0, 100, 0, FUEL_HEIGHT - 4); // ----------for fuel gauge
    fill = constrain(fill, 0, FUEL_HEIGHT - 4);
    v = FUEL_HEIGHT - 2 + FUEL_Y - fill;
    if (percent <= LOW_FUEL_LEVEL)
    {
      fuel_color = 0xE0;
    }
    else
    {
      fuel_color = 0xFF;
    }
    if (last_v != v || lastTime == 0)
    {
      tv.fillRect(FUEL_X + 2, FUEL_Y + 2, FUEL_WIDTH - 4, v - (FUEL_Y + 2),
                  0x00);
      tv.fillRect(FUEL_X + 2, v, FUEL_WIDTH - 4, fill, fuel_color);
      tv.setCursor(FUEL_X, FUEL_Y - 10);
      tv.setTextColor(
          0xFF,
          0x00); // White text on black background to erase old digits cleanly
      char buf[4];
      snprintf(buf, sizeof(buf), "%3d", percent);
      tv.print(buf);
    }

    //--------------------------------------------------------------------------
    bool temp_valid = isCanActiveState(currentState) &&
                      isCanPacketFresh(now, lastEnvPacketTime, FRONT_MCU_TIMEOUT_MS);
    int effective_temp = temp_valid ? temp_out : 40;
    fill2 = map(effective_temp, 40, 120, 0, TEMP_HEIGHT); // ----------for temp gauge
    int tick_y = 0;
    fill2 = constrain(fill2, 0, TEMP_HEIGHT);
    t = TEMP_HEIGHT + TEMP_Y - fill2;
    int disp_key = temp_valid ? temp_out : -2; // -2 = "---" while Front MCU is unpowered/offline
    static int last_temp_disp = -1;
    if (last_clear < 6)
    {
      last_temp_disp = -1;
    }
    if (last_t != t || last_temp_disp != disp_key || lastTime == 0)
    {
      // Erase previous needle only
      tv.fillRect(TEMP_X - 5, last_t - 2, TEMP_VALUE_TICK_WIDTH,
                  TEMP_VALUE_TICK_HEIGHT, 0x00);
      tv.drawFastVLine(TEMP_X, TEMP_Y, TEMP_HEIGHT, 0xFF);
      for (int i = 0; i <= 100; i += 25) //---------- draw ticks
      {
        tick_y = TEMP_Y + i;
        tv.fillRect(TEMP_X - 2, tick_y, TEMP_TICKS_WIDTH, TEMP_TICKS_HEIGHT,
                    0xFF);
      }
      if (temp_valid)
      {
        // draw temp_value tick only when Front MCU temperature signal is live
        tv.fillRect(TEMP_X - 5, t - 2, TEMP_VALUE_TICK_WIDTH,
                    TEMP_VALUE_TICK_HEIGHT, 0xE0);
      }

      // Display digital temperature readout on top of temp gauge ("---" when Front MCU unpowered)
      tv.setCursor(TEMP_X - 8, TEMP_Y - 10);
      uint8_t temp_color = (temp_valid && temp_out >= OVERHEAT_TEMP_C) ? 0xE0 : 0xFF;
      tv.setTextColor(temp_color, 0x00);
      char bufTemp[4];
      if (temp_valid)
      {
        snprintf(bufTemp, sizeof(bufTemp), "%3d", temp_out);
      }
      else
      {
        snprintf(bufTemp, sizeof(bufTemp), "---");
      }
      tv.print(bufTemp);
      last_temp_disp = disp_key;
    }

    // --- Fuel Consumption Calculations ---
    // First pass after boot: lastTime == 0 would make elapsed = whole uptime
    float elapsed_sec = (lastTime == 0) ? (FUEL_UPDATE_INTERVAL_MS / 1000.0f)
                                        : ((now - lastTime) / 1000.0f);
    if (elapsed_sec <= 0.0f)
      elapsed_sec = 1.0f;

    // Measured warm-idle consumption (L/h), learned below while idling; used for Auto Start-Stop savings
    static float measured_idle_l_per_hr = BASELINE_IDLE_FUEL_L_PER_HR;

    // Atomically fetch and reset cumulative CAN 0x04 counters from Core 0 (lossless)
    portENTER_CRITICAL(&dataMux);
    uint32_t interval_inj_time_us = accumulated_inj_time_us;
    uint32_t interval_inj_pulses = accumulated_inj_pulses;
    uint32_t interval_spd_pulses = spd_delta_pulses;
    accumulated_inj_time_us = 0;
    accumulated_inj_pulses = 0;
    spd_delta_pulses = 0;
    portEXIT_CRITICAL(&dataMux);

    // --- Dead-time compensation ---
    // Injector dead-time is the solenoid opening delay where no fuel flows.
    // It varies with battery voltage: lower voltage = longer dead-time.
    float dead_time_us = getInjectorDeadTime(local_voltage_filtered);

    // Subtract dead-time from each physical pulse received (one injector)
    float corrected_inj_us = (float)interval_inj_time_us -
                             ((float)interval_inj_pulses * dead_time_us);
    if (corrected_inj_us < 0.0f)
      corrected_inj_us = 0.0f;

    // Fuel consumed during the interval (in liters)
    float fuel_consumed_liters = (corrected_inj_us / 1000000.0f) *
                                 (INJECTOR_FLOW_RATE_CC_MIN / 60.0f / 1000.0f) *
                                 (float)NUM_INJECTORS;

    total_fuel_liters += fuel_consumed_liters;

    if (dfco_cut_state == 1 && rpm > 0)
    {
      // Calculate fuel saved using the pre-cutoff net pulse width (fallback 1500us net)
      float net_saved_pulse_us = (last_active_inj_pulse_us >= 500.0f)
                                     ? last_active_inj_pulse_us
                                     : 1500.0f;
      float saved_inj_time_us =
          ((float)rpm / 120.0f) * net_saved_pulse_us * elapsed_sec;
      float fuel_saved_interval =
          (saved_inj_time_us / 1000000.0f) *
          (INJECTOR_FLOW_RATE_CC_MIN / 60.0f / 1000.0f) * (float)NUM_INJECTORS;
      total_fuel_saved_liters += fuel_saved_interval;
    }
    else if (currentState == STATE_AUTO_STOP)
    {
      // Calculate fuel saved during Auto Start-Stop based on the measured warm-idle rate
      float fuel_saved_interval =
          (measured_idle_l_per_hr / 3600.0f) * elapsed_sec;
      total_fuel_saved_liters += fuel_saved_interval;
    }

    // Update trip distance from cumulative speed sensor pulses (lossless, exact)
    float interval_dist_km = (float)interval_spd_pulses / PULSES_PER_KM_F;
    total_distance_km += interval_dist_km;

    float speed_val = (float)spd;
    // Standard automotive threshold: calculate L/100km only when moving >= 3 km/h
    bool is_moving = (speed_val >= 3.0f);

    // Calculate instant consumption
    if (is_moving)
    {
      float dist_for_calc = (interval_dist_km > 0.0001f)
                                ? interval_dist_km
                                : ((speed_val / 3600.0f) * elapsed_sec);
      // inst_val in L/100km, clamped to 99.9 to prevent text overflow
      inst_val = (fuel_consumed_liters / dist_for_calc) * 100.0f;
      if (inst_val > 99.9f)
        inst_val = 99.9f;
    }
    else
    {
      // stationary consumption in L/h
      inst_val = (fuel_consumed_liters / elapsed_sec) * 3600.0f;

      // Learn warm-idle consumption under the same conditions Auto Start-Stop engages in
      if (currentState == STATE_RUNNING && injector_state == 0 &&
          th_switch_state == 1 && temp_out >= AUTO_STOP_MIN_TEMP_C &&
          rpm >= VAC_LEAK_MIN_RPM && rpm <= VAC_LEAK_MAX_RPM &&
          inst_val >= 0.3f && inst_val <= 3.0f)
      {
        measured_idle_l_per_hr = 0.02f * inst_val + 0.98f * measured_idle_l_per_hr; // ~50 s time constant
      }
    }

    // Calculate average consumption in L/100km
    if (total_distance_km > 0.001)
    {
      avg_l_100km = (float)((total_fuel_liters / total_distance_km) * 100.0);
    }
    else
    {
      avg_l_100km = 0.0f;
    }

    // Render fuel and distance metrics to screen
    tv.setTextSize(2);
    tv.setTextColor(0xFF, 0x00);
    char bufInst[20];
    if (is_moving)
    {
      snprintf(bufInst, sizeof(bufInst), "%4.1f L/100km", inst_val);
    }
    else
    {
      snprintf(bufInst, sizeof(bufInst), "%4.1f L/h", inst_val);
    }
    tv.fillRect(90, 210, 160, 16, 0x00); // Clear previous instant readout
    tv.setCursor(90, 210);
    tv.print(bufInst);
    tv.setTextSize(1);

    char bufAvg[20];
    snprintf(bufAvg, sizeof(bufAvg), "AVG:%5.1f L/100km", avg_l_100km);
    tv.fillRect(FUEL_X + FUEL_WIDTH + 5, FUEL_Y, 120, 8,
                0x00); // Clear previous AVG readout
    tv.setCursor(FUEL_X + FUEL_WIDTH + 5, FUEL_Y);
    tv.setTextColor(0xFF, 0x00);
    tv.print(bufAvg);

    char bufTrip[20];
    snprintf(bufTrip, sizeof(bufTrip), "TRIP:%6.1f km", total_distance_km);
    tv.fillRect(FUEL_X + FUEL_WIDTH + 5, FUEL_Y + 20, 120, 8,
                0x00); // Clear previous TRIP readout
    tv.setCursor(FUEL_X + FUEL_WIDTH + 5, FUEL_Y + 20);
    tv.setTextColor(0xFF, 0x00);
    tv.print(bufTrip);

    char bufUsed[20];
    snprintf(bufUsed, sizeof(bufUsed), "USED:%5.1f L",
             total_fuel_liters);
    tv.fillRect(FUEL_X + FUEL_WIDTH + 5, FUEL_Y + 40, 120, 8,
                0x00); // Clear previous USED readout
    tv.setCursor(FUEL_X + FUEL_WIDTH + 5, FUEL_Y + 40);
    tv.setTextColor(0xFF, 0x00);
    tv.print(bufUsed);

    char bufSaved[16];
    if (total_fuel_saved_liters < 1.0f)
    {
      snprintf(bufSaved, sizeof(bufSaved), "SAVED:%5.3f L",
               total_fuel_saved_liters);
    }
    else
    {
      snprintf(bufSaved, sizeof(bufSaved), "SAVED:%5.2f L",
               total_fuel_saved_liters);
    }
    tv.fillRect(FUEL_X + FUEL_WIDTH + 130, FUEL_Y, 80, 8, 0x00); // Clear previous SAVED readout
    tv.setCursor(FUEL_X + FUEL_WIDTH + 130, FUEL_Y);
    tv.setTextColor(0xFF, 0x00);
    tv.print(bufSaved);

    char bufRem[20];
    if (fuel_level_valid && avg_l_100km > 0.1f &&
        total_distance_km >= RANGE_MIN_TRIP_KM)
    {
      float rem_fuel_l = (fuel_percent_f / 100.0f) * FUEL_TANK_CAPACITY_LITERS;
      float rem_km = (rem_fuel_l / avg_l_100km) * 100.0f;
      rem_km = constrain(rem_km, 0.0f, 999.0f);
      snprintf(bufRem, sizeof(bufRem), "REM:%4.0fkm", rem_km);
    }
    else
    {
      snprintf(bufRem, sizeof(bufRem), "REM:---km");
    }
    tv.fillRect(FUEL_X + FUEL_WIDTH + 150, FUEL_Y + 40, 60, 8,
                0x00); // Clear previous REM readout
    tv.setCursor(FUEL_X + FUEL_WIDTH + 150, FUEL_Y + 40);
    tv.setTextColor(0xFF, 0x00);
    tv.print(bufRem);

    lastTime = now;
    last_v = v;
    last_t = t;
  }
  oil_level = (int)oil_level_t;
  warnings(now);
  processPushStart(now);

  // Mirror RUNNING state into reset-surviving RTC memory for crash recovery in setup()
  if (currentState == STATE_RUNNING)
  {
    rtc_run_marker = RTC_RUN_MARKER_MAGIC;
    rtc_run_marker_inv = ~RTC_RUN_MARKER_MAGIC;
  }
  else if (rtc_run_marker != 0)
  {
    rtc_run_marker = 0;
    rtc_run_marker_inv = 0;
  }
  // debug
  // Serial.print("current: ");
  // Serial.println(current_A_filtered);
  esp_task_wdt_reset();
}
