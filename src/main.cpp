#include "config.h"
#include "globals.h"
#include "display.h"
#include "regulator.h"
#include "pushstart.h"
#include "can_comm.h"
#include "fuel.h"

//=================== setup ===============//
void setup()
{
  // 1. Immediately turn on the switched rails
  pinMode(PIN_5V_GATE, OUTPUT);
  digitalWrite(PIN_5V_GATE, HIGH); // Enable Relay

  pinMode(PIN_3V3_DIGITAL_GATE, OUTPUT);
  digitalWrite(PIN_3V3_DIGITAL_GATE, HIGH); // Power 3.3V pull-ups/level shifter

  delay(30); // Allow voltage rails to stabilize

  // Release any GPIO holds from previous deep sleep
  gpio_hold_dis((gpio_num_t)PIN_RELAY_ACC);
  gpio_hold_dis((gpio_num_t)PIN_RELAY_IGN);
  gpio_hold_dis((gpio_num_t)PIN_RELAY_START);
  gpio_hold_dis((gpio_num_t)PIN_5V_GATE);
  gpio_hold_dis((gpio_num_t)field_relay_pin);
  gpio_hold_dis((gpio_num_t)buzzer_pin);
  gpio_hold_dis((gpio_num_t)PIN_3V3_DIGITAL_GATE);
  gpio_deep_sleep_hold_dis();

  setupPushStartPins();

  // Initialize system state to Standby with 2-minute sleep timeout on all
  // boots/resets
  currentState = STATE_STANDBY;
  standbyStartTime = millis();
  regulatorTaskRunning = true;
  stoppedToAcc = false;

  recoverI2CBus(21, 22);
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
  WiFi.mode(WIFI_OFF);
  WiFi.disconnect(true);
  btStop();
  tv.begin();
  tv.copyAfterSwap = true;

  if (rtc_trip_magic != RTC_TRIP_MAGIC_KEY || isnan(total_fuel_liters) ||
      isnan(total_distance_km) || isnan(total_fuel_saved_liters))
  {
    Preferences prefs;
    prefs.begin("trip_data", true); // Open in read-only mode
    total_fuel_liters = prefs.getFloat("fuel", 0.0f);
    total_distance_km = prefs.getFloat("dist", 0.0f);
    compounded_r_int = prefs.getFloat("r_int", 0.0f);
    total_fuel_saved_liters = prefs.getFloat("saved", 0.0f);
    prefs.end();

    if (isnan(total_fuel_liters))
      total_fuel_liters = 0.0f;
    if (isnan(total_distance_km))
      total_distance_km = 0.0f;
    if (isnan(compounded_r_int) || compounded_r_int < 0.0f)
      compounded_r_int = 0.0f;
    if (isnan(total_fuel_saved_liters) || total_fuel_saved_liters < 0.0f)
      total_fuel_saved_liters = 0.0f;

    rtc_trip_magic = RTC_TRIP_MAGIC_KEY;
  }

  pinModeFast(buzzer_pin, OUTPUT);
  pinModeFast(coolant_level_pin, INPUT);
  pinMode(field_relay_pin, OUTPUT);
  digitalWriteFast(field_relay_pin, LOW); // Start with field relay off
  Serial.begin(250000);
  delay(50); // Let UART stabilize

  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); // Match transmitter
  mcp2515.setNormalOneShotMode();
  esp_task_wdt_deinit();      // De-init default core WDT config
  esp_task_wdt_init(1, true); // 1s timeout with panic=true
  esp_task_wdt_add(nullptr);  // Add current loop task
  ledcSetup(0, 400, 10);      // channel, freq, resolution
  ledcAttachPin(FIELD_PIN, 0);

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
  lastPacketTime = millis(); // Initialize to avoid immediate timeout warning
}

// ===================== main loop ======================//
void loop()
{
  // uint32_t start = micros();
  unsigned long now = millis();

  //============= send health signal to front MCU ==================
  static unsigned long lastCanSendTimeMs = 0;
  if (now - lastCanSendTimeMs >= CAN_HEALTH_SEND_INTERVAL_MS)
  {
    bool regulator_ok =
        (regulatorTaskHandle == NULL) || (now - last_regulator_heartbeat < REGULATOR_HEARTBEAT_TIMEOUT_MS);
    health_state = regulator_ok ? 100 : 0;

    canMsgTx.can_id = 0x03;
    canMsgTx.can_dlc = 8;
    canMsgTx.data[0] = health_state;
    canMsgTx.data[1] = 0;
    canMsgTx.data[2] = 0;
    canMsgTx.data[3] = 0;
    canMsgTx.data[4] = 0;
    canMsgTx.data[5] = 0;
    canMsgTx.data[6] = 0;
    canMsgTx.data[7] = 0;
    mcp2515.sendMessage(&canMsgTx);
    lastCanSendTimeMs = now;
  }

  if (last_clear < 6)
  {
    tv.fillScreen(0x00);
    drawStaticGauge();
    last_clear++;
  }

  // Clear reset message after 5 seconds
  if (resetPrintTime && now - resetPrintTime >= 5000)
  {
    tv.fillRect(WARNING_X + 40, WARNING_Y + 50, 66, 8, 0x00);
    resetPrintTime = 0;
  }

  tv.waitForFrame();

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

  fuel_in_temporary =
      local_ads_fuel; // Use pre-fetched value from regulatorTask
  if (fuel_in_temporary < /*2190 max*/ 22000 &&
      fuel_in_temporary > 1200 /*1326 min*/)
  {
    raw = fuel_in_temporary;
  }
  if (lastTime == 0)
  {
    smoothVal = (float)raw;
    filtered = raw;
    lastValue = raw;
  }
  else
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
  }
  smoothVal =
      0.001f * filtered +
      (1.0f - 0.001f) * smoothVal; // Exponential moving average for smoothing
  percent = (int)getFuelPercent(smoothVal);
  percent = constrain(percent, 0, 100);

  //-------------------- Coolant level
  coolant_level = digitalReadFast(coolant_level_pin);

  //================================ Read data from engine MCU
  //==============================================//

  // CAN drain (catch anything that arrived during frame sync)
  checkCanErrors();
  drainCanRxBuffer(now);

  portENTER_CRITICAL(&dataMux);
  rpm = new_rpm;
  portEXIT_CRITICAL(&dataMux);

  // Front MCU sends km/h directly (pulse-counted from instrument cluster VSS)
  spd_l = constrain((int)spd_t, 0, 220);
  spd = spd_l;

  if (spd != last_spd || lastTime == 0)
  {
    tv.setCursor(72, 150);
    tv.setTextColor(0xFF, 0x00);
    tv.setTextSize(5);
    char spdStr[4];
    snprintf(spdStr, sizeof(spdStr), "%3d", spd);
    tv.print(spdStr);
    tv.setTextSize(1);

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

  temp_out = map((int)raw2, 250, 950, 40, 120);
  temp_out = constrain(temp_out, 40, 120);

  if (now - lastPacketTime > FRONT_MCU_CAN_TIMEOUT_MS)
  {
    rpm = 0;
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
    fill2 = map(temp_out, 40, 120, 0, TEMP_HEIGHT); // ----------for temp gauge
    int tick_y = 0;
    fill2 = constrain(fill2, 0, TEMP_HEIGHT);
    t = TEMP_HEIGHT + TEMP_Y - fill2;
    if (last_t != t || lastTime == 0)
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
      // draw temp_value tick
      tv.fillRect(TEMP_X - 5, t - 2, TEMP_VALUE_TICK_WIDTH,
                  TEMP_VALUE_TICK_HEIGHT, 0xE0);
    }

    // --- Fuel Consumption Calculations ---
    float elapsed_sec = (now - lastTime) / 1000.0f;
    if (elapsed_sec <= 0.0f)
      elapsed_sec = 1.0f;

    // --- Dead-time compensation ---
    // Injector dead-time is the solenoid opening delay where no fuel flows.
    // It varies with battery voltage: lower voltage = longer dead-time.
    float dead_time_us = getInjectorDeadTime(local_voltage_filtered);

    // Subtract dead-time from each physical pulse received (one injector)
    float corrected_inj_us = (float)accumulated_inj_time_us -
                             ((float)accumulated_inj_pulses * dead_time_us);
    if (corrected_inj_us < 0.0f)
      corrected_inj_us = 0.0f;

    // Fuel consumed during the interval (in liters)
    float fuel_consumed_liters = (corrected_inj_us / 1000000.0f) *
                                 (INJECTOR_FLOW_RATE_CC_MIN / 60.0f / 1000.0f) *
                                 (float)NUM_INJECTORS;

    // Track the actual net injection pulse width (effective fuel delivery duration per pulse)
    // right before injDisable engages.
    static float last_active_inj_pulse_us = 1500.0f; // Net pulse width fallback (~1.5ms net open time)
    if (injector_state == 0 && accumulated_inj_time_us > 0 && accumulated_inj_pulses > 0)
    {
      float current_net_pulse_us = corrected_inj_us / (float)accumulated_inj_pulses;
      if (current_net_pulse_us > 0.0f)
      {
        last_active_inj_pulse_us = current_net_pulse_us;
      }
    }
    accumulated_inj_time_us = 0; // Reset accumulator
    accumulated_inj_pulses = 0;  // Reset pulse accumulator

    total_fuel_liters += fuel_consumed_liters;

    if (injector_state == 1 && rpm > 0)
    {
      // Calculate fuel saved using pre-cutoff net pulse width
      float corrected_pulse = last_active_inj_pulse_us;

      float saved_inj_time_us =
          ((float)rpm / 120.0f) * corrected_pulse * elapsed_sec;
      float fuel_saved_interval =
          (saved_inj_time_us / 1000000.0f) *
          (INJECTOR_FLOW_RATE_CC_MIN / 60.0f / 1000.0f) * (float)NUM_INJECTORS;
      total_fuel_saved_liters += fuel_saved_interval;
    }

    // Update trip distance (speed is in km/h, convert to km/sec and multiply by
    // elapsed seconds)
    float speed_val = (float)spd;
    total_distance_km += (speed_val / 3600.0f) * elapsed_sec;

    // Calculate instant consumption
    if (speed_val > 0.0f)
    {
      // inst_val in L/100km
      inst_val =
          (fuel_consumed_liters / ((speed_val / 3600.0f) * elapsed_sec)) *
          100.0f;
    }
    else
    {
      // stationary consumption in L/h
      inst_val = (fuel_consumed_liters / elapsed_sec) * 3600.0f;
    }

    // Calculate average consumption in L/100km
    if (total_distance_km > 0.001f)
    {
      avg_l_100km = (total_fuel_liters / total_distance_km) * 100.0f;
    }
    else
    {
      avg_l_100km = 0.0f;
    }

    // Render fuel and distance metrics to screen
    tv.setTextSize(2);
    tv.setTextColor(0xFF, 0x00);
    char bufInst[20];
    if (speed_val > 0.0f)
    {
      snprintf(bufInst, sizeof(bufInst), "%5.1f L/100Km  ", inst_val);
    }
    else
    {
      snprintf(bufInst, sizeof(bufInst), "%5.1f L/h       ", inst_val);
    }
    tv.fillRect(90, 210, 165, 16, 0x00); // Clear previous instant readout
    tv.setCursor(90, 210);
    tv.print(bufInst);
    tv.setTextSize(1);

    char bufAvg[20];
    snprintf(bufAvg, sizeof(bufAvg), "AVG:%5.1f L/100km  ", avg_l_100km);
    tv.fillRect(FUEL_X + FUEL_WIDTH + 5, FUEL_Y, 120, 8,
                0x00); // Clear previous AVG readout
    tv.setCursor(FUEL_X + FUEL_WIDTH + 5, FUEL_Y);
    tv.setTextColor(0xFF, 0x00);
    tv.print(bufAvg);

    char bufTrip[20];
    snprintf(bufTrip, sizeof(bufTrip), "TRIP:%6.1f km     ", total_distance_km);
    tv.fillRect(FUEL_X + FUEL_WIDTH + 5, FUEL_Y + 20, 120, 8,
                0x00); // Clear previous TRIP readout
    tv.setCursor(FUEL_X + FUEL_WIDTH + 5, FUEL_Y + 20);
    tv.setTextColor(0xFF, 0x00);
    tv.print(bufTrip);

    char bufUsed[20];
    snprintf(bufUsed, sizeof(bufUsed), "USED:%5.1f L       ",
             total_fuel_liters);
    tv.fillRect(FUEL_X + FUEL_WIDTH + 5, FUEL_Y + 40, 120, 8,
                0x00); // Clear previous USED readout
    tv.setCursor(FUEL_X + FUEL_WIDTH + 5, FUEL_Y + 40);
    tv.setTextColor(0xFF, 0x00);
    tv.print(bufUsed);

    char bufSaved[14];
    if (total_fuel_saved_liters < 1.0f)
    {
      snprintf(bufSaved, sizeof(bufSaved), "SAVED:%5.3f L       ",
               total_fuel_saved_liters);
    }
    else
    {
      snprintf(bufSaved, sizeof(bufSaved), "SAVED:%5.2f L       ",
               total_fuel_saved_liters);
    }
    // tv.setCursor(FUEL_X + FUEL_WIDTH + 150, FUEL_Y + 20);
    tv.fillRect(FUEL_X + FUEL_WIDTH + 130, FUEL_Y, 80, 8, 0x00); // Clear previous SAVED readout
    tv.setCursor(FUEL_X + FUEL_WIDTH + 130, FUEL_Y);
    // tv.setCursor(FUEL_X + FUEL_WIDTH + 5, FUEL_Y + 60);
    tv.setTextColor(0xFF, 0x00);
    tv.print(bufSaved);

    char bufRem[20];
    if (avg_l_100km > 0.001f)
    {
      float rem_fuel_l = (percent / 100.0f) * FUEL_TANK_CAPACITY_LITERS;
      float rem_km = (rem_fuel_l / avg_l_100km) * 100.0f;
      snprintf(bufRem, sizeof(bufRem), "REM:%4.0fkm ", rem_km);
    }
    else
    {
      snprintf(bufRem, sizeof(bufRem), "REM:---km ");
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

  esp_task_wdt_reset();
}
