#include "can_comm.h"
#include "fuel.h"

// ============================================================================
// ESP32 TWAI (built-in CAN 2.0B controller) + SN65HVD230 transceiver
// ----------------------------------------------------------------------------
// Bit timing: 500 kbps to match the Front MCU (MCP2515 @ 8 MHz, CAN_500KBPS).
// Core affinity: `canRxTask` is pinned to Core 0 and installs the TWAI driver
// there, binding TWAI hardware interrupts to Core 0 (away from Core 1's I2S/DAC
// composite video ISR) and feeding Core 0's `regulatorTask` with zero V-Sync lag.
// ============================================================================

static twai_message_t canMsgTx;
static volatile bool canInstalled = false;
static SemaphoreHandle_t canDriverMutex = NULL;
static TaskHandle_t canRxTaskHandle = NULL;
static volatile bool canRxTaskRunning = false;

static void ensureCanMutex()
{
  if (canDriverMutex == NULL)
  {
    canDriverMutex = xSemaphoreCreateMutex();
  }
}

bool canBegin()
{
  ensureCanMutex();
  xSemaphoreTake(canDriverMutex, portMAX_DELAY);

  if (canInstalled)
  {
    xSemaphoreGive(canDriverMutex);
    return true;
  }

  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)PIN_CAN_TX, (gpio_num_t)PIN_CAN_RX, TWAI_MODE_NORMAL);
  g_config.rx_queue_len = CAN_RX_QUEUE_LEN;
  g_config.tx_queue_len = CAN_TX_QUEUE_LEN;
  g_config.alerts_enabled = TWAI_ALERT_NONE; // State is polled in checkCanErrors()

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

  // Hardware acceptance filter: accept standard IDs 0x000..0x007 only (Front MCU uses 0x02/0x04).
  // Single-filter layout for std frames: ID in bits [31:21]; mask bit = 1 -> "don't care".
  // ID bits [10:3] must be 0, bits [2:0] + RTR + data bytes are don't-care.
  twai_filter_config_t f_config;
  f_config.acceptance_code = 0x00000000;
  f_config.acceptance_mask = 0x00FFFFFF;
  f_config.single_filter = true;

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK)
  {
    xSemaphoreGive(canDriverMutex);
    return false;
  }
  // Keep weak internal pull-up on CRX so an unplugged transceiver never floats or picks up EMI
  gpio_set_pull_mode((gpio_num_t)PIN_CAN_RX, GPIO_PULLUP_ONLY);
  if (twai_start() != ESP_OK)
  {
    twai_driver_uninstall();
    pinMode(PIN_CAN_TX, INPUT_PULLUP);
    xSemaphoreGive(canDriverMutex);
    return false;
  }
  canInstalled = true;
  xSemaphoreGive(canDriverMutex);
  return true;
}

void canEnd()
{
  ensureCanMutex();
  xSemaphoreTake(canDriverMutex, portMAX_DELAY);

  // Park CTX recessive (HIGH) and disconnect it from the TWAI peripheral output matrix
  // BEFORE stopping/uninstalling the driver so disabling the peripheral never glitches CTX low.
  pinMode(PIN_CAN_TX, INPUT_PULLUP);
  pinMode(PIN_CAN_RX, INPUT_PULLUP);

  if (canInstalled)
  {
    twai_stop(); // Transitions RUNNING -> STOPPED (no-op if already STOPPED or BUS_OFF)
    if (twai_driver_uninstall() == ESP_OK)
    {
      canInstalled = false;
    }
  }

  xSemaphoreGive(canDriverMutex);
}

bool canIsRunning()
{
  if (!canInstalled || canDriverMutex == NULL)
    return false;
  if (xSemaphoreTake(canDriverMutex, 0) != pdTRUE)
    return false;
  twai_status_info_t st;
  bool running = canInstalled &&
                 (twai_get_status_info(&st) == ESP_OK) &&
                 (st.state == TWAI_STATE_RUNNING);
  xSemaphoreGive(canDriverMutex);
  return running;
}

void checkCanErrors(unsigned long now)
{
  static unsigned long lastResetAttempt = 0;
  static unsigned long canActiveSince = 0;

  if (!canInstalled)
  {
    // Driver install failed earlier -> retry until installed so passive RX is ready
    if (now - lastResetAttempt >= CAN_REINIT_INTERVAL_MS)
    {
      canBegin();
      lastResetAttempt = now;
    }
    return;
  }

  portENTER_CRITICAL(&dataMux);
  SystemState state = currentState;
  unsigned long pktTime = lastPacketTime;
  portEXIT_CRITICAL(&dataMux);

  // Front MCU is unpowered outside POS2 (IGN) and driving modes (SLEEP, STANDBY, ACC),
  // so skip all CAN diagnostics and driver re-initializations completely.
  if (!isCanActiveState(state))
  {
    canActiveSince = 0;
    return;
  }

  if (canActiveSince == 0)
  {
    canActiveSince = (now != 0) ? now : 1;
  }

  bool needsReinit = false;
  twai_status_info_t st;
  if (twai_get_status_info(&st) == ESP_OK)
  {
    if (st.state != TWAI_STATE_RUNNING)
    {
      // BUS_OFF or STOPPED: perform a full peripheral teardown + reinstall rather than
      // twai_initiate_recovery(), avoiding TWAI_STATE_RECOVERING lockups and ESP-IDF v4.4
      // tx_msg_count assertion bugs during bus-off recovery.
      needsReinit = true;
    }
  }
  else
  {
    needsReinit = true;
  }

  // Allow Front MCU CAN_STALE_REINIT_MS after POS2 turns ON to finish booting before
  // diagnosing a missing RX stream as a stale link.
  bool bootGracePassed = ((now - canActiveSince) >= CAN_STALE_REINIT_MS);
  bool staleLink = bootGracePassed && !isCanPacketFresh(now, pktTime, CAN_STALE_REINIT_MS);
  if ((needsReinit && (now - lastResetAttempt >= CAN_REINIT_INTERVAL_MS)) ||
      (staleLink && (now - lastResetAttempt >= CAN_STALE_REINIT_INTERVAL_MS)))
  {
    canEnd();
    canBegin();
    lastResetAttempt = now;
  }
}

void sendCanHealthFrame(unsigned long now)
{
  (void)now;
  // Read the cross-core heartbeat exactly once, then compute a signed age.
  // If core 0 wrote a heartbeat slightly newer than curMs, the age is negative => alive.
  unsigned long curMs = millis();
  uint32_t hb = last_regulator_heartbeat;
  int32_t hb_age_ms = (int32_t)(curMs - hb);
  bool regulator_ok =
      (regulatorTaskHandle != NULL) &&
      (hb_age_ms < (int32_t)REGULATOR_HEARTBEAT_TIMEOUT_MS);
  health_state = regulator_ok ? 100 : 0;

  if (!canInstalled || canDriverMutex == NULL)
    return;

  // Non-blocking lock: if Core 0 is mid-reinit (~150 us), skip rather than stalling video on Core 1
  if (xSemaphoreTake(canDriverMutex, 0) != pdTRUE)
    return;

  if (canInstalled)
  {
    twai_status_info_t st;
    if (twai_get_status_info(&st) == ESP_OK &&
        st.state == TWAI_STATE_RUNNING &&
        st.msgs_to_tx == 0)
    {
      memset(&canMsgTx, 0, sizeof(canMsgTx));
      canMsgTx.identifier = 0x03;
      canMsgTx.flags = TWAI_MSG_FLAG_SS; // Standard 11-bit ID, single-shot (no auto-retransmit)
      canMsgTx.data_length_code = 8;
      canMsgTx.data[0] = health_state;
      canMsgTx.data[1] = ecoInjCutActive ? 1 : 0;
      // data[2..7] = 0 (memset)
      twai_transmit(&canMsgTx, 0); // Non-blocking: hardware TX buffer is known free
    }
  }

  xSemaphoreGive(canDriverMutex);
}

// Calibrated for 2.3kΩ pull-down: 40°C = 437 ADC, 60°C = 636 ADC, 91°C = 846 ADC, 96°C = 871 ADC, 120°C = 990 ADC
static int rawAdcToTempC(uint16_t raw_adc)
{
  int t_c;
  if (raw_adc <= 437)
    t_c = 40;
  else if (raw_adc >= 990)
    t_c = 120;
  else if (raw_adc < 636)
    t_c = map((int)raw_adc, 437, 636, 40, 60);
  else if (raw_adc < 846)
    t_c = map((int)raw_adc, 636, 846, 60, 91);
  else if (raw_adc < 871)
    t_c = map((int)raw_adc, 846, 871, 91, 96);
  else
    t_c = map((int)raw_adc, 871, 990, 96, 120);
  return constrain(t_c, 40, 120);
}

#define INJ_HISTORY_SAMPLES 20 // 20 samples @ 50ms = 1000ms rolling window

static float inj_history_net_us[INJ_HISTORY_SAMPLES] = {0};
static uint16_t inj_history_pulses[INJ_HISTORY_SAMPLES] = {0};
static uint8_t history_idx = 0;

static uint32_t last_rx_inj_time_us = 0;
static uint16_t last_rx_inj_pulses = 0;
static uint16_t last_rx_spd_pulses = 0;

static uint8_t last_seq_02 = 0;
static bool seq_02_synced = false;
static bool inj_04_synced = false;
static unsigned long last_04_rx_time = 0;
// Time the injector snapshot last advanced. The Front MCU re-sends the previous snapshot
// while an injector is mid-pulse, so at high duty several consecutive 0x04 frames can carry
// delta = 0 and the next delta then spans several 50 ms intervals.
static unsigned long last_inj_change_time = 0;

static void processCanFrame(const twai_message_t &msg, unsigned long rxNow)
{
  if (rxNow == 0)
    rxNow = 1; // Reserve 0 strictly for "no packet received yet"

  // Front MCU only sends standard 11-bit data frames with DLC 8
  if ((msg.flags & (TWAI_MSG_FLAG_EXTD | TWAI_MSG_FLAG_RTR)) || msg.data_length_code < 8)
    return;

  if (msg.identifier == 0x02)
  {
    uint16_t rx_raw2 = (uint16_t)(msg.data[0] | (msg.data[1] << 8));
    int rx_temp = rawAdcToTempC(rx_raw2);
    uint16_t rx_spd = (uint16_t)msg.data[2];
    uint16_t rx_rpm = (uint16_t)(msg.data[3] | (msg.data[4] << 8));

    uint8_t raw_vac_x10 = msg.data[5];
    uint8_t flags = msg.data[6];
    uint8_t new_inj_state = flags & 0x01;
    uint8_t rx_oil = (flags >> 1) & 0x01;
    uint8_t rx_th = (flags >> 2) & 0x01;
    uint8_t rx_ac = (flags >> 4) & 0x01;
    bool is_baro = (flags >> 5) & 0x01;
    bool rx_map_fault = (flags >> 6) & 0x01;
    uint8_t rx_seq_02 = msg.data[7];

    unsigned long prevPacketTime = lastPacketTime;
    uint8_t prevInjState = injector_state;

    uint8_t lost_delta = 0;
    if (seq_02_synced && isCanPacketFresh(rxNow, prevPacketTime, FRONT_MCU_CAN_TIMEOUT_MS))
    {
      uint8_t expected = (last_seq_02 + 1) & 0xFF;
      if (rx_seq_02 != expected)
      {
        lost_delta = (uint8_t)(rx_seq_02 - expected);
      }
    }
    else
    {
      seq_02_synced = true;
    }
    last_seq_02 = rx_seq_02;

    // Transition 0 -> 1: injDisable just engaged (DFCO cutoff starts).
    // Calculate and latch the average net injector pulse width over the active period before cutoff.
    bool latch_active_pulse = false;
    float latched_avg_net = 0.0f;
    if (prevInjState == 0 && new_inj_state == 1)
    {
      float total_net_us = 0.0f;
      uint32_t total_p = 0;
      for (int k = 0; k < INJ_HISTORY_SAMPLES; k++)
      {
        total_net_us += inj_history_net_us[k];
        total_p += inj_history_pulses[k];
      }
      if (total_p > 0 && total_net_us > 0.0f)
      {
        float avg_net = total_net_us / (float)total_p;
        if (avg_net >= 500.0f)
        {
          latch_active_pulse = true;
          latched_avg_net = avg_net;
        }
      }
    }

    float parsed_val = (float)raw_vac_x10 / 10.0f;

    // Publish all fast status fields atomically (~0.5 us critical section, zero division/loops)
    portENTER_CRITICAL(&dataMux);
    raw2 = rx_raw2;
    temp_out = rx_temp;
    spd_t = rx_spd;
    new_rpm = rx_rpm;
    rpm = rx_rpm;
    oil_level_t = rx_oil;
    th_switch_state = rx_th;
    ac_switch_state = rx_ac;
    map_sensor_fault = rx_map_fault;
    if (is_baro)
    {
      if (!rx_map_fault && parsed_val >= 8.0f && parsed_val <= 16.5f)
      {
        baro_psi = parsed_val;
      }
      if (rx_rpm == 0)
      {
        vacuum_psi = 0.0f;
      }
    }
    else
    {
      vacuum_psi = parsed_val;
    }
    lastVacPacketTime = rxNow;
    if (lost_delta > 0)
    {
      can_packets_lost_02 += lost_delta;
    }
    if (latch_active_pulse)
    {
      last_active_inj_pulse_us = latched_avg_net;
    }
    injector_state = new_inj_state;
    lastPacketTime = rxNow;
    portEXIT_CRITICAL(&dataMux);
  }
  else if (msg.identifier == 0x04)
  {
    uint32_t rx_inj_time =
        (uint32_t)msg.data[0] | ((uint32_t)msg.data[1] << 8) |
        ((uint32_t)msg.data[2] << 16) | ((uint32_t)msg.data[3] << 24);
    uint16_t rx_inj_pulses =
        (uint16_t)msg.data[4] | ((uint16_t)msg.data[5] << 8);
    uint16_t rx_spd_pulses =
        (uint16_t)msg.data[6] | ((uint16_t)msg.data[7] << 8);

    if (!inj_04_synced || !isCanPacketFresh(rxNow, last_04_rx_time, FRONT_MCU_CAN_TIMEOUT_MS))
    {
      last_rx_inj_time_us = rx_inj_time;
      last_rx_inj_pulses = rx_inj_pulses;
      last_rx_spd_pulses = rx_spd_pulses;
      last_04_rx_time = rxNow;
      last_inj_change_time = rxNow;
      inj_04_synced = true;

      portENTER_CRITICAL(&dataMux);
      lastPacketTime = rxNow;
      portEXIT_CRITICAL(&dataMux);
    }
    else
    {
      uint32_t delta_time_us = (uint32_t)(rx_inj_time - last_rx_inj_time_us);
      uint16_t delta_pulses = (uint16_t)(rx_inj_pulses - last_rx_inj_pulses);
      uint16_t delta_spd = (uint16_t)(rx_spd_pulses - last_rx_spd_pulses);
      uint32_t elapsed_ms = rxNow - last_04_rx_time;
      // Injector delta may span several frames (mid-pulse deferral) -> measure from last change
      uint32_t inj_elapsed_ms = rxNow - last_inj_change_time;
      last_04_rx_time = rxNow;
      last_rx_inj_time_us = rx_inj_time;
      last_rx_inj_pulses = rx_inj_pulses;
      last_rx_spd_pulses = rx_spd_pulses;
      if (delta_time_us != 0 || delta_pulses != 0)
      {
        last_inj_change_time = rxNow;
      }

      // Plausible delta thresholds scaled to elapsed time with 200ms margin
      uint32_t max_plausible_time = (inj_elapsed_ms + 200) * 1000UL;
      if (max_plausible_time > 5000000UL)
        max_plausible_time = 5000000UL;

      bool add_inj = false;
      float next_live_net_pulse = live_net_pulse_us;
      float next_live_duty = live_inj_duty_cycle;
      bool update_live_net = false;
      bool update_live_duty = false;

      if (delta_time_us <= max_plausible_time)
      {
        add_inj = true;

        // Subtract dead-time from pulses in this interval to calculate net duty cycle.
        float dead_time_us = getInjectorDeadTime(voltage_filtered);
        float net_pulse_us = (float)delta_time_us - ((float)delta_pulses * dead_time_us);
        if (net_pulse_us < 0.0f)
          net_pulse_us = 0.0f;

        uint8_t cur_inj_state = injector_state;
        uint16_t current_rpm = rpm;

        // Record into rolling buffer while injectors are actively firing (filter out near-zero overrun coasting)
        if (cur_inj_state == 0 && delta_pulses > 0)
        {
          if ((net_pulse_us / (float)delta_pulses) >= 400.0f)
          {
            inj_history_net_us[history_idx] = net_pulse_us;
            inj_history_pulses[history_idx] = delta_pulses;
            history_idx = (history_idx + 1) % INJ_HISTORY_SAMPLES;
          }
        }

        if (delta_pulses > 0 && current_rpm > 0)
        {
          float avg_net_pulse_us = net_pulse_us / (float)delta_pulses;
          if (avg_net_pulse_us >= 400.0f)
          {
            next_live_net_pulse = 0.25f * avg_net_pulse_us + 0.75f * next_live_net_pulse;
            update_live_net = true;
          }
          // Engine cycle period for 1 injector (4-stroke: 1 injection per 2 revs = 120,000,000 / RPM us)
          float cycle_period_us = 120000000.0f / (float)current_rpm;
          float raw_duty = (avg_net_pulse_us / cycle_period_us) * 100.0f;
          raw_duty = constrain(raw_duty, 0.0f, 100.0f);
          next_live_duty = 0.25f * raw_duty + 0.75f * next_live_duty;
          update_live_duty = true;
        }
        else if (current_rpm == 0 || cur_inj_state == 1 ||
                 (rxNow - last_inj_change_time) > 500)
        {
          // Engine stopped, fuel cut active, or no pulses for > 500 ms: injectors genuinely closed
          next_live_duty *= 0.75f;
          update_live_duty = true;
        }
        // else: no completed pulse inside this 50 ms frame (normal below ~1200 RPM, or the
        // Front MCU deferred a mid-pulse snapshot) -> hold the last value instead of feeding 0%.
      }

      uint32_t max_plausible_spd_delta = (uint32_t)((elapsed_ms + 200) * 1.5f);
      if (max_plausible_spd_delta > 10000UL)
        max_plausible_spd_delta = 10000UL;
      bool add_spd = (delta_spd <= max_plausible_spd_delta);

      // Atomically accumulate counters so Core 1's 1 Hz fetch-and-zero never drops a pulse
      portENTER_CRITICAL(&dataMux);
      if (add_inj)
      {
        accumulated_inj_time_us += delta_time_us;
        accumulated_inj_pulses += delta_pulses;
      }
      if (update_live_net)
      {
        live_net_pulse_us = next_live_net_pulse;
      }
      if (update_live_duty)
      {
        live_inj_duty_cycle = next_live_duty;
      }
      if (add_spd)
      {
        spd_delta_pulses += delta_spd;
      }
      lastPacketTime = rxNow;
      portEXIT_CRITICAL(&dataMux);
    }
  }
}

void drainCanRxBuffer(unsigned long now)
{
  (void)now;
  if (!canInstalled)
    return;
  twai_message_t rxMsg;
  for (int i = 0; i < CAN_RX_DRAIN_MAX; i++)
  {
    if (twai_receive(&rxMsg, 0) != ESP_OK)
      break; // Non-blocking: queue empty
    processCanFrame(rxMsg, millis());
  }
}

static void canRxTask(void *pvParameters)
{
  (void)pvParameters;
  esp_task_wdt_add(NULL);

  // Install TWAI driver FROM Core 0 so esp_intr_alloc binds TWAI hardware interrupts to Core 0
  // (completely isolated from Core 1's composite video DMA/DAC scanline interrupts).
  if (!canBegin())
  {
    Serial.println("[CAN] TWAI driver start failed - will retry in CanRxTask");
  }

  bool wasCanActive = false;
  twai_message_t rxMsg;
  for (;;)
  {
    if (!canRxTaskRunning)
    {
      esp_task_wdt_delete(NULL);
      portENTER_CRITICAL(&dataMux);
      canRxTaskHandle = NULL;
      portEXIT_CRITICAL(&dataMux);
      vTaskDelete(NULL);
    }

    if (canInstalled)
    {
      // Block up to 20 ms waiting on TWAI RX queue: 0% CPU while idle, wakes within
      // microseconds of frame arrival so Core 0's regulatorTask always sees fresh telemetry.
      if (twai_receive(&rxMsg, pdMS_TO_TICKS(20)) == ESP_OK)
      {
        processCanFrame(rxMsg, millis());
        for (int i = 1; i < CAN_RX_DRAIN_MAX; i++)
        {
          if (!canRxTaskRunning || twai_receive(&rxMsg, 0) != ESP_OK)
            break;
          processCanFrame(rxMsg, millis());
        }
      }
    }
    else
    {
      vTaskDelay(pdMS_TO_TICKS(20));
    }

    unsigned long nowMs = millis();

    // Clear Front MCU live telemetry immediately when leaving POS2/driving (Front MCU loses
    // Terminal 15 power in SLEEP/STANDBY/ACC) and zero fast motion/injector states on CAN timeout.
    portENTER_CRITICAL(&dataMux);
    bool canActive = isCanActiveState(currentState);
    if (wasCanActive && !canActive)
    {
      lastPacketTime = 0;
      lastVacPacketTime = 0;
      rpm = 0;
      new_rpm = 0;
      spd_t = 0;
      oil_level_t = 0;
      th_switch_state = 1;
      ac_switch_state = 0;
      map_sensor_fault = false;
      vacuum_psi = 0.0f;
      injector_state = 0;
      live_inj_duty_cycle = 0.0f;
    }
    else if (!isCanPacketFresh(nowMs, lastPacketTime, FRONT_MCU_CAN_TIMEOUT_MS))
    {
      rpm = 0;
      new_rpm = 0;
      spd_t = 0;
      injector_state = 0;
      live_inj_duty_cycle = 0.0f;
    }
    portEXIT_CRITICAL(&dataMux);

    if (wasCanActive && !canActive)
    {
      // Reset sequence/counter sync so the Front MCU's next power-up in POS2 syncs cleanly from frame #1
      seq_02_synced = false;
      inj_04_synced = false;
    }
    wasCanActive = canActive;

    checkCanErrors(nowMs);
    esp_task_wdt_reset();
  }
}

void startCanRxTask()
{
  ensureCanMutex();
  if (canRxTaskHandle != NULL)
    return;

  canRxTaskRunning = true;
  xTaskCreatePinnedToCore(
      canRxTask,
      "CanRxTask",
      4096,
      nullptr,
      configMAX_PRIORITIES - 1, // Just above RegulatorTask: ~4 us burst on CAN RX, 0% idle CPU
      &canRxTaskHandle,
      0); // Pin to Core 0 alongside RegulatorTask
}

void stopCanRxTask()
{
  if (canRxTaskHandle != NULL)
  {
    canRxTaskRunning = false;
    for (int timeout = 0; timeout < 100; timeout++)
    {
      esp_task_wdt_reset();
      taskYIELD();
      vTaskDelay(pdMS_TO_TICKS(5));
      if (canRxTaskHandle == NULL)
        break;
    }
    portENTER_CRITICAL(&dataMux);
    TaskHandle_t h = canRxTaskHandle;
    canRxTaskHandle = NULL;
    portEXIT_CRITICAL(&dataMux);
    if (h != NULL)
    {
      esp_task_wdt_delete(h);
      vTaskDelete(h);
    }
  }
}
