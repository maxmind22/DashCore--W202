#include "can_comm.h"
#include "fuel.h"

// ============================================================================
// ESP32 TWAI (built-in CAN 2.0B controller) + SN65HVD230 transceiver
// ----------------------------------------------------------------------------
// Bit timing: 500 kbps to match the Front MCU (MCP2515 @ 8 MHz, CAN_500KBPS).
// Mode: NORMAL (ACKs frames). Health frames use single-shot TX, mirroring the
// previous MCP2515 "NormalOneShot" mode, so a missing Front MCU never causes
// endless retransmissions on the bus.
// ============================================================================

static twai_message_t canMsg;
static twai_message_t canMsgTx;
static bool canInstalled = false;

bool canBegin()
{
  if (canInstalled)
    return true;

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
    return false;
  // Keep weak internal pull-up on CRX so an unplugged transceiver never floats or picks up EMI
  gpio_set_pull_mode((gpio_num_t)PIN_CAN_RX, GPIO_PULLUP_ONLY);
  if (twai_start() != ESP_OK)
  {
    twai_driver_uninstall();
    pinMode(PIN_CAN_TX, INPUT_PULLUP);
    return false;
  }
  canInstalled = true;
  return true;
}

void canEnd()
{
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
}

bool canIsRunning()
{
  if (!canInstalled)
    return false;
  twai_status_info_t st;
  return (twai_get_status_info(&st) == ESP_OK) && (st.state == TWAI_STATE_RUNNING);
}

void checkCanErrors(unsigned long now) {
  static unsigned long lastResetAttempt = 0;

  if (!canInstalled) {
    // Driver install failed earlier (or was torn down) -> retry periodically
    if (now - lastResetAttempt >= CAN_REINIT_INTERVAL_MS) {
      canBegin();
      lastResetAttempt = now;
    }
    return;
  }

  bool needsReinit = false;
  twai_status_info_t st;
  if (twai_get_status_info(&st) == ESP_OK) {
    if (st.state != TWAI_STATE_RUNNING) {
      // BUS_OFF or STOPPED: perform a full peripheral teardown + reinstall rather than
      // twai_initiate_recovery(), avoiding TWAI_STATE_RECOVERING lockups and ESP-IDF v4.4
      // tx_msg_count assertion bugs during bus-off recovery.
      needsReinit = true;
    }
  } else {
    needsReinit = true;
  }

  // Auto-recovery for stale/disconnected communication:
  // If no packets received for > 2000ms (or controller entered BUS_OFF/STOPPED),
  // periodically tear down and re-install the TWAI driver every 1000ms.
  if ((needsReinit || (now - lastPacketTime > CAN_STALE_REINIT_MS)) &&
      (now - lastResetAttempt >= CAN_REINIT_INTERVAL_MS)) {
    canEnd();
    canBegin();
    lastResetAttempt = now;
  }
}

void sendCanHealthFrame(unsigned long now) {
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

  if (!canInstalled)
    return;

  twai_status_info_t st;
  if (twai_get_status_info(&st) != ESP_OK || st.state != TWAI_STATE_RUNNING || st.msgs_to_tx > 0)
    return;

  memset(&canMsgTx, 0, sizeof(canMsgTx));
  canMsgTx.identifier = 0x03;
  canMsgTx.flags = TWAI_MSG_FLAG_SS; // Standard 11-bit ID, single-shot (no auto-retransmit)
  canMsgTx.data_length_code = 8;
  canMsgTx.data[0] = health_state;
  canMsgTx.data[1] = ecoInjCutActive ? 1 : 0;
  // data[2..7] = 0 (memset)
  twai_transmit(&canMsgTx, 0); // Non-blocking: hardware TX buffer is known free
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

void drainCanRxBuffer(unsigned long now) {
  if (!canInstalled) return;
  for (int i = 0; i < CAN_RX_DRAIN_MAX; i++) {
    if (twai_receive(&canMsg, 0) != ESP_OK) break; // Non-blocking: queue empty
    // Front MCU only sends standard 11-bit data frames with DLC 8
    if ((canMsg.flags & (TWAI_MSG_FLAG_EXTD | TWAI_MSG_FLAG_RTR)) || canMsg.data_length_code < 8) continue;
    if (canMsg.identifier == 0x02) {
      raw2 = (uint16_t)(canMsg.data[0] | (canMsg.data[1] << 8));
      spd_t = (uint16_t)canMsg.data[2];
      new_rpm = (uint16_t)(canMsg.data[3] | (canMsg.data[4] << 8));

      uint8_t raw_vac_x10 = canMsg.data[5];
      uint8_t flags = canMsg.data[6];
      uint8_t new_inj_state = flags & 0x01;
      oil_level_t = (flags >> 1) & 0x01;
      th_switch_state = (flags >> 2) & 0x01;
      ac_switch_state = (flags >> 4) & 0x01;
      bool is_baro = (flags >> 5) & 0x01;
      map_sensor_fault = (flags >> 6) & 0x01;

      portENTER_CRITICAL(&dataMux);
      if (is_baro) {
        if (!map_sensor_fault) {
          float r_baro = (float)raw_vac_x10 / 10.0f;
          if (r_baro >= 8.0f && r_baro <= 16.5f) {
            baro_psi = r_baro;
          }
        }
        if (new_rpm == 0 && rpm == 0) {
          vacuum_psi = 0.0f;
        }
      } else {
        vacuum_psi = (float)raw_vac_x10 / 10.0f;
      }
      lastVacPacketTime = now;
      portEXIT_CRITICAL(&dataMux);

      uint8_t rx_seq_02 = canMsg.data[7];

      if (seq_02_synced && (now - lastPacketTime <= FRONT_MCU_CAN_TIMEOUT_MS)) {
        uint8_t expected = (last_seq_02 + 1) & 0xFF;
        if (rx_seq_02 != expected) {
          can_packets_lost_02 += (uint8_t)(rx_seq_02 - expected);
        }
      } else {
        seq_02_synced = true;
      }
      last_seq_02 = rx_seq_02;

      // Transition 0 -> 1: injDisable just engaged (DFCO cutoff starts).
      // Calculate and latch the average net injector pulse width over the active period before cutoff.
      if (injector_state == 0 && new_inj_state == 1) {
        float total_net_us = 0.0f;
        uint32_t total_p = 0;
        for (int k = 0; k < INJ_HISTORY_SAMPLES; k++) {
          total_net_us += inj_history_net_us[k];
          total_p += inj_history_pulses[k];
        }
        if (total_p > 0 && total_net_us > 0.0f) {
          float avg_net = total_net_us / (float)total_p;
          if (avg_net >= 500.0f) {
            last_active_inj_pulse_us = avg_net;
          }
        }
      }

      injector_state = new_inj_state;
      lastPacketTime = now;
    } else if (canMsg.identifier == 0x04) {
      uint32_t rx_inj_time =
          (uint32_t)canMsg.data[0] | ((uint32_t)canMsg.data[1] << 8) |
          ((uint32_t)canMsg.data[2] << 16) | ((uint32_t)canMsg.data[3] << 24);
      uint16_t rx_inj_pulses =
          (uint16_t)canMsg.data[4] | ((uint16_t)canMsg.data[5] << 8);
      uint16_t rx_spd_pulses =
          (uint16_t)canMsg.data[6] | ((uint16_t)canMsg.data[7] << 8);

      if (!inj_04_synced || (now - last_04_rx_time > FRONT_MCU_CAN_TIMEOUT_MS)) {
        last_rx_inj_time_us = rx_inj_time;
        last_rx_inj_pulses = rx_inj_pulses;
        last_rx_spd_pulses = rx_spd_pulses;
        last_04_rx_time = now;
        last_inj_change_time = now;
        inj_04_synced = true;
      } else {
        uint32_t delta_time_us = (uint32_t)(rx_inj_time - last_rx_inj_time_us);
        uint16_t delta_pulses = (uint16_t)(rx_inj_pulses - last_rx_inj_pulses);
        uint16_t delta_spd = (uint16_t)(rx_spd_pulses - last_rx_spd_pulses);
        uint32_t elapsed_ms = now - last_04_rx_time;
        // Injector delta may span several frames (mid-pulse deferral) -> measure from last change
        uint32_t inj_elapsed_ms = now - last_inj_change_time;
        last_04_rx_time = now;
        last_rx_inj_time_us = rx_inj_time;
        last_rx_inj_pulses = rx_inj_pulses;
        last_rx_spd_pulses = rx_spd_pulses;
        if (delta_time_us != 0 || delta_pulses != 0) {
          last_inj_change_time = now;
        }

        // Plausible delta thresholds scaled to elapsed time with 200ms margin
        uint32_t max_plausible_time = (inj_elapsed_ms + 200) * 1000UL;
        if (max_plausible_time > 5000000UL) max_plausible_time = 5000000UL;

        if (delta_time_us <= max_plausible_time) {
          accumulated_inj_time_us += delta_time_us;
          accumulated_inj_pulses += delta_pulses;

          // Subtract dead-time from pulses in this interval to calculate net duty cycle.
          float dead_time_us = getInjectorDeadTime(voltage_filtered);
          float net_pulse_us = (float)delta_time_us - ((float)delta_pulses * dead_time_us);
          if (net_pulse_us < 0.0f) net_pulse_us = 0.0f;

          // Record into rolling buffer while injectors are actively firing (filter out near-zero overrun coasting)
          if (injector_state == 0 && delta_pulses > 0) {
            if ((net_pulse_us / (float)delta_pulses) >= 400.0f) {
              inj_history_net_us[history_idx] = net_pulse_us;
              inj_history_pulses[history_idx] = delta_pulses;
              history_idx = (history_idx + 1) % INJ_HISTORY_SAMPLES;
            }
          }

          uint16_t current_rpm = (rpm > 0) ? rpm : new_rpm;
          if (delta_pulses > 0 && current_rpm > 0) {
            float avg_net_pulse_us = net_pulse_us / (float)delta_pulses;
            if (avg_net_pulse_us >= 400.0f) {
              live_net_pulse_us = 0.25f * avg_net_pulse_us + 0.75f * live_net_pulse_us;
            }
            // Engine cycle period for 1 injector (4-stroke: 1 injection per 2 revs = 120,000,000 / RPM us)
            float cycle_period_us = 120000000.0f / (float)current_rpm;
            float raw_duty = (avg_net_pulse_us / cycle_period_us) * 100.0f;
            raw_duty = constrain(raw_duty, 0.0f, 100.0f);
            live_inj_duty_cycle =
                0.25f * raw_duty + 0.75f * live_inj_duty_cycle;
          } else if (current_rpm == 0 || injector_state == 1 ||
                     (now - last_inj_change_time) > 500) {
            // Engine stopped, fuel cut active, or no pulses for > 500 ms: injectors genuinely closed
            live_inj_duty_cycle *= 0.75f;
          }
          // else: no completed pulse inside this 50 ms frame (normal below ~1200 RPM, or the
          // Front MCU deferred a mid-pulse snapshot) -> hold the last value instead of feeding 0%.
        }

        uint32_t max_plausible_spd_delta = (uint32_t)((elapsed_ms + 200) * 1.5f);
        if (max_plausible_spd_delta > 10000UL) max_plausible_spd_delta = 10000UL;
        if (delta_spd <= max_plausible_spd_delta) {
          spd_delta_pulses += delta_spd;
        }
      }
      lastPacketTime = now;
    }
  }
}
