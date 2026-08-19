#include "can_comm.h"
#include "fuel.h"

void checkCanErrors() {
  uint8_t errFlags = mcp2515.getErrorFlags();
  if (errFlags != 0) {
    // Clear overflow flags to resume reception
    if (errFlags & (MCP2515::EFLG_RX0OVR | MCP2515::EFLG_RX1OVR)) {
      mcp2515.clearRXnOVR();
    }

    // Only completely reset the chip if it goes into Bus-Off (fatal state).
    // Do NOT interfere if it's just in Error Passive (TXEP/RXEP); it will
    // self-recover.
    if (errFlags & MCP2515::EFLG_TXBO) {
      mcp2515.reset();
      mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
      mcp2515.setNormalOneShotMode();
    }
  }
}

#define INJ_HISTORY_SAMPLES 20 // 20 samples @ 50ms = 1000ms rolling window

static float inj_history_net_us[INJ_HISTORY_SAMPLES] = {0};
static uint16_t inj_history_pulses[INJ_HISTORY_SAMPLES] = {0};
static uint8_t history_idx = 0;

static uint32_t last_rx_inj_time_us = 0;
static uint16_t last_rx_inj_pulses = 0;
static uint32_t last_rx_spd_pulses = 0;

static uint8_t last_seq_02 = 0;
static uint8_t last_seq_04 = 0;
static uint8_t last_seq_05 = 0;

static bool seq_02_synced = false;
static bool inj_04_synced = false;
static bool spd_05_synced = false;

static unsigned long last_04_rx_time = 0;
static unsigned long last_05_rx_time = 0;

void drainCanRxBuffer(unsigned long now) {
  for (int i = 0; i < 6; i++) {
    if (mcp2515.readMessage(&canMsg) != MCP2515::ERROR_OK) break;
    if (canMsg.can_id == 0x02) {
      raw2 = (uint16_t)((canMsg.data[1] << 8) | canMsg.data[0]);
      spd_t = (uint16_t)((canMsg.data[3] << 8) | canMsg.data[2]);
      new_rpm = (uint16_t)((canMsg.data[5] << 8) | canMsg.data[4]);
      uint8_t flags = canMsg.data[6];
      uint8_t new_inj_state = flags & 0x01;
      oil_level_t = (flags >> 1) & 0x01;
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
      // Calculate and latch the average net injector pulse width over the 1 second before cutoff.
      if (injector_state == 0 && new_inj_state == 1) {
        float total_net_us = 0.0f;
        uint32_t total_p = 0;
        for (int k = 0; k < INJ_HISTORY_SAMPLES; k++) {
          total_net_us += inj_history_net_us[k];
          total_p += inj_history_pulses[k];
        }
        if (total_p > 0 && total_net_us > 0.0f) {
          last_active_inj_pulse_us = total_net_us / (float)total_p;
        }
      }

      injector_state = new_inj_state;
      lastPacketTime = now;
    } else if (canMsg.can_id == 0x04) {
      uint32_t rx_inj_time =
          (uint32_t)canMsg.data[0] | ((uint32_t)canMsg.data[1] << 8) |
          ((uint32_t)canMsg.data[2] << 16) | ((uint32_t)canMsg.data[3] << 24);
      uint16_t rx_inj_pulses =
          (uint16_t)canMsg.data[4] | ((uint16_t)canMsg.data[5] << 8);
      uint8_t rx_seq_04 = canMsg.data[7];

      if (!inj_04_synced || (now - last_04_rx_time > FRONT_MCU_CAN_TIMEOUT_MS)) {
        last_rx_inj_time_us = rx_inj_time;
        last_rx_inj_pulses = rx_inj_pulses;
        last_seq_04 = rx_seq_04;
        last_04_rx_time = now;
        inj_04_synced = true;
      } else if (rx_seq_04 == last_seq_04) {
        // Duplicate frame from hardware retransmission - ignore to avoid double counting
      } else {
        uint8_t expected = (last_seq_04 + 1) & 0xFF;
        if (rx_seq_04 != expected) {
          can_packets_lost_04 += (uint8_t)(rx_seq_04 - expected);
        }
        last_seq_04 = rx_seq_04;

        uint32_t delta_time_us = (uint32_t)(rx_inj_time - last_rx_inj_time_us);
        uint16_t delta_pulses = (uint16_t)(rx_inj_pulses - last_rx_inj_pulses);
        uint32_t elapsed_ms = now - last_04_rx_time;
        last_04_rx_time = now;
        last_rx_inj_time_us = rx_inj_time;
        last_rx_inj_pulses = rx_inj_pulses;

        // Plausible delta threshold scaled to elapsed time with 200ms margin (capped at 5s)
        uint32_t max_plausible_time = (elapsed_ms + 200) * 1000UL;
        if (max_plausible_time > 5000000UL) max_plausible_time = 5000000UL;

        if (delta_time_us <= max_plausible_time) {
          accumulated_inj_time_us += delta_time_us;
          accumulated_inj_pulses += delta_pulses;

          // Subtract dead-time from pulses in this interval to calculate net duty cycle.
          float dead_time_us = getInjectorDeadTime(voltage_filtered);
          float net_pulse_us = (float)delta_time_us - ((float)delta_pulses * dead_time_us);
          if (net_pulse_us < 0.0f) net_pulse_us = 0.0f;

          // Record into rolling 1-sec buffer while injectors are actively firing
          if (injector_state == 0) {
            inj_history_net_us[history_idx] = net_pulse_us;
            inj_history_pulses[history_idx] = delta_pulses;
            history_idx = (history_idx + 1) % INJ_HISTORY_SAMPLES;
          }

          float raw_duty = 0.0f;
          uint16_t current_rpm = (rpm > 0) ? rpm : new_rpm;
          if (delta_pulses > 0 && current_rpm > 0) {
            float avg_net_pulse_us = net_pulse_us / (float)delta_pulses;
            // Engine cycle period for 1 injector (4-stroke: 1 injection per 2 revs = 120,000,000 / RPM us)
            float cycle_period_us = 120000000.0f / (float)current_rpm;
            raw_duty = (avg_net_pulse_us / cycle_period_us) * 100.0f;
          }
          raw_duty = constrain(raw_duty, 0.0f, 100.0f);
          live_inj_duty_cycle =
              0.25f * raw_duty + 0.75f * live_inj_duty_cycle;
        }
      }
    } else if (canMsg.can_id == 0x05) {
      uint32_t rx_spd_pulses =
          (uint32_t)canMsg.data[0] | ((uint32_t)canMsg.data[1] << 8) |
          ((uint32_t)canMsg.data[2] << 16) | ((uint32_t)canMsg.data[3] << 24);
      uint8_t rx_seq_05 = canMsg.data[4];

      if (!spd_05_synced || (now - last_05_rx_time > FRONT_MCU_CAN_TIMEOUT_MS)) {
        last_rx_spd_pulses = rx_spd_pulses;
        last_seq_05 = rx_seq_05;
        last_05_rx_time = now;
        spd_05_synced = true;
      } else if (rx_seq_05 == last_seq_05) {
        // Duplicate frame - ignore
      } else {
        uint8_t expected = (last_seq_05 + 1) & 0xFF;
        if (rx_seq_05 != expected) {
          can_packets_lost_05 += (uint8_t)(rx_seq_05 - expected);
        }
        last_seq_05 = rx_seq_05;

        uint32_t delta_pulses = (uint32_t)(rx_spd_pulses - last_rx_spd_pulses);
        uint32_t elapsed_ms = now - last_05_rx_time;
        last_05_rx_time = now;
        last_rx_spd_pulses = rx_spd_pulses;

        // Plausible delta speed pulse limit (max ~1500 pulses/sec at 220 km/h)
        uint32_t max_plausible_spd_delta = (uint32_t)((elapsed_ms + 200) * 1.5f);
        if (max_plausible_spd_delta > 10000UL) max_plausible_spd_delta = 10000UL;

        if (delta_pulses <= max_plausible_spd_delta) {
          spd_delta_pulses += delta_pulses;
        }
      }
    }
  }
}
