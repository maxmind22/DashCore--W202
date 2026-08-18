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

void drainCanRxBuffer(unsigned long now) {
  for (int i = 0; i < 6; i++) {
    if (mcp2515.readMessage(&canMsg) != MCP2515::ERROR_OK) break;
    if (canMsg.can_id == 0x02) {
      raw2 = (uint16_t)((canMsg.data[1] << 8) | canMsg.data[0]);
      spd_t = (uint16_t)(canMsg.data[3] << 8 | canMsg.data[2]);
      uint8_t new_inj_state = canMsg.data[4];

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
      new_rpm = (uint16_t)((canMsg.data[6] << 8) | canMsg.data[5]);
      oil_level_t = (uint8_t)canMsg.data[7];
      lastPacketTime = now;
    } else if (canMsg.can_id == 0x04) {
      uint32_t pulse =
          (uint32_t)canMsg.data[0] | ((uint32_t)canMsg.data[1] << 8) |
          ((uint32_t)canMsg.data[2] << 16) | ((uint32_t)canMsg.data[3] << 24);
      uint16_t pulses =
          (uint16_t)canMsg.data[4] | ((uint16_t)canMsg.data[5] << 8);
      if (pulse <= MAX_INJ_PULSE_PER_INTERVAL_US) {
        accumulated_inj_time_us += pulse;
        accumulated_inj_pulses += pulses;
        // Subtract dead-time from pulses in this interval to calculate net duty cycle.
        float dead_time_us = getInjectorDeadTime(voltage_filtered);
        float net_pulse_us = (float)pulse - ((float)pulses * dead_time_us);
        if (net_pulse_us < 0.0f) net_pulse_us = 0.0f;

        // Record into rolling 1-sec buffer while injectors are actively firing
        if (injector_state == 0) {
          inj_history_net_us[history_idx] = net_pulse_us;
          inj_history_pulses[history_idx] = pulses;
          history_idx = (history_idx + 1) % INJ_HISTORY_SAMPLES;
        }

        float raw_duty = 0.0f;
        uint16_t current_rpm = (rpm > 0) ? rpm : new_rpm;
        if (pulses > 0 && current_rpm > 0) {
          float avg_net_pulse_us = net_pulse_us / (float)pulses;
          // Engine cycle period for 1 injector (4-stroke: 1 injection per 2 revs = 120,000,000 / RPM us)
          float cycle_period_us = 120000000.0f / (float)current_rpm;
          raw_duty = (avg_net_pulse_us / cycle_period_us) * 100.0f;
        }
        raw_duty = constrain(raw_duty, 0.0f, 100.0f);
        live_inj_duty_cycle =
            0.25f * raw_duty + 0.75f * live_inj_duty_cycle;
      }
    }
  }
}
