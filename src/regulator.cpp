#include "regulator.h"
#include "globals.h"
#include <Adafruit_ADS1X15.h>
#include <Wire.h>
#include <esp_task_wdt.h>

void recoverI2CBus(int sdaPin, int sclPin) {
  pinMode(sdaPin, INPUT_PULLUP);
  pinMode(sclPin, OUTPUT);
  digitalWrite(sclPin, HIGH);
  delay(1);

  // Toggle SCL if SDA is held low by a stuck slave device
  if (digitalRead(sdaPin) == LOW) {
    for (int i = 0; i < 9; i++) {
      digitalWrite(sclPin, LOW);
      delayMicroseconds(5);
      digitalWrite(sclPin, HIGH);
      delayMicroseconds(5);
      if (digitalRead(sdaPin) == HIGH) {
        break; // Device released the bus
      }
    }
  }

  // Generate standard I2C STOP condition so all slaves reset state machine
  digitalWrite(sclPin, LOW);
  delayMicroseconds(5);
  pinMode(sdaPin, OUTPUT);
  digitalWrite(sdaPin, LOW);
  delayMicroseconds(5);
  digitalWrite(sclPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(sdaPin, HIGH);
  delayMicroseconds(5);
  pinMode(sdaPin, INPUT);
}

void stopRegulatorTask() {
  if (regulatorTaskHandle != NULL) {
    regulatorTaskRunning = false;
    for (int timeout = 0; timeout < 100; timeout++) {
      esp_task_wdt_reset();
      taskYIELD();
      vTaskDelay(pdMS_TO_TICKS(5));
      if (regulatorTaskHandle == NULL)
        break;
    }
    portENTER_CRITICAL(&dataMux);
    TaskHandle_t h = regulatorTaskHandle;
    regulatorTaskHandle = NULL;
    portEXIT_CRITICAL(&dataMux);
    if (h != NULL) {
      esp_task_wdt_delete(h);
      vTaskDelete(h);
    }
  }
  // Guarantee safe hardware state upon task stop
  ledcWrite(0, 0);
  digitalWrite(field_relay_pin, LOW);
}

static int16_t readADC_Timeout(uint8_t channel, uint32_t timeout_ms = 25) {
  if (channel > 3) return -1;
  adc.startADCReading(MUX_BY_CHANNEL[channel], /*continuous=*/false);
  uint32_t start = millis();
  while (!adc.conversionComplete()) {
    if (millis() - start >= timeout_ms) {
      return -1; // Timed out waiting for conversion (I2C failure or disconnected sensor)
    }
    vTaskDelay(1);
  }
  return adc.getLastConversionResults();
}

void regulatorTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  int consecutive_failures = 0;

  const float voltage_alpha = VOLTAGE_EMA_ALPHA;
  const float current_sensor_offset_mv = CURRENT_SENSOR_OFFSET_MV;
  const float current_sensor_mV_per_A = CURRENT_SENSOR_MV_PER_A;
  const float current_alpha = CURRENT_EMA_ALPHA;

  // Baseline PID Gains (Tuned for physical plant: ~20 PWM counts per Amp)
  const float base_Kp_v = PID_KP_VOLTAGE;
  const float base_Ki_v = PID_KI_VOLTAGE;
  const float base_Kp_i = PID_KP_CURRENT;
  const float base_Ki_i = PID_KI_CURRENT;

  // Controller State Variables
  static float integral_v = 0.0f;
  static float integral_i = 0.0f;
  static float active_pwm = 0.0f;
  static unsigned long last_regulator_time = 0;
  static uint32_t last_fuel_time = 0;
  static unsigned long relay_trip_time = 0;

  unsigned long runningStartTime = 0;
  const unsigned long CHARGE_DELAY_MS = 2000; // 2 seconds delay before charging starts
  unsigned long last_severe_time = 0;
  bool severe_hold_active = false;
  unsigned long overvoltage_start_time = 0;
  bool overvoltage_latched = false;
  uint32_t undercharging_accum_ms = 0;
  uint32_t charging_ok_ms = 0;
  bool unresponsiveness_fault = false;
  unsigned long battery_low_start_time = 0;
  unsigned long last_diag_time = 0;

  float pre_crank_discharge_a = 0.0f;
  bool pre_crank_i_init = false;
  bool crank_cap_active = false;
  unsigned long cap_relay_on_ms = 0;
  unsigned long cap_engage_ms = 0;
  float cap_base_discharge_a = 0.0f;
  float cap_peak_total_a = 0.0f;
  float cap_peak_starter_a = 0.0f;
  float cap_min_v = 99.0f;
  float cap_steady_v_sum = 0.0f;
  float cap_steady_i_sum = 0.0f;
  uint16_t cap_steady_samples = 0;
  uint16_t cap_engage_latency_ms = 0;
  bool cap_engaged = false;
  bool cap_steady_closed = false;
  bool cap_in_dropout = false;
  bool cap_had_sensor_err = false;
  uint8_t cap_dropout_count = 0;
  uint32_t cap_seq = 0;

  for (;;) {
    if (!regulatorTaskRunning) {
      esp_task_wdt_delete(NULL);
      ledcWrite(0, 0); // Ensure field coil off before deleting
      portENTER_CRITICAL(&dataMux);
      regulatorTaskHandle = NULL;
      portEXIT_CRITICAL(&dataMux);
      vTaskDelete(NULL); // Delete self safely
    }

    // --- 1. ADC Voltage and Current Measurement ---
    int16_t voltage_raw = readADC_Timeout(0, 25);
    int16_t current_raw_t = readADC_Timeout(1, 25);

    esp_task_wdt_reset(); // Reset WDT after potentially blocking I2C reads

    float new_v = voltage_filtered;
    float new_c = current_A_filtered;
    float raw_v = new_v;
    float raw_c = new_c;
    bool raw_sample_valid = false;

    if (voltage_raw > 0 && current_raw_t > 0) {
      consecutive_failures = 0;
      // GAIN_ONE is 0.125mV per bit (0.000125V)
      // Voltage divider multiplier: 0.000125f * 4.337667187f = 0.0005422084f
      float voltage = voltage_raw * 0.0005422084f;
      new_v = voltage_alpha * voltage + (1.0f - voltage_alpha) * voltage_filtered;

      // Current conversion: 0.125mV per bit
      float current_mv = current_raw_t * 0.125f;
      float current_raw =
          (current_mv - current_sensor_offset_mv) / current_sensor_mV_per_A;
      new_c = current_alpha * current_raw +
              (1.0f - current_alpha) * current_A_filtered;
      raw_v = voltage;
      raw_c = current_raw;
      raw_sample_valid = true;
    } else {
      consecutive_failures++;
      if (consecutive_failures > 50) {
        // I2C Bus Recovery
        Wire.end();
        vTaskDelay(pdMS_TO_TICKS(10)); // Yield CPU properly instead of blocking delay()
        recoverI2CBus(PIN_I2C_SDA, PIN_I2C_SCL); // Toggle SCL to release any stuck I2C slave
        Wire.begin();
        Wire.setClock(100000);
        Wire.setTimeOut(20); // Ensure I2C transaction timeout is set in recovery
        adc.begin();
        adc.setGain(GAIN_ONE);
        adc.setDataRate(RATE_ADS1115_860SPS);
        consecutive_failures = 0;
      }
    }

    bool sensor_error = (consecutive_failures > 5);

    // --- 2. Emergency Protection & Relay Anti-Chatter Hysteresis ---
    bool trip_condition = (new_v >= REGULATOR_V_EMERGENCY ||
                           new_c > EMERGENCY_OVERCURRENT_A ||
                           sensor_error);

    if (trip_condition) {
      relay_trip_time = millis();
    }

    // Latch relay open for minimum cooldown and until voltage falls below safe limit
    bool relay_latched = false;
    if (relay_trip_time != 0) {
      if (millis() - relay_trip_time < REGULATOR_RELAY_COOLDOWN_MS ||
          new_v >= REGULATOR_RELAY_RESET_V) {
        relay_latched = true;
      } else {
        relay_trip_time = 0;
      }
    }

    bool severe_failure = relay_latched;
    if (relay_latched) {
      digitalWrite(field_relay_pin, HIGH); // Open high-side safety relay
    } else {
      digitalWrite(field_relay_pin, LOW);  // Closed (normal operation)
    }

    // --- 3. Snapshot Shared Engine State (Fed directly on Core 0 by CanRxTask) ---
    portENTER_CRITICAL(&dataMux);
    uint16_t in_rpm = rpm;
    SystemState in_state = currentState;
    unsigned long in_last_packet = lastPacketTime;
    float in_vac = vacuum_psi;
    unsigned long in_last_vac = lastVacPacketTime;
    unsigned long in_last_env = lastEnvPacketTime;
    bool in_map_fault = map_sensor_fault;
    int in_temp = temp_out;
    uint8_t in_ac = ac_switch_state;
    bool in_relay_active = starterRelayActive;
    unsigned long in_relay_on_ms = starterRelayOnMs;
    unsigned long in_relay_off_ms = starterRelayOffMs;
    uint8_t in_crank_outcome = lastCrankOutcome;
    portEXIT_CRITICAL(&dataMux);

    unsigned long reg_now = millis();
    bool frontMcuConnected = isCanPacketFresh(reg_now, in_last_packet, FRONT_MCU_CAN_TIMEOUT_MS);

    // --- 3b. High-Speed Starter Waveform Capture (860 SPS unfiltered during crank) ---
    bool crank_fast_mode = (in_state == STATE_CRANKING || in_relay_active);

    if (!in_relay_active && in_state != STATE_RUNNING && raw_sample_valid) {
      float disch_a = -raw_c;
      if (disch_a < 0.0f) disch_a = 0.0f;
      if (disch_a > 40.0f) disch_a = 40.0f;
      if (!pre_crank_i_init) {
        pre_crank_discharge_a = disch_a;
        pre_crank_i_init = true;
      } else {
        pre_crank_discharge_a = 0.30f * disch_a + 0.70f * pre_crank_discharge_a;
      }
    }

    if (in_relay_active && !crank_cap_active) {
      crank_cap_active = true;
      cap_relay_on_ms = in_relay_on_ms;
      cap_engage_ms = 0;
      cap_base_discharge_a = pre_crank_i_init ? pre_crank_discharge_a : 0.0f;
      cap_peak_total_a = 0.0f;
      cap_peak_starter_a = 0.0f;
      cap_min_v = 99.0f;
      cap_steady_v_sum = 0.0f;
      cap_steady_i_sum = 0.0f;
      cap_steady_samples = 0;
      cap_engage_latency_ms = 0;
      cap_engaged = false;
      cap_steady_closed = false;
      cap_in_dropout = false;
      cap_had_sensor_err = false;
      cap_dropout_count = 0;
    }

    if (crank_cap_active && in_relay_active) {
      if (sensor_error) {
        cap_had_sensor_err = true;
      }
      if (raw_sample_valid) {
        float total_disch_a = (raw_c < 0.0f) ? -raw_c : 0.0f;
        float starter_a = total_disch_a - cap_base_discharge_a;
        if (starter_a < 0.0f) starter_a = 0.0f;

        if (total_disch_a > cap_peak_total_a) cap_peak_total_a = total_disch_a;
        if (starter_a > cap_peak_starter_a) cap_peak_starter_a = starter_a;
        if (raw_v < cap_min_v) cap_min_v = raw_v;

        uint32_t since_relay_on = ((int32_t)(reg_now - cap_relay_on_ms) > 0)
                                      ? (uint32_t)(reg_now - cap_relay_on_ms)
                                      : 0UL;
        if (!cap_engaged && starter_a >= STARTER_ENGAGE_CURRENT_A) {
          cap_engaged = true;
          cap_engage_ms = reg_now;
          cap_engage_latency_ms = (since_relay_on <= 65535UL) ? (uint16_t)since_relay_on : 65535U;
        }

        if (cap_engaged) {
          uint32_t since_engage = ((int32_t)(reg_now - cap_engage_ms) > 0)
                                      ? (uint32_t)(reg_now - cap_engage_ms)
                                      : 0UL;
          if (!cap_steady_closed &&
              since_engage >= STARTER_INRUSH_SKIP_MS &&
              since_engage <= STARTER_STEADY_END_MS) {
            if (cap_steady_samples >= 12) {
              float running_mean_i = cap_steady_i_sum / (float)cap_steady_samples;
              if (starter_a < 0.50f * running_mean_i) {
                cap_steady_closed = true;
              }
            }
            if (!cap_steady_closed && starter_a >= STARTER_STEADY_MIN_CURRENT_A) {
              cap_steady_v_sum += raw_v;
              cap_steady_i_sum += starter_a;
              cap_steady_samples++;
            }
          } else if (since_engage > STARTER_STEADY_END_MS) {
            cap_steady_closed = true;
          }

          if (since_engage >= STARTER_DROPOUT_ARM_MS) {
            if (!cap_in_dropout) {
              if (starter_a < STARTER_DROPOUT_LOW_A) {
                cap_in_dropout = true;
              }
            } else if (starter_a >= STARTER_DROPOUT_RECOVER_A) {
              cap_in_dropout = false;
              if (cap_dropout_count < 255) {
                cap_dropout_count++;
              }
            }
          }
        }
      }
    } else if (crank_cap_active && !in_relay_active) {
      crank_cap_active = false;
      StarterCrankResult res = {};
      res.peak_total_a = cap_peak_total_a;
      res.peak_starter_a = cap_peak_starter_a;
      res.min_crank_v = (cap_min_v < 90.0f) ? cap_min_v : raw_v;
      res.engage_latency_ms = cap_engage_latency_ms;
      int32_t dur_ms = (int32_t)(in_relay_off_ms - cap_relay_on_ms);
      res.crank_duration_ms = (dur_ms > 0)
                                  ? ((dur_ms <= 65535L) ? (uint16_t)dur_ms : 65535U)
                                  : 0U;
      res.dropout_count = cap_dropout_count;
      res.steady_samples = (cap_steady_samples <= 255) ? (uint8_t)cap_steady_samples : 255U;
      res.engaged = cap_engaged;
      res.sensor_err = cap_had_sensor_err || sensor_error;
      res.outcome = in_crank_outcome;
      res.coolant_temp_c = (int16_t)in_temp;
      res.env_fresh = isCanPacketFresh(reg_now, in_last_env, FRONT_MCU_TIMEOUT_MS);

      if (cap_steady_samples >= STARTER_MIN_STEADY_SAMPLES && cap_steady_i_sum > 1.0f) {
        res.steady_v = cap_steady_v_sum / (float)cap_steady_samples;
        res.steady_i_a = cap_steady_i_sum / (float)cap_steady_samples;
        res.r_eff_mohm = (res.steady_v / res.steady_i_a) * 1000.0f;
        res.valid_r_eff = (res.r_eff_mohm >= STARTER_R_MIN_VALID_MOHM &&
                           res.r_eff_mohm <= STARTER_R_MAX_VALID_MOHM);
      } else {
        res.steady_v = 0.0f;
        res.steady_i_a = 0.0f;
        res.r_eff_mohm = 0.0f;
        res.valid_r_eff = false;
      }

      res.seq = ++cap_seq;
      portENTER_CRITICAL(&dataMux);
      lastStarterResult = res;
      portEXIT_CRITICAL(&dataMux);
    }

    // --- 4. Loop Timing dt Calculation ---
    uint32_t current_micros = micros();
    float dt = (current_micros - last_regulator_time) / 1000000.0f;
    if (last_regulator_time == 0 || dt > 0.1f || dt <= 0.0f) {
      dt = 0.010f; // Default nominal loop time
    }
    last_regulator_time = current_micros;

    // --- 5. Charging Permission & Soft Start Delay ---
    bool engine_charging_allowed = (in_state == STATE_RUNNING);

    if (in_state != STATE_RUNNING) {
      runningStartTime = 0;
    } else if (runningStartTime == 0) {
      runningStartTime = millis();
    }

    bool delay_active = (runningStartTime != 0 &&
                         (millis() - runningStartTime < CHARGE_DELAY_MS));

    // --- 6. Static Baseline Gains (RPM dependence disabled for stabilization) ---
    float Kp_v = base_Kp_v;
    float Ki_v = base_Ki_v;
    float Kp_i = base_Kp_i;
    float Ki_i = base_Ki_i;

    // --- 7. Dual Parallel PI Regulators (CC/CV) ---
    // Voltage PI Controller (Target 13.60V)
    float err_v = REGULATOR_V_TARGET - new_v;
    float p_term_v = Kp_v * err_v;
    float target_pwm_v = p_term_v + integral_v;

    // --- Load-Aware Acceleration De-Excitation (Full Field Cutoff or 0A A/C Mode) ---
    static bool accel_load_active = false;

    bool vac_connected = isCanActiveState(in_state) &&
                         (in_last_vac != 0) &&
                         isCanPacketFresh(reg_now, in_last_vac, FRONT_MCU_TIMEOUT_MS) &&
                         !in_map_fault;
    bool env_connected = isCanActiveState(in_state) &&
                         isCanPacketFresh(reg_now, in_last_env, FRONT_MCU_TIMEOUT_MS);
    bool ac_enabled = env_connected && (in_ac != 0);
    bool voltage_safe_for_cutoff = (new_v >= REGULATOR_V_CUTOFF_FLOOR);
    bool engine_warm = env_connected && (in_temp >= REGULATOR_LOAD_MIN_TEMP_C);

    if (MAP_SENSOR_ENABLED && engine_charging_allowed && vac_connected && engine_warm && !delay_active) {
      if (!accel_load_active) {
        // Trigger load reduction whenever manifold vacuum drops below 2.0 psi and battery voltage is healthy
        if (in_vac < REGULATOR_LOAD_CUTOFF_VAC_PSI && voltage_safe_for_cutoff) {
          accel_load_active = true;
        }
      } else {
        // Disengage load reduction if vacuum recovers above hysteresis threshold or voltage drops below safety floor
        bool vacuum_recovered = (in_vac >= REGULATOR_LOAD_REENGAGE_VAC_PSI);
        if (vacuum_recovered || !voltage_safe_for_cutoff) {
          accel_load_active = false;
        }
      }
    } else {
      accel_load_active = false;
    }

    bool field_cutoff_active = (accel_load_active && !ac_enabled);
    bool ac_zero_charge_active = (accel_load_active && ac_enabled);

    // Current PI Controller (20.00A normal ceiling; 0.00A during high load with A/C ON)
    float active_i_limit = ac_zero_charge_active ? REGULATOR_I_LIMIT_AC_LOAD : REGULATOR_I_LIMIT;
    float err_i = active_i_limit - new_c;
    float p_term_i = Kp_i * err_i;
    float target_pwm_i = p_term_i + integral_i;

    // Minimum Selection Arbiter: Whichever loop commands less field excitation takes control
    float commanded_pwm = (target_pwm_v < target_pwm_i) ? target_pwm_v : target_pwm_i;
    float clamped_pwm = constrain(commanded_pwm, 0.0f, 1023.0f);

    // --- 8. Field Soft-Start Slew Rate Limiter & Integrator Anti-Windup ---
    float max_step_up = REGULATOR_RAMP_UP_PER_SEC * dt;
    float max_step_down = REGULATOR_RAMP_DOWN_PER_SEC * dt;

    bool hard_reset = (!engine_charging_allowed || delay_active || sensor_error || relay_latched);
    if (hard_reset) {
      active_pwm = 0.0f;
      integral_v = 0.0f;
      integral_i = 0.0f;
    } else if (field_cutoff_active) {
      // Cut field excitation immediately for zero alternator drag, but freeze integrators
      // at their pre-cutoff operating point so charging recovers without a 10s ramp delay
      active_pwm = 0.0f;
    } else {
      bool slew_limiting_up = (clamped_pwm > active_pwm + max_step_up);

      // Bumpless Transfer & Anti-Windup Tracking (paused during cutoff and upward slew limiting):
      if (target_pwm_v <= target_pwm_i) {
        // Voltage loop is in control (CV Mode)
        bool saturated = ((clamped_pwm >= 1023.0f || slew_limiting_up) && err_v > 0.0f) ||
                         (clamped_pwm <= 0.0f && err_v < 0.0f);
        if (isfinite(err_v) && isfinite(dt) && !saturated) {
          integral_v += (Ki_v * err_v * dt);
        }
        integral_v = constrain(integral_v, 0.0f, 1023.0f);
        // Track current integrator to active output to prevent windup while in CV mode
        if (!slew_limiting_up || clamped_pwm < integral_i) {
          integral_i = clamped_pwm;
        }
      } else {
        // Current loop is in control (CC Mode)
        bool saturated = ((clamped_pwm >= 1023.0f || slew_limiting_up) && err_i > 0.0f) ||
                         (clamped_pwm <= 0.0f && err_i < 0.0f);
        if (isfinite(err_i) && isfinite(dt) && !saturated) {
          integral_i += (Ki_i * err_i * dt);
        }
        integral_i = constrain(integral_i, 0.0f, 1023.0f);
        // Track voltage integrator to active output to allow current loop full headroom up to 20A
        if (!slew_limiting_up || clamped_pwm < integral_v) {
          integral_v = clamped_pwm;
        }
      }

      if (slew_limiting_up) {
        active_pwm += max_step_up;
      } else if (clamped_pwm < active_pwm - max_step_down) {
        active_pwm -= max_step_down;
      } else {
        active_pwm = clamped_pwm;
      }
    }

    int field_pwm = (int)constrain(active_pwm, 0.0f, 1023.0f);
    ledcWrite(0, field_pwm);

    // --- 9. Diagnostics & Malfunction Detection ---
    unsigned long diag_now = millis();
    uint32_t diag_dt_ms = (last_diag_time == 0) ? 10UL : (uint32_t)(diag_now - last_diag_time);
    if (diag_dt_ms > 250UL) {
      diag_dt_ms = 250UL;
    }
    last_diag_time = diag_now;

    // 9a. Severe hardware/sensor fault (emergency relay latched due to >=14.2V, >40A, or I2C/ADC failure)
    // Hold briefly past the 5s relay cooldown so cyclic faults (e.g. shorted field MOSFET) don't drop the warning.
    if (severe_failure) {
      last_severe_time = diag_now;
      severe_hold_active = true;
    } else if (severe_hold_active && (diag_now - last_severe_time >= CHARGE_SEVERE_HOLD_MS)) {
      severe_hold_active = false;
    }
    bool severe_warning = severe_failure || severe_hold_active;

    // 9b. Sustained overvoltage fault (>= 14.0V for 2s; clears with hysteresis below 13.85V)
    if (new_v >= REGULATOR_V_TARGET + 0.40f) {
      if (overvoltage_start_time == 0) {
        overvoltage_start_time = (diag_now != 0) ? diag_now : 1;
      } else if (diag_now - overvoltage_start_time >= CHARGE_OVERVOLTAGE_DELAY_MS) {
        overvoltage_latched = true;
      }
    } else {
      overvoltage_start_time = 0;
      if (new_v < REGULATOR_V_TARGET + 0.25f) {
        overvoltage_latched = false;
      }
    }

    // 9c. Broken belt / dead alternator / open field circuit detection:
    // Sticky accumulator pauses during gear shifts, idle stops, accel cutoffs, and auto-stop
    // so transient pauses don't erase fault progress; clears only on confirmed healthy charging or engine stop.
    bool undercharging_detected = false;
    if (frontMcuConnected && in_state == STATE_RUNNING && !delay_active && !accel_load_active) {
      if (field_pwm >= 900 && in_rpm >= 1200 &&
          new_v <= (REGULATOR_V_TARGET - 0.40f) && new_c <= 0.0f) {
        undercharging_detected = true;
      }
    }

    bool charging_confirmed = (in_state == STATE_RUNNING && !delay_active && !accel_load_active &&
                               !sensor_error && !relay_latched &&
                               (new_v >= (REGULATOR_V_TARGET - 0.20f) || new_c >= 2.0f));

    if (in_state != STATE_RUNNING && in_state != STATE_AUTO_STOP) {
      undercharging_accum_ms = 0;
      charging_ok_ms = 0;
      unresponsiveness_fault = false;
    } else if (undercharging_detected) {
      charging_ok_ms = 0;
      if (undercharging_accum_ms < CHARGE_UNDERCHARGE_PERSIST_MS) {
        undercharging_accum_ms += diag_dt_ms;
        if (undercharging_accum_ms >= CHARGE_UNDERCHARGE_PERSIST_MS) {
          undercharging_accum_ms = CHARGE_UNDERCHARGE_PERSIST_MS;
          unresponsiveness_fault = true;
        }
      } else {
        unresponsiveness_fault = true;
      }
    } else if (charging_confirmed) {
      uint32_t decay = diag_dt_ms * 2UL;
      undercharging_accum_ms = (undercharging_accum_ms > decay) ? (undercharging_accum_ms - decay) : 0;
      if (unresponsiveness_fault) {
        charging_ok_ms += diag_dt_ms;
        if (charging_ok_ms >= CHARGE_RECOVERY_CONFIRM_MS) {
          unresponsiveness_fault = false;
          undercharging_accum_ms = 0;
          charging_ok_ms = 0;
        }
      }
    } else {
      charging_ok_ms = 0;
    }

    // 9d. Battery low while running (< 12.2V continuously for BATTERY_LOW_DELAY_MS)
    bool battery_low_fault = false;
    if (in_state == STATE_RUNNING && !delay_active && !sensor_error &&
        new_v < (REGULATOR_V_TARGET - 1.40f)) {
      if (battery_low_start_time == 0) {
        battery_low_start_time = (diag_now != 0) ? diag_now : 1;
      } else if (diag_now - battery_low_start_time >= BATTERY_LOW_DELAY_MS) {
        battery_low_fault = true;
      }
    } else {
      battery_low_start_time = 0;
    }

    uint8_t next_charge_state = 0;
    if (severe_warning || overvoltage_latched || unresponsiveness_fault) {
      next_charge_state = 1; // Confirmed charging system malfunction warning
    } else if (battery_low_fault) {
      next_charge_state = 2; // Confirmed battery low warning (< 12.2V while running)
    } else {
      next_charge_state = 0; // Normal
    }

    // --- 10. Sample Auxiliary Fuel ADC (every 2s, immediately on first pass; skipped during fast crank capture) ---
    static bool fuel_first_read_pending = true;
    int16_t new_fuel_val = -1;
    if (!crank_fast_mode && (fuel_first_read_pending || current_micros - last_fuel_time >= FUEL_ADC_INTERVAL_US)) {
      int16_t read_f = readADC_Timeout(2, 25);
      if (read_f >= 0) {
        new_fuel_val = read_f;
        fuel_first_read_pending = false;
      }
      last_fuel_time = current_micros;
    }

    // --- 11. Write Shared Variables Atomically ---
    portENTER_CRITICAL(&dataMux);
    voltage_filtered = new_v;
    current_A_filtered = new_c;
    charge_state = next_charge_state;
    if (new_fuel_val >= 0) {
      ads_fuel = new_fuel_val;
    }
    portEXIT_CRITICAL(&dataMux);

    vTaskDelay(pdMS_TO_TICKS(crank_fast_mode ? 1 : 5)); // Yield CPU to Core 0 scheduler
    last_regulator_heartbeat = millis(); // Signal Core 1 that regulator is alive
    esp_task_wdt_reset();
  }
}
