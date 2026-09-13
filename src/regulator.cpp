#include "regulator.h"

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
  field_pwm = 0;
  digitalWriteFast(field_relay_pin, LOW);
}


void regulatorTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  int consecutive_failures = 0;

  const float voltage_alpha = 0.8f;
  const float current_sensor_offset_mv = 2495.44f; // Calibrated: -0.30A target vs -1.44A reading (nominal 2500.0f)
  const float current_sensor_mV_per_A = 4.0f; // mV per Amp (FS500E2T)
  const float current_alpha = 0.40f; // Faster response (reduces phase delay to ~50ms) to eliminate rocking

  // Baseline PID Gains (Tuned for physical plant: ~20 PWM counts per Amp)
  const float base_Kp_v = 900.0f;  // PWM counts per Volt error (~90 PWM for 0.1V error, suppresses ripple)
  const float base_Ki_v = 150.0f;  // PWM counts per Volt-second error
  const float base_Kp_i = 12.0f;   // PWM counts per Amp error (critically damped, eliminates 13A-22A oscillation)
  const float base_Ki_i = 3.0f;    // PWM counts per Amp-second error (smooth zero-steady-state tracking)

  // Controller State Variables
  static float integral_v = 0.0f;
  static float integral_i = 0.0f;
  static float active_pwm = 0.0f;
  static unsigned long last_regulator_time = 0;
  static uint32_t last_fuel_time = 0;
  static unsigned long undercharging_start_time = 0;
  static unsigned long relay_trip_time = 0;

  unsigned long runningStartTime = 0;
  const unsigned long CHARGE_DELAY_MS = 2000; // 2 seconds delay before charging starts

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
    int voltage_raw = adc.readADC_SingleEnded(0);
    int16_t current_raw_t = adc.readADC_SingleEnded(1);

    esp_task_wdt_reset(); // Reset WDT after potentially blocking I2C reads

    float new_v = voltage_filtered;
    float new_c = current_A_filtered;

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
        adc.setDataRate(RATE_ADS1115_250SPS);
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
          new_v >= REGULATOR_V_TARGET + 0.20f) {
        relay_latched = true;
      } else {
        relay_trip_time = 0;
      }
    }

    bool severe_failure = relay_latched;
    if (relay_latched) {
      digitalWriteFast(field_relay_pin, HIGH); // Open high-side safety relay
    } else {
      digitalWriteFast(field_relay_pin, LOW);  // Closed (normal operation)
    }

    // --- 3. Snapshot Shared Engine State ---
    portENTER_CRITICAL(&dataMux);
    uint16_t in_rpm = rpm;
    SystemState in_state = currentState;
    unsigned long in_last_packet = lastPacketTime;
    portEXIT_CRITICAL(&dataMux);

    bool frontMcuConnected = (millis() - in_last_packet < FRONT_MCU_CAN_TIMEOUT_MS);

    // --- 4. Loop Timing dt Calculation ---
    uint32_t current_micros = micros();
    float dt = (current_micros - last_regulator_time) / 1000000.0f;
    if (last_regulator_time == 0 || dt > 0.1f || dt <= 0.0f) {
      dt = 0.025f; // Default nominal loop time
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

    // Current PI Controller (Ceiling 20.00A charge into battery)
    float err_i = REGULATOR_I_LIMIT - new_c;
    float p_term_i = Kp_i * err_i;
    float target_pwm_i = p_term_i + integral_i;

    // Minimum Selection Arbiter: Whichever loop commands less field excitation takes control
    float commanded_pwm = (target_pwm_v < target_pwm_i) ? target_pwm_v : target_pwm_i;
    float clamped_pwm = constrain(commanded_pwm, 0.0f, 1023.0f);

    // Bumpless Transfer & Anti-Windup Tracking:
    if (target_pwm_v <= target_pwm_i) {
      // Voltage loop is in control (CV Mode)
      bool saturated = (clamped_pwm >= 1023.0f && err_v > 0.0f) ||
                       (clamped_pwm <= 0.0f && err_v < 0.0f);
      if (isfinite(err_v) && isfinite(dt) && !saturated) {
        integral_v += (Ki_v * err_v * dt);
      }
      integral_v = constrain(integral_v, 0.0f, 1023.0f);
      // Track current integrator to active output to prevent windup while in CV mode
      integral_i = clamped_pwm;
    } else {
      // Current loop is in control (CC Mode)
      bool saturated = (clamped_pwm >= 1023.0f && err_i > 0.0f) ||
                       (clamped_pwm <= 0.0f && err_i < 0.0f);
      if (isfinite(err_i) && isfinite(dt) && !saturated) {
        integral_i += (Ki_i * err_i * dt);
      }
      integral_i = constrain(integral_i, 0.0f, 1023.0f);
      // Track voltage integrator to active output to allow current loop full headroom up to 20A
      integral_v = clamped_pwm;
    }

    // --- 8. Field Soft-Start Slew Rate Limiter ---
    float max_step_up = REGULATOR_RAMP_UP_PER_SEC * dt;
    float max_step_down = REGULATOR_RAMP_DOWN_PER_SEC * dt;

    if (!engine_charging_allowed || delay_active || sensor_error || relay_latched) {
      active_pwm = 0.0f;
      integral_v = 0.0f;
      integral_i = 0.0f;
    } else {
      if (clamped_pwm > active_pwm + max_step_up) {
        active_pwm += max_step_up;
      } else if (clamped_pwm < active_pwm - max_step_down) {
        active_pwm -= max_step_down;
      } else {
        active_pwm = clamped_pwm;
      }
    }

    field_pwm = (int)constrain(active_pwm, 0.0f, 1023.0f);
    ledcWrite(0, field_pwm);

    // --- 9. Diagnostics & Malfunction Detection ---
    bool overvoltage_fault = (new_v >= REGULATOR_V_TARGET + 0.40f);

    // Broken belt / open circuit detection:
    // Only evaluate if Front MCU is online, engine is running, start delay passed,
    // field is near 100% duty (saturated), engine RPM >= 1200,
    // and voltage/current remain completely unresponsive.
    bool undercharging_detected = false;
    if (frontMcuConnected && in_state == STATE_RUNNING && !delay_active) {
      if (field_pwm >= 900 && in_rpm >= 1200 &&
          new_v <= (REGULATOR_V_TARGET - 0.40f) && new_c <= 0.0f) {
        undercharging_detected = true;
      }
    }

    if (undercharging_detected) {
      if (undercharging_start_time == 0) undercharging_start_time = millis();
    } else {
      undercharging_start_time = 0;
    }

    // Persist for 3 continuous seconds before asserting malfunction
    bool unresponsiveness_fault = (undercharging_start_time != 0 &&
                                   (millis() - undercharging_start_time >= 3000));

    bool logical_failure = overvoltage_fault || unresponsiveness_fault;

    int next_charge_state = 0;
    if (severe_failure || logical_failure) {
      next_charge_state = 1; // Charging system malfunction warning
    } else if (in_state == STATE_RUNNING && !delay_active &&
               new_v < (REGULATOR_V_TARGET - 1.40f)) {
      next_charge_state = 2; // Battery low warning (< 12.2V while running)
    } else {
      next_charge_state = 0; // Normal
    }

    // --- 10. Sample Auxiliary Fuel ADC (every 2s) ---
    int new_fuel_val = -1;
    if (current_micros - last_fuel_time >= FUEL_ADC_INTERVAL_US) {
      int read_f = adc.readADC_SingleEnded(2);
      if (read_f >= 0) {
        new_fuel_val = read_f;
      }
      last_fuel_time = current_micros;
    }

    // --- 11. Write Shared Variables Atomically ---
    portENTER_CRITICAL(&dataMux);
    voltage_filtered = new_v;
    current_A_filtered = new_c;
    charge_state = next_charge_state;
    if (charge_state == 0) {
      last_charge = millis();
    }
    if (new_fuel_val >= 0) {
      ads_fuel = new_fuel_val;
    }
    portEXIT_CRITICAL(&dataMux);

    vTaskDelay(pdMS_TO_TICKS(20)); // Yield CPU to Core 0 scheduler
    last_regulator_heartbeat = millis(); // Signal Core 1 that regulator is alive
    esp_task_wdt_reset();
  }
}
