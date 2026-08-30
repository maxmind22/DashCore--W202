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
}

void regulatorTask(void *pvParameters) {
  esp_task_wdt_add(NULL);
  int consecutive_failures = 0;

  const float voltage_alpha = 0.8f;               // 0.3
  const float current_sensor_offset_mv = 2500.0f; // 2519
  const float current_sensor_mV_per_A = 4.0f; // 4.0f; // mV per Amp (FS500E2T)
  const float current_limit_upper = 20.000f;  // start pulling back above this
  const float current_alpha = 0.2f;
  // const float current_alpha = 0.3f; // previously 0.2
  const float base_Kp = 30.000f;
  const float base_Ki = 5.0f;
  const float Kd = 0.0f;

  static float integral_error = 0.0f;
  static unsigned long last_regulator_time = 0;
  static uint32_t last_fuel_time = 0;
  static uint32_t last_stack_check = 0;

  static float last_error = 0.0f;
  const float v_target = 13.6000f; // Max voltage is 13.6V
  SystemState local_state = STATE_SLEEP;
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
    // uint32_t start = micros();

    //-------------------- Voltage measurement
    int voltage_raw = adc.readADC_SingleEnded(0);
    int16_t current_raw_t = adc.readADC_SingleEnded(1);

    // Glitch protection: only update if reading is sane
    esp_task_wdt_reset(); // Reset WDT after potentially blocking I2C reads

    if (voltage_raw > 0 && current_raw_t > 0) {
      consecutive_failures = 0;
      // GAIN_ONE is 0.125mV per bit (0.000125V)
      // Pre-calculated voltage multiplier: 0.000125f * 4.33766718718175f
      float voltage = voltage_raw * 0.0005422084f;
      float new_v =
          voltage_alpha * voltage + (1.0f - voltage_alpha) * voltage_filtered;

      // Pre-calculated current multiplier: 0.000125V * 1000 = 0.125mV per bit
      float current_mv = current_raw_t * 0.125f;
      float current_raw =
          (current_mv - current_sensor_offset_mv) / current_sensor_mV_per_A;
      float new_c = current_alpha * current_raw +
                    (1.0f - current_alpha) * current_A_filtered;

      portENTER_CRITICAL(&dataMux);
      voltage_filtered = new_v;
      current_A_filtered = new_c;
      portEXIT_CRITICAL(&dataMux);
    } else {
      consecutive_failures++;
      if (consecutive_failures > 50) {
        // I2C Bus Recovery
        Wire.end();
        vTaskDelay(pdMS_TO_TICKS(10));              // Yield CPU properly instead of blocking delay()
        recoverI2CBus(21, 22); // Toggle SCL to release any stuck I2C slave
        Wire.begin();
        Wire.setClock(100000);
        Wire.setTimeOut(20); // Ensure I2C transaction timeout is set in recovery
        adc.begin();
        adc.setGain(GAIN_ONE);
        adc.setDataRate(RATE_ADS1115_250SPS);
        consecutive_failures = 0;
      }
    }

    // --- Evaluate charging system state ---
    bool severe_failure = false;
    bool sensor_error = (consecutive_failures > 5);

    // 1. Emergency hard cut on severe overvoltage, overcurrent, or sensor
    // failure
    if (voltage_filtered >= v_target + 0.6f || current_A_filtered > EMERGENCY_OVERCURRENT_A ||
        sensor_error) {
      digitalWriteFast(field_relay_pin, HIGH);
      severe_failure = true;
    } else
      digitalWriteFast(field_relay_pin, LOW);

    // 2. Logical malfunctions (regulator output doesn't match expected
    // response)
    // - High voltage/current despite commanding low field (runaway/short)
    // - Low voltage and no current despite commanding high field (open
    // circuit/broken belt) We use <= 0.5A instead of 0.0A to account for small
    // ADC noise around zero
    portENTER_CRITICAL(&dataMux);
    local_rpm = rpm;
    local_state = currentState;
    unsigned long local_last_packet = lastPacketTime;
    portEXIT_CRITICAL(&dataMux);

    bool frontMcuConnected = (millis() - local_last_packet < FRONT_MCU_CAN_TIMEOUT_MS);

    bool logical_failure =
        ((voltage_filtered >= v_target + 0.4f || current_A_filtered >= EMERGENCY_OVERCURRENT_A) ||
         (frontMcuConnected && voltage_filtered <= v_target - 0.2f && current_A_filtered <= 0.0f &&
          (local_state == STATE_RUNNING || local_rpm > ENGINE_ACTIVE_RPM_THRESHOLD)));

    // Consolidate state hierarchy
    int next_charge_state = 0;
    if (severe_failure || logical_failure) {
      next_charge_state = 1; // charging malfunction warning
    } else if (voltage_filtered < v_target - 1.4) // If voltage is very low while engine is running.
    {
      next_charge_state = 2; // battery low warning
    } else {
      next_charge_state = 0; // normal
    }

    portENTER_CRITICAL(&dataMux);
    charge_state = next_charge_state;
    if (charge_state == 0) {
      last_charge = millis();
    }
    portEXIT_CRITICAL(&dataMux);
    // --- Target settings ---

    // Calculate dt (time elapsed in seconds) to make PID immune to loop delays
    uint32_t current_micros = micros();
    float dt = (current_micros - last_regulator_time) / 1000000.0f;
    if (last_regulator_time == 0 || dt > 0.1f || dt <= 0.0f) {
      dt = 0.020f; // Default to 20ms on first run or severe lag (typical loop time)
    }
    last_regulator_time = current_micros;

    // 1. Seamless CC/CV Error
    // We increase PWM until we hit EITHER 13.6V or our 20A limit.
    // The most restrictive target (smallest error) commands the loop seamlessly.
    float err_v = (v_target - voltage_filtered) * 100;
    float err_i = (current_limit_upper - current_A_filtered) * 10;
    float err = (err_v < err_i) ? err_v : err_i;

    float p_term = base_Kp * err;

    if (isfinite(err) && isfinite(dt)) {
      integral_error += (err * dt);
    }

    if (!isfinite(integral_error))
      integral_error = 0.0f;
    if (integral_error > 1023.0f)
      integral_error = 1023.0f;
    if (integral_error < 0.0f)
      integral_error = 0.0f;

    float i_term = base_Ki * integral_error;
    last_error = err;

    int voltage_pwm = constrain((int)(p_term + i_term), 0, 1023);

    // 2. Absolute Hard Voltage Ceiling (Safety Override)
    // We already have CC/CV in the PID above. These overrides are just to
    // provide an extra push if we cross the absolute limits, but they must
    // not be so aggressive that they cause oscillation.
    int safety_pwm = voltage_pwm;
    // Extra pullback if overvoltage occurs (Safety backup for PID)
    if (voltage_filtered >= v_target + 0.2f) {
      float ov_err = ((v_target + 0.2f) - voltage_filtered) * 100;
      safety_pwm = safety_pwm + (int)(200.0f * ov_err);
    }
    // Extra pullback if overcurrent occurs (Safety backup for PID)
    if (current_A_filtered >= current_limit_upper + 10.0f) {
      float oi_err = ((current_limit_upper + 10.0f) - current_A_filtered) * 10;
      safety_pwm = safety_pwm + (int)(200.0f * oi_err); // Mild push-back
    }

    // Reset running start time if engine is not running or not in RUNNING state
    if (local_state != STATE_RUNNING) {
      runningStartTime = 0;
    } else if (runningStartTime == 0) {
      runningStartTime = millis();
    }

    bool delay_active = (runningStartTime != 0 &&
                         (millis() - runningStartTime < CHARGE_DELAY_MS));

    // Allow charging strictly in STATE_RUNNING (disabled during cranking, standby, ACC, IGN, and auto-stop)
    bool engine_charging_allowed = (local_state == STATE_RUNNING);

    // Force field coil off if engine is not running, during cranking/start delay (CHARGE_DELAY_MS), or sensor error occurs
    if (!engine_charging_allowed || delay_active || sensor_error) {
      field_pwm = 0;
      integral_error = 0.0f; // Reset integrator
    } else {
      field_pwm = constrain(safety_pwm, 0, 1023);
    }

    ledcWrite(0, field_pwm);

    // --- Auxiliary Sensors (Not used for regulator control, but read here to
    // avoid I2C contention) ---
    if (current_micros - last_fuel_time >= 100000) {
      int new_fuel = adc.readADC_SingleEnded(2);
      if (new_fuel >= 0) // Only update on valid I2C read
      {
        portENTER_CRITICAL(&dataMux);
        ads_fuel = new_fuel;
        portEXIT_CRITICAL(&dataMux);
      }
      last_fuel_time = current_micros;
    }

    // --- Stack monitoring ---
    // if (current_micros - last_stack_check >= 1000000) // Every 1 second
    // {
    //   free_stack = uxTaskGetStackHighWaterMark(NULL);
    //   last_stack_check = current_micros;
    // }
    // task_duration = micros() - start;
    vTaskDelay(pdMS_TO_TICKS(20)); // Use vTaskDelay instead of vTaskDelayUntil to always yield
    last_regulator_heartbeat = millis(); // Signal Core 1 that regulator is alive
    esp_task_wdt_reset();
  }
}
