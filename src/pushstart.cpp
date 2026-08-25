#include "pushstart.h"
#include "fuel.h" // For resetFuelTripData

static LockRelayState lockRelayState = LOCK_IDLE;
static unsigned long lockRelayStartTime = 0;

static unsigned long unlockFirstPulseTime = 0;
static unsigned long unlockLastPulseTime = 0;
static uint8_t unlockPulseCount = 0;
static bool lastUnlockPinState = LOW;
static unsigned long lastUnlockEdgeTime = 0;

static ToneState toneState = TONE_IDLE;
static unsigned long tonePhaseStart = 0;
static int toneBeepsRemaining = 0;
static unsigned long toneOnMs = 0;
static unsigned long toneOffMs = 0;

void triggerLockPulse(unsigned long now)
{
  if (lockRelayState == LOCK_IDLE && !vehicleLockDisabled)
  {
    pinMode(PIN_RELAY_LOCK, OUTPUT);
    digitalWrite(PIN_RELAY_LOCK, HIGH);
    lockRelayStartTime = (now != 0) ? now : millis();
    lockRelayState = LOCK_PULSE_ACTIVE;
  }
}

void updateLockRelay(unsigned long now)
{
  if (now == 0)
    now = millis();
  if (lockRelayState == LOCK_PULSE_ACTIVE &&
      (now - lockRelayStartTime >= 200))
  {
    digitalWrite(PIN_RELAY_LOCK, LOW);
    pinMode(PIN_RELAY_LOCK, INPUT); // Float pin to save power
    lockRelayState = LOCK_IDLE;
  }
}

void queueTone(int beeps, unsigned long onMs, unsigned long offMs, unsigned long now)
{
  toneBeepsRemaining = beeps;
  toneOnMs = onMs;
  toneOffMs = offMs;
  digitalWriteFast(buzzer_pin, HIGH);
  tonePhaseStart = (now != 0) ? now : millis();
  toneState = TONE_ON;
}

void updateToneStateMachine(unsigned long now)
{
  if (toneState == TONE_IDLE)
    return;
  if (now == 0)
    now = millis();
  if (toneState == TONE_ON && (now - tonePhaseStart >= toneOnMs))
  {
    digitalWriteFast(buzzer_pin, LOW);
    toneBeepsRemaining--;
    if (toneBeepsRemaining <= 0)
    {
      toneState = TONE_IDLE;
    }
    else
    {
      tonePhaseStart = now;
      toneState = TONE_OFF;
    }
  }
  else if (toneState == TONE_OFF && (now - tonePhaseStart >= toneOffMs))
  {
    digitalWriteFast(buzzer_pin, HIGH);
    tonePhaseStart = now;
    toneState = TONE_ON;
  }
}

bool isTonePlaying() { return toneState != TONE_IDLE; }

void playUnlockToggleTone(bool disabled)
{
  if (disabled)
    queueTone(2, 200, 200); // 2 short beeps
  else
    queueTone(1, 300, 0); // 1 long beep
}

void playLockdownToggleTone(bool lockdownActive)
{
  if (lockdownActive)
    queueTone(3, 200, 200); // 3 short beeps
  else
    queueTone(1, 400, 0); // 1 long beep
}

void playStartStopToggleTone(bool disabled)
{
  if (disabled)
    queueTone(2, 150, 150); // 2 short beeps
  else
    queueTone(1, 350, 0); // 1 long beep
}

void playAuthWarningTone()
{
  queueTone(2, 100, 100); // 2 quick beeps
}

void processUnlockSignals(unsigned long now)
{
  if (now == 0)
    now = millis();
  bool currentUnlockPinState = digitalRead(PIN_WAKE_UNLOCK);

  // Optocoupler output goes HIGH on unlock pulse (active high into ESP32)
  if (currentUnlockPinState == HIGH && lastUnlockPinState == LOW)
  {
    if (now - lastUnlockEdgeTime >= 150) // 150ms debounce
    {
      lastUnlockEdgeTime = now;
      unlockLastPulseTime = now;

      if (unlockPulseCount == 0)
      {
        unlockFirstPulseTime = now;
      }
      unlockPulseCount++;
    }
  }
  lastUnlockPinState = currentUnlockPinState;

  // Evaluate the pulse burst after 1.5 seconds of inactivity
  if (unlockPulseCount > 0 && (now - unlockLastPulseTime >= 1500))
  {
    unsigned long burstDuration = unlockLastPulseTime - unlockFirstPulseTime;
    if (burstDuration <= 10000)
    {
      if (unlockPulseCount == 3)
      {
        vehicleLockDisabled = !vehicleLockDisabled;
        playUnlockToggleTone(vehicleLockDisabled);
      }
      else if (unlockPulseCount == 4)
      {
        engineStartDisabled = !engineStartDisabled;
        playLockdownToggleTone(engineStartDisabled);
      }
      else if (unlockPulseCount == 5)
      {
        autoStartStopDisabled = !autoStartStopDisabled;
        playStartStopToggleTone(autoStartStopDisabled);
      }
    }

    // Reset for next burst
    unlockPulseCount = 0;
    unlockFirstPulseTime = 0;
    unlockLastPulseTime = 0;
  }
}

void setRelays(bool acc, bool ign, bool start)
{
  digitalWrite(PIN_RELAY_ACC, acc ? HIGH : LOW);
  digitalWrite(PIN_RELAY_IGN, ign ? HIGH : LOW);
  digitalWrite(PIN_RELAY_START, start ? HIGH : LOW);
}

void startTVDisplay()
{
  dac_output_enable(DAC_CHANNEL_1);
  dac_i2s_enable();
  I2S0.conf.tx_start = 1;
  I2S0.out_link.start = 1;
}

void stopTVDisplay()
{
  tv.waitForFrame();       // Let any in-flight DMA frame complete first
  I2S0.out_link.start = 0; // Stop DMA linked list (must stop before tx)
  delay(1);
  I2S0.conf.tx_start = 0; // Stop I2S TX
  delay(1);
  dac_output_disable(DAC_CHANNEL_1);
  dac_i2s_disable();
}

void sleepCANController() { mcp2515.setSleepMode(); }

void wakeupCANController()
{
  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalOneShotMode();
}

void enterPowerDownSleep()
{
  // Save trip stats to NVS Flash memory right before shutdown
  Preferences prefs;
  prefs.begin("trip_data", false);
  prefs.putFloat("fuel", total_fuel_liters);
  prefs.putFloat("dist", total_distance_km);
  prefs.putFloat("r_int", compounded_r_int);
  prefs.putFloat("saved", total_fuel_saved_liters);
  prefs.end();

  // --- ORDERED SHUTDOWN: Stop everything safely before deep sleep ---

  // 1. Safely stop the regulator FreeRTOS task first (running on Core 0, does I2C)
  //    Keep the original 1s WDT alive by resetting it in the wait loop.
  if (regulatorTaskHandle != NULL)
  {
    regulatorTaskRunning = false;
    for (int timeout = 0; timeout < 100; timeout++)
    {
      esp_task_wdt_reset(); // Keep current WDT alive while waiting for task exit
      taskYIELD();
      delay(5);
      if (regulatorTaskHandle == NULL)
        break;
    }
    // Use critical section to safely check-and-delete (prevents race with self-deleting task)
    portENTER_CRITICAL(&dataMux);
    TaskHandle_t h = regulatorTaskHandle;
    regulatorTaskHandle = NULL;
    portEXIT_CRITICAL(&dataMux);
    if (h != NULL)
    {
      vTaskDelete(h);
    }
  }

  // 2. Now that regulatorTask is stopped, extend WDT to 5s for remaining shutdown
  esp_task_wdt_delete(NULL);
  esp_task_wdt_deinit();
  esp_task_wdt_init(5, true); // 5s timeout with panic=true
  esp_task_wdt_add(NULL);

  // 3. Turn off field coil PWM and detach LEDC
  ledcWrite(0, 0);
  ledcDetachPin(FIELD_PIN);

  // 4. Stop TV display (I2S DMA) — must stop before deep sleep or DMA crash
  stopTVDisplay();
  delay(10); // Let DMA finish any in-flight transfer

  // 5. Put MCP2515 CAN controller to sleep (SPI device)
  sleepCANController();

  // 6. Close communication buses
  Wire.end();
  SPI.end(); // Stop SPI bus to release SCK, MOSI, MISO
  esp_task_wdt_reset();

  // 6.1 Put communication pins into high-impedance (floating) mode
  // This prevents leakage through level-shifter pull-ups
  pinMode(21, INPUT); // SDA
  pinMode(22, INPUT); // SCL
  pinMode(5, INPUT);  // CS (CAN)
  pinMode(18, INPUT); // SCK
  pinMode(19, INPUT); // MISO
  pinMode(23, INPUT); // MOSI

  // 7. Turn off and de-energize all relays and peripherals
  setRelays(false, false, false);
  digitalWrite(PIN_5V_GATE, LOW); // Disable 5V Relay
  digitalWrite(field_relay_pin, LOW);
  digitalWrite(buzzer_pin, LOW);

  // Hold relay pins LOW during deep sleep to prevent ghost-engagement
  // from noise/moisture on floating pins in the automotive environment
  gpio_hold_en((gpio_num_t)PIN_RELAY_ACC);
  gpio_hold_en((gpio_num_t)PIN_RELAY_IGN);
  gpio_hold_en((gpio_num_t)PIN_RELAY_START);
  gpio_hold_en((gpio_num_t)PIN_5V_GATE);
  gpio_hold_en((gpio_num_t)field_relay_pin);
  gpio_hold_en((gpio_num_t)buzzer_pin);
  gpio_deep_sleep_hold_en();

  // Allow relay switching and vehicle state to settle completely (prevent transient unlock)
  delay(200);

  // 7.1 Activate vehicle locking relay briefly to ensure it remains locked
  // after sleep (if locking is not temporarily disabled)
  if (!vehicleLockDisabled)
  {
    pinMode(PIN_RELAY_LOCK, OUTPUT);
    digitalWrite(PIN_RELAY_LOCK, HIGH); // Ground the lock wire via relay
    delay(200);                         // Ground pulse duration of 200ms
    digitalWrite(PIN_RELAY_LOCK, LOW);
    pinMode(PIN_RELAY_LOCK, INPUT); // Float pin to prevent sleep leakage
  }

  // 8. Turn off the 3.3V digital gate (must happen after locking relay is pulsed)
  digitalWrite(PIN_3V3_DIGITAL_GATE, LOW); // Cut 3.3V pull-ups/shifter
  gpio_hold_en((gpio_num_t)PIN_3V3_DIGITAL_GATE);

  // 10. Disarm WDT before deep sleep
  esp_task_wdt_delete(NULL);
  esp_task_wdt_deinit();

  // 11. Configure RTC wakeup: Wake on HIGH because unlock pulses to 0V (opto output goes HIGH)
  esp_sleep_enable_ext1_wakeup(1ULL << PIN_WAKE_UNLOCK,
                               ESP_EXT1_WAKEUP_ANY_HIGH);

  // 12. Enter deep sleep — CPU halts here, wakes up via reset
  currentState = STATE_SLEEP;
  esp_deep_sleep_start();
}

void setupPushStartPins()
{
  pinMode(PIN_RELAY_ACC, OUTPUT);
  pinMode(PIN_RELAY_IGN, OUTPUT);
  pinMode(PIN_RELAY_START, OUTPUT);
  setRelays(false, false, false);

  pinMode(PIN_RELAY_LOCK, OUTPUT);
  digitalWrite(PIN_RELAY_LOCK, LOW);

  pinMode(PIN_BTN_START, INPUT);
  pinMode(PIN_INPUT_BRAKE, INPUT);
  pinMode(PIN_WAKE_UNLOCK, INPUT);
}

void processPushStart(unsigned long now)
{
  if (now == 0)
    now = millis();
  updateLockRelay(now);
  updateToneStateMachine(now);

  static enum { CRANK_PRIME,
                CRANK_SOLENOID } crankStage = CRANK_PRIME;
  static unsigned long crankStageTime = 0;

  // Continuously monitor unlock pulses while engine is not running.
  if (currentState != STATE_RUNNING)
  {
    processUnlockSignals(now);
  }

  // Edge detection for push button
  static bool lastBtnState = HIGH;
  bool currentBtnState = digitalRead(PIN_BTN_START);
  bool btnPressed = (currentBtnState == LOW && lastBtnState == HIGH);
  lastBtnState = currentBtnState;

  bool brakeHeld = (digitalRead(PIN_INPUT_BRAKE) == LOW);
  int currentRpm = rpm;

  static unsigned long buttonDownTime = 0;
  static bool buttonLongPressHandled = false;

  if (currentBtnState == LOW)
  {
    if (buttonDownTime == 0)
    {
      buttonDownTime = now;
      buttonLongPressHandled = false;
    }
    else if (!buttonLongPressHandled &&
             now - buttonDownTime >= BUTTON_LONGPRESS_RESET_MS)
    {
      buttonLongPressHandled = true;
      if (currentState != STATE_RUNNING && currentState != STATE_CRANKING &&
          spd == 0)
      {

        lastButtonPressTime = now; // avoid immediate short-press transition
        resetFuelTripData(now);
      }
    }
  }
  else
  {
    buttonDownTime = 0;
    buttonLongPressHandled = false;
  }

  if (btnPressed)
  {
    standbyStartTime = now;
  }

  // Boot-lock: Lock 2 minute after booting when in ACC or IGN state
  static bool bootLockDone = false;
  if (!bootLockDone &&
      (currentState == STATE_ACC || currentState == STATE_IGNITION))
  {
    if (now >= 120000) // 2 minutes after booting
    {
      triggerLockPulse(now);
      bootLockDone = true;
    }
  }

  // Drive-lock: Lock after 10 sec every time the vehicle is started & moving
  static bool driveLockTriggered = false;
  static bool driveLockDone = false;
  static unsigned long driveLockTime = 0;

  if (currentState == STATE_RUNNING)
  {
    if (spd > 0 && !driveLockTriggered)
    {
      driveLockTriggered = true;
      driveLockTime = now;
    }

    if (driveLockTriggered && !driveLockDone &&
        (now - driveLockTime >= 10000))
    {
      triggerLockPulse(now);
      driveLockDone = true;
    }
  }
  else
  {
    // Reset drive-lock flags when not in running state
    driveLockTriggered = false;
    driveLockDone = false;
  }

  // Handle sleep timeouts when system is in Standby (OFF) or ACC/Ignition
  if (currentState == STATE_STANDBY)
  {
    if (now - standbyStartTime > STANDBY_TIMEOUT_MS)
    {
      enterPowerDownSleep();
      return;
    }
  }
  else if (currentState == STATE_ACC || currentState == STATE_IGNITION)
  {
    if (now - standbyStartTime > ACCESSORY_TIMEOUT_MS)
    {
      enterPowerDownSleep();
      return;
    }
  }

  // Handle state transitions
  switch (currentState)
  {
  case STATE_SLEEP:
    // Woken up by deep sleep reset (unlock pulse) -> Authenticated
    currentState = STATE_STANDBY;
    standbyStartTime = now;
    lastButtonPressTime = 0; // Clear cooldown on first boot

    wakeupCANController();
    startTVDisplay();
    if (regulatorTaskHandle != NULL)
    {
      vTaskResume(regulatorTaskHandle);
    }
    break;

  case STATE_STANDBY:
  {
    // Serial.println("OFF");
    // Relays: ACC OFF, IGN OFF, START OFF
    static bool standbyBrakeCheckPending = false;
    static unsigned long standbyBrakeCheckTime = 0;

    if (standbyBrakeCheckPending)
    {
      setRelays(true, true, false); // Turn on ACC & IGN to power brake circuit
      if (digitalRead(PIN_INPUT_BRAKE) == LOW)
      {
        standbyBrakeCheckPending = false;
        if (!engineStartDisabled)
        {
          currentState = STATE_CRANKING;
        }
        else
        {
          // Serial.println("[LOCKDOWN] Engine start blocked! PLZ AUTHENTICATE");
          playAuthWarningTone();
          currentState = STATE_STANDBY;
        }
      }
      else if (now - standbyBrakeCheckTime >= BRAKE_CHECK_SETTLE_MS)
      {
        standbyBrakeCheckPending = false;
        currentState = STATE_ACC; // 1st press (without brake) goes to ACC (POS1)
        standbyStartTime = now;   // Reset 2-min timeout
        stoppedToAcc = false;     // Reset flag as we didn't stop from a running engine to ACC
      }
    }
    else
    {
      setRelays(false, false, false);

      if (btnPressed && (now - lastButtonPressTime >= BUTTON_COOLDOWN_MS))
      {
        lastButtonPressTime = now;
        // Temporarily turn on ACC & IGN to power the brake switch circuit
        setRelays(true, true, false);
        standbyBrakeCheckPending = true;
        standbyBrakeCheckTime = now;
      }
    }
    break;
  }

  case STATE_ACC:
  {
    // Serial.println("ACC");
    // Relays: ACC ON, IGN OFF, START OFF (POS1)
    // Non-blocking brake check: after button press, IGN turns on
    // to let relay/optocoupler settle, then brake is read continuously during BRAKE_CHECK_SETTLE_MS window
    static bool accBrakeCheckPending = false;
    static unsigned long accBrakeCheckTime = 0;

    if (accBrakeCheckPending)
    {
      setRelays(true, true, false); // Keep IGN on during settle wait
      if (digitalRead(PIN_INPUT_BRAKE) == LOW)
      {
        accBrakeCheckPending = false;
        if (!engineStartDisabled)
        {
          currentState = STATE_CRANKING;
          stoppedToAcc = false;
        }
        else
        {
          Serial.println("[LOCKDOWN] Engine start blocked! PLZ AUTHENTICATE");
          playAuthWarningTone();
          currentState = STATE_ACC;
        }
      }
      else if (now - accBrakeCheckTime >= BRAKE_CHECK_SETTLE_MS)
      {
        accBrakeCheckPending = false;
        if (stoppedToAcc)
        {
          // Serial.println("[DEBUG] Failed to start from ACC: Brake not detected within window. Going to STANDBY.");
          currentState = STATE_STANDBY; // Go to OFF (standby)
          stoppedToAcc = false;
        }
        else
        {
          currentState = STATE_IGNITION; // 2nd press (without brake) goes to POS2 (IGNITION)
        }
        standbyStartTime = now; // Reset 2-min timeout
      }
    }
    else
    {
      setRelays(true, false, false);

      if (btnPressed && (now - lastButtonPressTime >= BUTTON_COOLDOWN_MS))
      {
        lastButtonPressTime = now;
        // Temporarily turn on IGN to power the brake switch circuit
        setRelays(true, true, false);
        accBrakeCheckPending = true;
        accBrakeCheckTime = now;
      }
    }
    break;
  }

  case STATE_IGNITION:
    // Relays: ACC ON, IGN ON, START OFF (POS2)
    setRelays(true, true, false);

    if (btnPressed && (now - lastButtonPressTime >= BUTTON_COOLDOWN_MS))
    {
      lastButtonPressTime = now;
      if (brakeHeld)
      {
        if (!engineStartDisabled)
        {
          currentState = STATE_CRANKING;
        }
        else
        {
          playAuthWarningTone();
        }
      }
      else
      {
        currentState = STATE_STANDBY; // 3rd press (without brake) goes to OFF (STANDBY)
        standbyStartTime = now;       // Reset 2-min timeout
      }
    }
    break;

  case STATE_CRANKING:
    if (engineStartDisabled)
    {
      // Serial.println("[DEBUG] Cranking aborted: Engine start disabled (Lockdown)");
      setRelays(true, false, false); // Abort cranking immediately
      currentState = STATE_ACC;
      playAuthWarningTone();
      crankStage = CRANK_PRIME;
      crankStageTime = 0;
      isEcoRestart = false;
      ecoInjCutActive = false;
      break;
    }
    // Non-blocking stage machine for cranking sequence

    if (crankStage == CRANK_PRIME)
    {
      // Step 1: Go to POS2 (ACC & IGN ON) for fuel pump/ECU prime
      setRelays(true, true, false);
      if (crankStageTime == 0)
      {
        crankStageTime = now;
      }
      unsigned long requiredPrime = isEcoRestart ? ECO_CRANK_PRIME_MS : COLD_CRANK_PRIME_MS;
      if (now - crankStageTime >= requiredPrime)
      {
        crankStage = CRANK_SOLENOID;
        crankStageTime = now; // Reset timer for max crank limit
      }
    }
    else if (crankStage == CRANK_SOLENOID)
    {
      // Step 2: Engage starter solenoid (ACC ON, IGN ON, START ON)
      setRelays(true, true, true);

      // Only evaluate RPM after a minimum crank time to avoid noise spikes
      if ((now - crankStageTime >= MIN_CRANK_TIME_MS) && (currentRpm > ENGINE_STARTED_RPM))
      {
        // Engine started successfully
        currentState = STATE_RUNNING;
        setRelays(true, true, false); // Disengage starter, keep ACC/IGN on
        lastEngineStartTime = now;    // Record start time for cooldown tracking
        standstillStartTime = 0;
        isEcoRestart = false;
        ecoInjCutActive = false;
        crankStage = CRANK_PRIME;     // Reset stages
        crankStageTime = 0;
      }
      else if (now - crankStageTime > MAX_CRANK_TIME_MS)
      {
        // Cranking failed or timed out (5s safety cutoff)
        Serial.println("[DEBUG] Cranking aborted: Timed out (exceeded MAX_CRANK_TIME_MS)");
        setRelays(true, false, false); // Disengage starter to ACC for another try (w202 prevent double starting)
        currentState = STATE_ACC;
        standbyStartTime = now; // Reset 2-min timeout
        isEcoRestart = false;
        ecoInjCutActive = false;
        crankStage = CRANK_PRIME;
        crankStageTime = 0;
        stoppedToAcc = false; // Reset flag on crank timeout
      }
    }
    break;

  case STATE_RUNNING:
    // Relays: ACC ON, IGN ON, START OFF (Engine running)
    setRelays(true, true, false);

    // Track continuous standstill duration (spd == 0 with brake held)
    if (spd == 0 && brakeHeld)
    {
      if (standstillStartTime == 0)
      {
        standstillStartTime = now;
      }
    }
    else
    {
      standstillStartTime = 0;
    }

    // Auto Start-Stop evaluation with aggressive wear-protection gates
    {
      bool autoStopPermitted = !autoStartStopDisabled &&
                               !engineStartDisabled &&
                               (lastEngineStartTime != 0) &&
                               (now - lastEngineStartTime >= AUTO_STOP_COOLDOWN_MS) &&
                               (temp_out >= AUTO_STOP_MIN_TEMP_C) &&
                               (temp_out <= AUTO_STOP_MAX_TEMP_C) &&
                               (voltage_filtered >= AUTO_STOP_MIN_VOLTAGE) &&
                               (now - lastPacketTime < FRONT_MCU_TIMEOUT_MS);

      if (autoStopPermitted && standstillStartTime != 0 &&
          (now - standstillStartTime >= AUTO_STOP_STANDSTILL_DELAY_MS))
      {
        currentState = STATE_AUTO_STOP;
        autoStopStartTime = now;
        ecoInjCutActive = true; // Signal Front MCU over CAN 0x03 to cut injectors
        standstillStartTime = 0;
        break;
      }
    }

    // Handle Engine Stall Safety
    // Only treat rpm==0 as stall if CAN packets are still being received
    // (prevents ignition cut on CAN bus failure at highway speed)
    if (currentRpm == 0 && (now - lastPacketTime < 2000))
    {
      currentState = STATE_ACC;
      standbyStartTime = now;
      stoppedToAcc = false;
      standstillStartTime = 0;
      isEcoRestart = false;
      ecoInjCutActive = false;
    }

    // Handle Engine Stop Button Press (Only if vehicle is stationary)
    if (btnPressed && (now - lastButtonPressTime >= BUTTON_COOLDOWN_MS))
    {
      if (spd == 0)
      { // Safety check: speed must be zero
        lastButtonPressTime = now;
        standstillStartTime = 0;
        isEcoRestart = false;
        ecoInjCutActive = false;
        bool brakeHeld = (digitalRead(PIN_INPUT_BRAKE) == LOW);
        if (brakeHeld)
        {
          setRelays(true, false, false); // Keep ACC ON, kill IGN and START
          currentState = STATE_ACC;      // Go to ACC position
          stoppedToAcc = true;           // Mark that we just stopped the engine to ACC
        }
        else
        {
          setRelays(false, false, false); // Kill ACC, IGN, and START
          currentState = STATE_STANDBY;   // Go to Standby (OFF)
          stoppedToAcc = false;           // Reset flag on stop to standby
        }
        standbyStartTime = now; // Start sleep timeout timer
      }
    }
    break;

  case STATE_AUTO_STOP:
  {
    // Relays: ACC ON, IGN ON, START OFF (ECU awake, fuel cut via Front MCU CAN)
    setRelays(true, true, false);

    // Keep eco injector cut active over CAN
    ecoInjCutActive = true;

    // Check manual button press: Driver shuts down car completely
    if (btnPressed && (now - lastButtonPressTime >= BUTTON_COOLDOWN_MS))
    {
      lastButtonPressTime = now;
      ecoInjCutActive = false;
      isEcoRestart = false;
      setRelays(false, false, false); // Turn off all relays
      currentState = STATE_STANDBY;
      standbyStartTime = now;
      stoppedToAcc = false;
      break;
    }

    // Restart triggers:
    // 1. Primary restart trigger: Driver releases foot from brake
    bool brakeReleased = (digitalRead(PIN_INPUT_BRAKE) == HIGH);

    // 2. Safety restart triggers:
    // - Battery drops below restart threshold (11.6V)
    // - Max auto-stop duration exceeded (90s)
    // - Engine coolant temp creeping high (> 95°C)
    // - Front MCU communication loss
    bool maxDurationExceeded = (now - autoStopStartTime >= AUTO_STOP_MAX_DURATION_MS);
    bool batteryLow = (voltage_filtered < AUTO_STOP_RESTART_VOLTAGE);
    bool tempCreep = (temp_out > AUTO_STOP_MAX_TEMP_C);
    bool canLoss = (now - lastPacketTime > FRONT_MCU_TIMEOUT_MS);

    if (brakeReleased || maxDurationExceeded || batteryLow || tempCreep || canLoss)
    {
      // Restore injectors immediately over CAN
      ecoInjCutActive = false;
      isEcoRestart = true;
      currentState = STATE_CRANKING;
      crankStage = CRANK_PRIME;
      crankStageTime = 0;
    }
    break;
  }
  }
}
