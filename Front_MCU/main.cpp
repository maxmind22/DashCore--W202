#include <digitalWriteFast.h>
#include <avr/wdt.h>
#include <mcp2515.h>
#include <SPI.h>

#define oil_level_pin 6
#define rpm_pin 2
#define spd_pin 3
#define th_pin 4
#define inj_pin 8
#define regulator_pin 9

// W202 C200 instrument cluster speed pulse (205/55R16, 48 ABS teeth, falling edge)
#define PULSES_PER_KM 24714UL
#define SPD_WINDOW_MS 100UL
#define RPM_STALE_TIMEOUT_US 400000UL
#define RPM_MIN_PERIOD_US 2500UL // Glitch filter: max ~12,000 RPM (2 pulses/rev)


const int tempPin = A0;
const int fan = 5;
const int ac = A1;
#define inj_sense_pin 7

// --- DFCO Configuration ---
#define DFCO_ENGAGE_RPM 1500
#define DFCO_DISENGAGE_RPM 1000
#define DFCO_ENGAGE_DELAY_MS 1000
#define DFCO_ENGINE_WARM_ADC 440
#define DFCO_INJ_WINDOW_TICKS 8000 // 8000 ticks @ 0.5us/tick = 4000us (safe window before next cylinder fires)
#define MAX_INJ_ACTIVE_MS 30       // 30ms max pulse timeout to prevent telemetry lockup

// --- Failsafe ---
#define HEARTBEAT_TIMEOUT_MS 1000
#define REGULATOR_FAIL_THRESHOLD 3

// --- Fan Control ---
#define FAN_TEMP_MIN_ADC 690
#define FAN_TEMP_HYST_ADC 15   // Turn off at 675 ADC to prevent cycling
#define FAN_TEMP_MAX_ADC 730
#define FAN_AC_MIN_ADC 50
#define FAN_AC_MAX_ADC 500
#define FAN_DUTY_MIN 20
#define FAN_DUTY_MAX 255

// --- Speed ---
#define MAX_SPEED_KMH 220

// --- CAN ---
#define CAN_SEND_INTERVAL_MS 50
#define CAN_SENSOR_READ_INTERVAL_MS 500

uint8_t oil_level = 0;
unsigned long last_check = 0;
uint8_t regulator_fail_count = 0;

// Variables shared with ISR MUST be volatile
volatile uint32_t lastTime = 0;
volatile uint32_t period = 0;
volatile uint16_t spd_pulse_count = 0;
volatile uint32_t total_spd_pulses = 0;
volatile uint32_t total_inj_time_us = 0;

volatile uint16_t total_inj_pulses = 0;
volatile uint16_t inj_start_ticks = 0;
volatile uint16_t inj_end_ticks = 0;
volatile bool inj_just_ended = false;
volatile bool inj_active = false;

// State variables (loop only)
uint32_t rpm = 0;
uint32_t spd = 0;
uint16_t spd_s = 0;
bool injDisable = false;
bool eco_inj_cut_cmd = false;
bool eco_inj_cut_active = false;
bool fan_active = false;

float temp_avg = 0.0f;    // Float for EMA temp
float acState_avg = 0.0f; // Float for EMA AC

// Timers
unsigned long lastSpdCalculationTime = 0;
unsigned long lastCanSendTime = 0;
unsigned long lastSensorTime = 0;

struct can_frame canMsgRx;
struct can_frame canMsgTx;
MCP2515 mcp2515(10, 8000000);

//=============Interrupt Service Routine ===============//
void rpmISR()
{
  uint32_t now = micros();
  uint32_t diff = now - lastTime;
  if (diff >= RPM_MIN_PERIOD_US) // Glitch / noise filter
  {
    period = diff;
    lastTime = now;
  }
}

void spdISR()
{
  spd_pulse_count++;
  total_spd_pulses++;
}


// injector ISR
ISR(PCINT2_vect)
{
  uint16_t inj_now_ticks = TCNT1;
  if (!(PIND & _BV(PD7))) // Injector firing (active low)
  {
    if (!inj_active)
    {
      inj_start_ticks = inj_now_ticks;
      inj_active = true;
    }
  }
  else // Injector closed
  {
    if (inj_active)
    {
      uint16_t pulse_ticks = inj_now_ticks - inj_start_ticks;
      if (pulse_ticks >= 400) // Glitch filter: ignore inductive spikes < 200us (400 * 0.5us)
      {
        uint32_t pulse_width = pulse_ticks >> 1; // Convert 0.5us ticks (Prescaler 8) to us
        total_inj_time_us += pulse_width;
        total_inj_pulses++;
      }
      inj_active = false;
      inj_end_ticks = inj_now_ticks;
      inj_just_ended = true;
    }
  }
}

//=================== CAN Diagnostics ==================//
void checkCanErrors(unsigned long now)
{
  uint8_t errFlags = mcp2515.getErrorFlags();
  if (errFlags != 0)
  {
    // Clear overflow flags in EFLG without wiping CANINTF unread interrupt flags
    if (errFlags & (MCP2515::EFLG_RX0OVR | MCP2515::EFLG_RX1OVR))
    {
      mcp2515.clearRXnOVRFlags();
    }
    // Only reset the chip on fatal Bus-Off (TXBO) with 1000ms cooldown to avoid reset thrashing / CPU starvation
    static unsigned long lastResetAttempt = 0;
    if ((errFlags & MCP2515::EFLG_TXBO) && (now - lastResetAttempt >= 1000))
    {
      mcp2515.reset();
      mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
      mcp2515.setNormalOneShotMode();
      lastResetAttempt = now;
    }
  }
}

// Disable watchdog timer immediately after reset (before C runtime init) to prevent bootloops on ATmega328P
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));
void wdt_init(void)
{
  MCUSR = 0;
  wdt_disable();
}

void setup()
{
  MCUSR = 0;
  wdt_disable();

  pinMode(fan, OUTPUT);
  pinMode(tempPin, INPUT);
  pinModeFast(rpm_pin, INPUT);
  pinModeFast(spd_pin, INPUT);
  pinModeFast(th_pin, INPUT);
  pinModeFast(inj_pin, OUTPUT);
  digitalWriteFast(inj_pin, LOW); // Normal state: injectors connected
  pinMode(ac, INPUT);
  pinModeFast(oil_level_pin, INPUT);
  pinModeFast(regulator_pin, OUTPUT);
  digitalWriteFast(regulator_pin, LOW); // Safe state on boot
  Serial.begin(115200);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(rpm_pin), rpmISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(spd_pin), spdISR, FALLING);

  pinModeFast(inj_sense_pin, INPUT);
  PCICR |= (1 << PCIE2); // Enable PCINT2 group (Port D)
  PCMSK2 = _BV(PCINT23);

  // Initialize Timer1 for high-precision fuel injection pulse timing (0.5us per tick @ 16MHz)
  TCCR1A = 0;
  TCCR1B = _BV(CS11); // Normal mode, Prescaler 8

  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalOneShotMode();

  wdt_enable(WDTO_2S);
}

void loop()
{
  unsigned long currentMillis = millis();
  unsigned long currentMicros = micros();

  // Watchdog check for stuck injector active state (evaluated in loop without ISR overhead)
  static unsigned long inj_active_start_ms = 0;
  if (inj_active)
  {
    if (inj_active_start_ms == 0)
      inj_active_start_ms = currentMillis;
    else if (currentMillis - inj_active_start_ms > MAX_INJ_ACTIVE_MS)
    {
      inj_active = false;
      inj_active_start_ms = 0;
    }
  }
  else
  {
    inj_active_start_ms = 0;
  }

  //=================== Read Sensors & Control Fan (Every 500ms) ======================//
  if (currentMillis - lastSensorTime >= CAN_SENSOR_READ_INTERVAL_MS)
  {
    float temp_t = analogRead(tempPin);
    static bool temp_initialized = false;
    if (!temp_initialized)
    {
      temp_avg = temp_t;
      temp_initialized = true;
    }
    else
    {
      temp_avg += (temp_t - temp_avg) * 0.0625f;
    }

    // Fan temperature control with hysteresis
    int dutyCycle_temp = 0;
    if (fan_active)
    {
      if (temp_avg < (FAN_TEMP_MIN_ADC - FAN_TEMP_HYST_ADC))
        fan_active = false;
    }
    else
    {
      if (temp_avg >= FAN_TEMP_MIN_ADC)
        fan_active = true;
    }

    if (fan_active)
    {
      dutyCycle_temp = map((int)temp_avg, FAN_TEMP_MIN_ADC - FAN_TEMP_HYST_ADC, FAN_TEMP_MAX_ADC, FAN_DUTY_MIN, FAN_DUTY_MAX);
      dutyCycle_temp = constrain(dutyCycle_temp, FAN_DUTY_MIN, FAN_DUTY_MAX);
    }


    // AC fan control
    float acState_t = analogRead(ac);
    static bool ac_initialized = false;
    if (!ac_initialized)
    {
      acState_avg = acState_t;
      ac_initialized = true;
    }
    else
    {
      acState_avg += (acState_t - acState_avg) * 0.125f;
    }

    int dutyCycle_ac = 0;
    if (acState_avg >= FAN_AC_MIN_ADC)
    {
      dutyCycle_ac = map((int)acState_avg, FAN_AC_MIN_ADC, FAN_AC_MAX_ADC, FAN_DUTY_MIN, FAN_DUTY_MAX);
      dutyCycle_ac = constrain(dutyCycle_ac, FAN_DUTY_MIN, FAN_DUTY_MAX);
    }

    int dutyCycle = max(dutyCycle_temp, dutyCycle_ac);
    analogWrite(fan, dutyCycle);

    oil_level = digitalReadFast(oil_level_pin);

    lastSensorTime = currentMillis;
  }

  //=================== Calculate RPM (Every loop) ======================//
  noInterrupts();
  uint32_t p = period;
  uint32_t last_rpm_edge = lastTime;
  interrupts();

  if (currentMicros - last_rpm_edge > RPM_STALE_TIMEOUT_US || p == 0)
  {
    rpm = 0;
  }
  else
  {
    rpm = 30000000UL / p; // 60,000,000 / (p * 2 pulses/rev)
  }

  //=================== Control Injectors (DFCO) =====================//
  int th_Pos = digitalReadFast(th_pin);
  static unsigned long last_inj_check = 0;

  // Read end ticks and elapsed time atomically
  noInterrupts();
  uint16_t elapsed_ticks = (uint16_t)(TCNT1 - inj_end_ticks);
  bool just_ended = inj_just_ended;
  bool inj_busy = inj_active;
  interrupts();

  if (just_ended && elapsed_ticks >= DFCO_INJ_WINDOW_TICKS)
  {
    noInterrupts();
    inj_just_ended = false; // Safe window expired, prevent 16-bit timer wraparound aliasing
    interrupts();
  }

  if (rpm > DFCO_ENGAGE_RPM && th_Pos == 1 && temp_avg > DFCO_ENGINE_WARM_ADC)
  {
    if (last_inj_check == 0)
      last_inj_check = currentMillis; // Start 1000ms timer
    if (currentMillis - last_inj_check >= DFCO_ENGAGE_DELAY_MS && !injDisable)
    {
      // Safe cut window: right after monitored injector finishes, before the next cylinder fires
      if (!inj_busy && just_ended && elapsed_ticks < DFCO_INJ_WINDOW_TICKS)
      {
        digitalWriteFast(inj_pin, HIGH);
        injDisable = true;
        noInterrupts();
        inj_just_ended = false;
        interrupts();
      }
    }
  }
  else
  {
    last_inj_check = 0; // Reset timer if condition no longer met
  }

  // Deactivation is instant when throttle is released or RPM drops below hysteresis limit
  if ((th_Pos == 0 || rpm < DFCO_DISENGAGE_RPM) && injDisable)
  {
    injDisable = false;
    // Only bring pin LOW if Auto Start-Stop is not actively cutting injectors
    if (!eco_inj_cut_cmd && !eco_inj_cut_active)
    {
      digitalWriteFast(inj_pin, LOW);
    }
  }

  //=================== Auto Start-Stop Safe Injector Cut =====================//
  if (eco_inj_cut_cmd)
  {
    if (!eco_inj_cut_active)
    {
      // Safe cut: if engine is already stopped (rpm == 0) or in the safe inter-injector window
      if (!inj_busy && (rpm == 0 || (just_ended && elapsed_ticks < DFCO_INJ_WINDOW_TICKS)))
      {
        digitalWriteFast(inj_pin, HIGH);
        eco_inj_cut_active = true;
        noInterrupts();
        inj_just_ended = false;
        interrupts();
      }
    }
  }
  else
  {
    if (eco_inj_cut_active)
    {
      eco_inj_cut_active = false;
      if (!injDisable)
      {
        digitalWriteFast(inj_pin, LOW);
      }
    }
  }

  //==================== Calculate Vehicle Speed (Every 100ms) ====================//
  if (currentMillis - lastSpdCalculationTime >= SPD_WINDOW_MS)
  {
    noInterrupts();
    uint16_t count = spd_pulse_count;
    spd_pulse_count = 0;
    interrupts();

    if (count > 0)
    {
      // Rounded km/h calculation: (count * 3600000 + half_divisor) / divisor
      spd = ((uint32_t)count * 3600000UL + 1235700UL) / (PULSES_PER_KM * SPD_WINDOW_MS);
    }
    else
    {
      spd = 0;
    }
    spd_s = (spd > MAX_SPEED_KMH) ? MAX_SPEED_KMH : (uint16_t)spd;

    lastSpdCalculationTime = currentMillis;
  }

  static int alive = 0;
  // Process CAN and auto-recover errors
  while (mcp2515.readMessage(&canMsgRx) == MCP2515::ERROR_OK)
  {
    if (canMsgRx.can_id == 0x03)
    {
      alive = canMsgRx.data[0];
      eco_inj_cut_cmd = (canMsgRx.data[1] & 0x01) != 0;
      last_check = currentMillis;

      // Regulator failsafe hysteresis
      if (alive == 100)
      {
        regulator_fail_count = 0;
        digitalWriteFast(regulator_pin, HIGH);
      }
      else
      {
        if (regulator_fail_count < REGULATOR_FAIL_THRESHOLD)
        {
          regulator_fail_count++;
        }
        if (regulator_fail_count >= REGULATOR_FAIL_THRESHOLD)
        {
          digitalWriteFast(regulator_pin, LOW);
        }
      }
    }
  }
  checkCanErrors(currentMillis);

  // Failsafe timeout: if no message in 1000ms, assume dead
  if (currentMillis - last_check > HEARTBEAT_TIMEOUT_MS)
  {
    alive = 0;
    eco_inj_cut_cmd = false;              // Safe state: clear fuel cut so engine can run
    digitalWriteFast(regulator_pin, LOW); // De-energize field disconnect relay
  }

  //================= Send to Display MCU (Every 50ms) ===============//
  if (currentMillis - lastCanSendTime >= CAN_SEND_INTERVAL_MS)
  {
    static uint8_t seq_02 = 0;

    uint8_t injDisable_s = (uint8_t)(injDisable || eco_inj_cut_active);
    uint16_t rpm_s = (uint16_t)rpm;
    uint16_t temp_s = (uint16_t)temp_avg;

    // CAN ID 0x02: Instantaneous Status (DLC 8)
    canMsgTx.can_id = 0x02;
    canMsgTx.can_dlc = 8;
    canMsgTx.data[0] = temp_s & 0xFF;
    canMsgTx.data[1] = temp_s >> 8;
    canMsgTx.data[2] = spd_s & 0xFF;
    canMsgTx.data[3] = spd_s >> 8;
    canMsgTx.data[4] = rpm_s & 0xFF;
    canMsgTx.data[5] = rpm_s >> 8;
    canMsgTx.data[6] = (injDisable_s & 0x01) | ((oil_level & 0x01) << 1);
    canMsgTx.data[7] = seq_02++;
    mcp2515.sendMessage(&canMsgTx);

    // Atomic snapshot of cumulative counters.
    // Only update injector snapshot when injector is not mid-fire (!inj_active)
    // to guarantee inj_time and inj_pulses reflect only complete pulses.
    // If mid-fire, re-send last snapshot (Display MCU computes delta=0 and
    // defers the fuel to the next 50ms packet — seamless with cumulative counters).
    static uint32_t inj_time_snap = 0;
    static uint16_t inj_pulses_snap = 0;
    uint16_t spd_pulses_snap;
    noInterrupts();
    if (!inj_active)
    {
      inj_time_snap = total_inj_time_us;
      inj_pulses_snap = total_inj_pulses;
    }
    spd_pulses_snap = (uint16_t)total_spd_pulses;
    interrupts();

    // CAN ID 0x04: Cumulative Injector & Speed Telemetry (DLC 8)
    canMsgTx.can_id = 0x04;
    canMsgTx.can_dlc = 8;
    canMsgTx.data[0] = inj_time_snap & 0xFF;
    canMsgTx.data[1] = (inj_time_snap >> 8) & 0xFF;
    canMsgTx.data[2] = (inj_time_snap >> 16) & 0xFF;
    canMsgTx.data[3] = (inj_time_snap >> 24) & 0xFF;
    canMsgTx.data[4] = inj_pulses_snap & 0xFF;
    canMsgTx.data[5] = (inj_pulses_snap >> 8) & 0xFF;
    canMsgTx.data[6] = spd_pulses_snap & 0xFF;
    canMsgTx.data[7] = (spd_pulses_snap >> 8) & 0xFF;
    mcp2515.sendMessage(&canMsgTx);

    lastCanSendTime = currentMillis;
  }

  wdt_reset();
}
