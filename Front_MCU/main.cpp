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
#define PULSES_PER_KM 24714
#define SPD_WINDOW_MS 100UL
#define SPD_STALE_TIMEOUT_US 500000UL
#define RPM_STALE_TIMEOUT_US 1000000UL

const int tempPin = A0;
const int fan = 5;
const int ac = A1;
#define inj_sense_pin 7

// --- DFCO Configuration ---
#define DFCO_ENGAGE_RPM 1500
#define DFCO_DISENGAGE_RPM 1000
#define DFCO_ENGAGE_DELAY_MS 1000
#define DFCO_ENGINE_WARM_ADC 440
#define DFCO_INJ_WINDOW_TICKS 8000 // 8000 ticks @ 0.5us/tick = 4000us

// --- Failsafe ---
#define HEARTBEAT_TIMEOUT_MS 1000
#define REGULATOR_FAIL_THRESHOLD 3

// --- Fan Control ---
#define FAN_TEMP_MIN_ADC 690
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

// Variables shared with ISR MUST be volatile
volatile uint32_t lastTime = 0;
volatile uint32_t period = 0;
volatile uint16_t spd_pulse_count = 0;
volatile uint32_t total_spd_pulses = 0;
volatile uint32_t spd_last_pulse_us = 0;
volatile uint32_t total_inj_time_us = 0;
volatile uint16_t total_inj_pulses = 0;
volatile uint16_t inj_start_ticks = 0;
volatile uint32_t last_inj_pulse_width = 0;
volatile uint16_t inj_end_ticks = 0;
volatile bool inj_active = false;
volatile bool injDisable = false;
volatile bool inj_has_data = false;

// These are only used in loop(), so they do NOT need to be volatile
uint32_t rpm = 0;
uint32_t last_rpm = 0;
uint32_t spd = 0;
uint16_t spd_s = 0;
bool eco_inj_cut_cmd = false;
bool eco_inj_cut_active = false;

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
  period = now - lastTime;
  lastTime = now;
}

void spdISR()
{
  spd_pulse_count++;
  total_spd_pulses++;
  spd_last_pulse_us = micros();
}
// injector ISR
ISR(PCINT2_vect)
{

  uint16_t inj_now_ticks = TCNT1;
  if (!(PIND & _BV(PD7)))
  {
    if (!inj_active)
    {
      inj_start_ticks = inj_now_ticks;
      inj_active = true;
    }
  }
  else
  {
    if (inj_active)
    {
      uint16_t pulse_ticks = inj_now_ticks - inj_start_ticks;
      if (pulse_ticks >= 400) // Glitch filter: ignore inductive spikes < 200us (400 * 0.5us)
      {
        last_inj_pulse_width = pulse_ticks >> 1; // Convert 0.5us ticks (Prescaler 8) to us
        total_inj_time_us += last_inj_pulse_width;
        total_inj_pulses++;
        inj_has_data = true;
      }
      inj_active = false;
      inj_end_ticks = inj_now_ticks;
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
  pinMode(ac, INPUT);
  pinModeFast(oil_level_pin, INPUT);
  pinModeFast(regulator_pin, OUTPUT);
  digitalWrite(regulator_pin, HIGH);
  Serial.begin(115200);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(rpm_pin), rpmISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(spd_pin), spdISR, FALLING);

  pinModeFast(inj_sense_pin, INPUT);
  PCICR |= (1 << PCIE2); // Enable PCINT2 group (Port D)
  PCMSK2 = _BV(PCINT23);
  // PCMSK2 |= (1 << PCINT23); // Mask to ONLY pin 7 (PCINT23) — any future Port D
  // pins used with PCINT MUST also be added here,
  // otherwise unintended ISR(PCINT2_vect) calls will occur.

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

  //=================== Read Sensors & Control Fan (Every 500ms) ======================//
  if (currentMillis - lastSensorTime >= CAN_SENSOR_READ_INTERVAL_MS)
  {
    float temp_t = analogRead(tempPin);
    static bool temp_initialized = false; // Initialize averages on first run or use float-based exponential moving average
    if (!temp_initialized)
    {
      temp_avg = temp_t;
      temp_initialized = true;
    }
    else
      temp_avg = temp_avg + (temp_t - temp_avg) * 0.0625f; // 1/16 = 0.0625
    int dutyCycle_temp = map((int)temp_avg, FAN_TEMP_MIN_ADC, FAN_TEMP_MAX_ADC, FAN_DUTY_MIN, FAN_DUTY_MAX);
    dutyCycle_temp = constrain(dutyCycle_temp, 0, FAN_DUTY_MAX);

    float acState_t = analogRead(ac);
    static bool ac_initialized = false;
    if (!ac_initialized)
    {
      acState_avg = acState_t;
      ac_initialized = true;
    }
    else
      acState_avg = acState_avg + (acState_t - acState_avg) * 0.125f; // 1/8 = 0.125
    int dutyCycle_ac = map((int)acState_avg, FAN_AC_MIN_ADC, FAN_AC_MAX_ADC, FAN_DUTY_MIN, FAN_DUTY_MAX);
    dutyCycle_ac = constrain(dutyCycle_ac, 0, FAN_DUTY_MAX);

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

  if (currentMicros - last_rpm_edge > RPM_STALE_TIMEOUT_US)
  {
    rpm = 0;
  }
  else if (p > 0)
  {
    uint32_t calc = 60000000UL / p;
    rpm = calc / 2; // Only update with valid readings  // Real engine RPM (2 pulses per rev)
  }
  else
  {
    rpm = 0;
  }

  //=================== Control Injectors =====================//
  int th_Pos = digitalReadFast(th_pin);
  static unsigned long last_inj_check = 0;

  if (rpm > DFCO_ENGAGE_RPM && th_Pos == 1 && temp_avg > DFCO_ENGINE_WARM_ADC)
  {
    if (last_inj_check == 0)
      last_inj_check = currentMillis; // Start 1000ms timer
    if (currentMillis - last_inj_check >= DFCO_ENGAGE_DELAY_MS && injDisable == false)
    {
      bool inj_state = false;
      uint16_t inj_end_ticks_t = 0;
      uint16_t tcnt1_snap = 0;
      noInterrupts();
      inj_end_ticks_t = inj_end_ticks; // Capture the value of inj_end_ticks atomically
      inj_state = inj_active;
      tcnt1_snap = TCNT1;
      interrupts();
      uint16_t elapsed_ticks = (uint16_t)(tcnt1_snap - inj_end_ticks_t);
      if (!inj_state && elapsed_ticks < DFCO_INJ_WINDOW_TICKS) // 8000 ticks @ 0.5us/tick = 4000us
      {
        digitalWriteFast(inj_pin, HIGH);
        injDisable = true;
      }
    }
  }
  else
  {
    last_inj_check = 0; // Reset timer if condition no longer met
  }

  // Deactivation is instant when throttle is released or RPM drops below hysteresis limit
  if ((th_Pos == 0 || rpm < DFCO_DISENGAGE_RPM) && injDisable == true)
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
      // Safe window check: never cut mid-injection pulse
      bool inj_state = false;
      uint16_t inj_end_ticks_t = 0;
      uint16_t tcnt1_snap = 0;
      noInterrupts();
      inj_end_ticks_t = inj_end_ticks;
      inj_state = inj_active;
      tcnt1_snap = TCNT1;
      interrupts();
      uint16_t elapsed_ticks = (uint16_t)(tcnt1_snap - inj_end_ticks_t);

      // Only cut when injector is not mid-fire and we are in the inter-pulse window or stopped
      if (!inj_state && (rpm == 0 || elapsed_ticks < DFCO_INJ_WINDOW_TICKS))
      {
        digitalWriteFast(inj_pin, HIGH);
        eco_inj_cut_active = true;
      }
    }
    else
    {
      // Hold pin HIGH while eco cut is commanded
      digitalWriteFast(inj_pin, HIGH);
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
    uint32_t last_pulse = spd_last_pulse_us;
    interrupts();

    if (currentMicros - last_pulse > SPD_STALE_TIMEOUT_US)
    {
      spd = 0;
    }
    else if (count > 0)
    {
      // km/h = pulses * 3600000 / (PULSES_PER_KM * window_ms)
      spd = (uint32_t)count * 3600000UL / (PULSES_PER_KM * SPD_WINDOW_MS);
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
  if (mcp2515.readMessage(&canMsgRx) == MCP2515::ERROR_OK)
  {
    if (canMsgRx.can_id == 0x03)
    {
      alive = canMsgRx.data[0];
      eco_inj_cut_cmd = (canMsgRx.data[1] & 0x01) != 0;
      last_check = currentMillis;
    }
  }
  checkCanErrors(currentMillis);

  // Failsafe timeout: if no message in 1000ms, assume dead
  if (currentMillis - last_check > HEARTBEAT_TIMEOUT_MS)
  {
    alive = 0;
  }
  //================= Send to Display MCU (Every 50ms) ===============//
  if (currentMillis - lastCanSendTime >= CAN_SEND_INTERVAL_MS)
  {
    static uint8_t seq_02 = 0;

    uint8_t injDisable_s = (uint8_t)injDisable;
    uint16_t rpm_s = (uint16_t)(rpm);
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

  //================= Regulator Failsafe ===============//
  if (alive == 100)
  {
    digitalWriteFast(regulator_pin, HIGH);
  }
  else
  {
    digitalWriteFast(regulator_pin, LOW);
  }
  wdt_reset();
}
