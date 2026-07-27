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

// W202 C200 instrument cluster speed pulse (195/65R15, 48 ABS teeth, falling edge)
#define PULSES_PER_KM        24080UL
#define SPD_WINDOW_MS        100UL
#define SPD_STALE_TIMEOUT_US 500000UL

const int tempPin = A0;
const int fan = 5;
const int ac = A1;
#define inj_sense_pin 7
uint8_t oil_level = 0;
unsigned long last_check = 0;

// Variables shared with ISR MUST be volatile
volatile uint32_t lastTime = 0;
volatile uint32_t period = 0;
volatile uint16_t spd_pulse_count = 0;
volatile uint32_t spd_last_pulse_us = 0;
volatile uint32_t total_inj_time_us = 0;
volatile uint32_t inj_start_micros = 0;
volatile uint32_t last_inj_pulse_width = 0;
volatile bool inj_active = false;
volatile bool inj_disable_pending = false;
volatile bool injDisable = false;

// These are only used in loop(), so they do NOT need to be volatile
uint32_t rpm = 0;
uint32_t last_rpm = 0;
uint32_t spd = 0;
uint16_t spd_s = 0;

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
  spd_last_pulse_us = micros();
}

ISR(PCINT2_vect)
{
  uint32_t now = micros();
  bool pin_state = digitalReadFast(inj_sense_pin);
  if (pin_state == LOW)
  {
    if (!inj_active)
    {
      inj_start_micros = now;
      inj_active = true;
    }
  }
  else
  {
    if (inj_active)
    {
      last_inj_pulse_width = now - inj_start_micros;
      total_inj_time_us += last_inj_pulse_width;
      inj_active = false;

      // Safely apply disable right as Cyl 1 finishes, before next cylinder fires
      if (inj_disable_pending)
      {
        injDisable = true;
        digitalWriteFast(inj_pin, HIGH);
        inj_disable_pending = false;
      }
    }
  }
}

//=================== CAN Diagnostics ==================//
void checkCanErrors()
{
  uint8_t errFlags = mcp2515.getErrorFlags();
  if (errFlags != 0)
  {
    if (errFlags & (MCP2515::EFLG_RX0OVR | MCP2515::EFLG_RX1OVR))
    {
      mcp2515.clearRXnOVR();
    }
    // Only completely reset the chip if it goes into Bus-Off (fatal state).
    // Do NOT interfere if it's just in Error Passive (TXEP/RXEP); it will self-recover.
    if (errFlags & MCP2515::EFLG_TXBO)
    {
      mcp2515.reset();
      mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
      mcp2515.setNormalOneShotMode();
    }
  }
}

void setup()
{
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
  wdt_disable();
  wdt_enable(WDTO_2S);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(rpm_pin), rpmISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(spd_pin), spdISR, FALLING);

  pinModeFast(inj_sense_pin, INPUT_PULLUP);
  PCICR |= (1 << PCIE2);    // Enable PCINT2 group (Port D)
  PCMSK2 |= (1 << PCINT23); // Mask to ONLY pin 7 (PCINT23) — any future Port D
                            // pins used with PCINT MUST also be added here,
                            // otherwise unintended ISR(PCINT2_vect) calls will occur.

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalOneShotMode();
}

void loop()
{
  unsigned long currentMillis = millis();
  unsigned long currentMicros = micros();

  //=================== Read Sensors & Control Fan (Every 500ms) ======================//
  if (currentMillis - lastSensorTime >= 500)
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
    int dutyCycle_temp = map((int)temp_avg, 690, 730, 20, 255);
    dutyCycle_temp = constrain(dutyCycle_temp, 0, 255);

    float acState_t = analogRead(ac);
    static bool ac_initialized = false;
    if (!ac_initialized)
    {
      acState_avg = acState_t;
      ac_initialized = true;
    }
    else
      acState_avg = acState_avg + (acState_t - acState_avg) * 0.125f; // 1/8 = 0.125
    int dutyCycle_ac = map((int)acState_avg, 50, 500, 20, 255);
    dutyCycle_ac = constrain(dutyCycle_ac, 0, 255);

    int dutyCycle = max(dutyCycle_temp, dutyCycle_ac);
    analogWrite(fan, dutyCycle);

    oil_level = digitalReadFast(oil_level_pin);

    lastSensorTime = currentMillis;
  }

  //=================== Calculate RPM (Every loop) ======================//
  noInterrupts();
  uint32_t p = period;
  interrupts();
  if (p > 0)
  {
    uint32_t calc = 60000000UL / p;
    rpm = calc / 2; // Only update with valid readings  // Real engine RPM (2 pulses per rev)
  }

  //=================== Control Injectors =====================//
  int th_Pos = digitalReadFast(th_pin);
  static unsigned long last_inj_check = 0;

  if (rpm > 1500 && th_Pos == 1 && temp_avg > 440)
  {
    if (last_inj_check == 0)
      last_inj_check = currentMillis; // Start 1000ms timer
    if (currentMillis - last_inj_check >= 1000)
    {
      if (!injDisable)
      {
        inj_disable_pending = true;
      }
    }
  }
  else
  {
    last_inj_check = 0; // Reset timer if condition no longer met
  }

  // Fallback: If inj_disable_pending is true but no pulse arrives for > 1 engine cycle, disable safely
  // if (inj_disable_pending && !inj_active)
  // {
  //   noInterrupts();
  //   uint32_t p = period;
  //   uint32_t start = inj_start_micros;
  //   interrupts();
  //   if (p > 0 && (currentMicros - start > p))
  //   {
  //     injDisable = true;
  //     inj_disable_pending = false;
  //     digitalWriteFast(inj_pin, HIGH);
  //   }
  // }

  // Deactivation is instant when throttle is released or RPM drops below hysteresis limit
  if (th_Pos == 0 || rpm < 1100)
  {
    inj_disable_pending = false;
    injDisable = false;
    digitalWriteFast(inj_pin, LOW);
  }
  else if (injDisable)
  {
    digitalWriteFast(inj_pin, HIGH);
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
    spd_s = (spd > 220) ? 220 : (uint16_t)spd;

    lastSpdCalculationTime = currentMillis;
  }

  static int alive = 0;
  // Process CAN and auto-recover errors
  checkCanErrors();
  if (mcp2515.readMessage(&canMsgRx) == MCP2515::ERROR_OK)
  {
    if (canMsgRx.can_id == 0x03)
    {
      alive = canMsgRx.data[0];
      last_check = currentMillis;
    }
  }

  // Failsafe timeout: if no message in 1000ms, assume dead
  if (currentMillis - last_check > 1000)
  {
    alive = 0;
  }

  uint8_t injDisable_s = (uint8_t)injDisable;
  uint16_t rpm_s = (uint16_t)(rpm);
  //================= Send to Display MCU (Every 50ms) ===============//
  if (currentMillis - lastCanSendTime >= 50)
  {
    uint16_t temp_s = (uint16_t)temp_avg;
    canMsgTx.can_id = 0x02;
    canMsgTx.can_dlc = 8;
    canMsgTx.data[0] = temp_s & 0xFF; // lowByte(temp_s);
    canMsgTx.data[1] = temp_s >> 8;   // highByte(temp_s);
    canMsgTx.data[2] = spd_s & 0xFF;
    canMsgTx.data[3] = spd_s >> 8;
    canMsgTx.data[4] = injDisable_s;
    canMsgTx.data[5] = rpm_s & 0xFF;
    canMsgTx.data[6] = rpm_s >> 8;
    canMsgTx.data[7] = oil_level;
    mcp2515.sendMessage(&canMsgTx);

    // Atomically read and reset total_inj_time_us
    noInterrupts();
    uint32_t local_inj_time = total_inj_time_us;
    total_inj_time_us = 0;
    interrupts();

    canMsgTx.can_id = 0x04;
    canMsgTx.can_dlc = 4;
    canMsgTx.data[0] = local_inj_time & 0xFF;
    canMsgTx.data[1] = (local_inj_time >> 8) & 0xFF;
    canMsgTx.data[2] = (local_inj_time >> 16) & 0xFF;
    canMsgTx.data[3] = (local_inj_time >> 24) & 0xFF;
    mcp2515.sendMessage(&canMsgTx);

    lastCanSendTime = currentMillis;
  }

  //================= Regulator Failsafe ===============//
  if (alive != 100)
  {
    digitalWriteFast(regulator_pin, LOW);
  }
  else
  {
    digitalWriteFast(regulator_pin, HIGH);
  }
  wdt_reset();
}
