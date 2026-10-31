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
#define SPD_MIN_EDGE_US 300UL         // Edge debounce: 220 km/h = ~660 us between pulses
#define SPD_STOP_TIMEOUT_US 1000000UL // No pulse for 1 s => 0 km/h (1 pulse/s = 0.15 km/h)
#define RPM_STALE_TIMEOUT_US 1000000UL

const int tempPin = A0;
const int fan = 5;
const int ac = A1;
const int map_pin = A3;
#define inj_sense_pin 7

// --- MAP / Vacuum Sensor (Toyota 89420-02010 / 89420-02020) ---
#define MAP_EMA_ALPHA 0.25f       // 0.25 EMA filter for smooth signal with ~60ms settling
#define MAP_CALIB_ATM_DEFAULT 780 // ~3.8V at sea level atmospheric pressure (~101.3 kPa)
#define MAP_ADC_TO_PSI 0.02035f   // Conversion factor from ADC counts to PSI vacuum
#define MAP_READ_INTERVAL_MS 20
#define MAP_FAULT_MIN_ADC 10   // Broken wire / ground short
#define MAP_FAULT_MAX_ADC 1015 // 5V short / disconnected pull-up

float map_adc_avg = (float)MAP_CALIB_ATM_DEFAULT;
uint16_t map_atm_adc = MAP_CALIB_ATM_DEFAULT;
float vac_psi = 0.0f;
uint16_t vac_psi_x100 = 0;     // Gauge vacuum in 0.01 PSI (0..1500 => 0.00..15.00 PSI)
uint16_t baro_psi_x100 = 1470; // Barometric baseline in 0.01 PSI (default 14.70 PSI at sea level)
bool baro_calibrated = false;  // True once atmospheric baseline was sampled with engine stopped (RPM == 0)
bool map_sensor_fault = false;
unsigned long lastMapReadTime = 0;

// --- DFCO Configuration ---
#define DFCO_ENGAGE_RPM 1500
#define DFCO_DISENGAGE_RPM 1000
#define DFCO_ENGAGE_DELAY_MS 1000
#define DFCO_ENGINE_WARM_ADC 636   // Engine warm (>60°C with 2.3kΩ pull-down)
#define DFCO_INJ_WINDOW_TICKS 8000 // 8000 ticks @ 0.5us/tick = 4000us (safe window before next cylinder fires)
#define MAX_INJ_ACTIVE_MS 30       // 30ms max pulse timeout to prevent telemetry lockup

// --- Failsafe ---
#define HEARTBEAT_TIMEOUT_MS 1000
#define REGULATOR_FAIL_THRESHOLD 3

// --- Fan Control (Calibrated for 2.3kΩ pull-down: 90°C = 841 ADC, 93°C = 856 ADC, 96°C = 871 ADC, 100°C = 891 ADC) ---
#define FAN_TEMP_MIN_ADC 856 // Turn on at 93°C (gentle ~18% speed)
#define FAN_TEMP_HYST_ADC 15 // Turn off below 841 (90°C, 3°C hysteresis to prevent short-cycling)
#define FAN_TEMP_MAX_ADC 891 // Full 100% fan speed at 100°C
#define FAN_AC_MIN_ADC 50
#define FAN_AC_MAX_ADC 500
#define FAN_DUTY_MIN 45 // ~18% PWM to reliably start Toyota fan module without stall/hum
#define FAN_DUTY_MAX 255

// --- Speed ---
#define MAX_SPEED_KMH 220

// --- Oil Level Filter (W202 S11 Switch: LOW = OK, HIGH = Low Oil with 47k pull-up) ---
#define OIL_LOW_PERSIST_MS 30000UL // 30s sustained low oil to trigger warning (rejects slosh)
#define OIL_OK_RECOVERY_MS 10000UL // 10s continuous OK to clear warning
#define OIL_CHECK_MIN_RPM 500      // Engine running qualification threshold

// --- CAN ---
#define CAN_FAST_SEND_INTERVAL_MS 20  // 0x02 (Engine/Speed) & 0x05 (Vacuum/MAP) at 50 Hz
#define CAN_TELEM_SEND_INTERVAL_MS 50 // 0x04: Cumulative injector & speed counters (20 Hz)
#define CAN_ENV_SEND_INTERVAL_MS 100  // 0x06: Thermal, Oil, AC & Fan duty (10 Hz)
#define CAN_TX_STAGGER_MS 4           // Minimum 4ms spacing between CAN TX frames so SPI/TXB0 never bursts
#define CAN_SENSOR_READ_INTERVAL_MS 500

uint8_t oil_level = 0;
unsigned long last_check = 0;
uint8_t regulator_fail_count = 0;

// Variables shared with ISR MUST be volatile
volatile uint32_t lastTime = 0;
volatile uint32_t period = 0;
volatile uint32_t spd_last_edge_us = 0; // micros() timestamp of the latest speed pulse edge
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
uint8_t fan_duty = 0;

float temp_avg = 0.0f;    // Float for EMA temp
float acState_avg = 0.0f; // Float for EMA AC

// Timers
unsigned long lastSpdCalculationTime = 0;
unsigned long lastCan02SendTime = 0;
unsigned long lastCan04SendTime = 0;
unsigned long lastCan05SendTime = 0;
unsigned long lastCan06SendTime = 0;
unsigned long lastCanAnyTxTime = 0;
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
  uint32_t t = micros();
  if (t - spd_last_edge_us < SPD_MIN_EDGE_US)
    return; // Reject noise edges (faster than any physically possible pulse rate)
  spd_last_edge_us = t;
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
  pinMode(map_pin, INPUT);
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

  //=================== Read MAP Sensor & Calculate Vacuum (Every 20ms) ===================//
  static bool map_first_read_done = false;
  static bool map_initialized = false;
  if (!map_first_read_done || currentMillis - lastMapReadTime >= MAP_READ_INTERVAL_MS)
  {
    map_first_read_done = true;
    float map_raw = (float)analogRead(map_pin);
    if (map_raw < MAP_FAULT_MIN_ADC || map_raw > MAP_FAULT_MAX_ADC)
    {
      map_sensor_fault = true;
    }
    else
    {
      map_sensor_fault = false;
      if (!map_initialized)
      {
        map_adc_avg = map_raw;
        map_initialized = true;
      }
      else
      {
        map_adc_avg += (map_raw - map_adc_avg) * MAP_EMA_ALPHA;
      }

      // Atmospheric baseline: when engine is stopped (RPM == 0), manifold pressure is barometric
      if (rpm == 0 && map_adc_avg >= 500.0f && map_adc_avg <= 950.0f)
      {
        map_atm_adc = (uint16_t)(map_adc_avg + 0.5f);
        baro_calibrated = true;
      }
    }

    // Calculate barometric PSI using the same sensor slope as vacuum (MAP_ADC_TO_PSI),
    // referenced to the nominal 780 ADC = 14.7 PSI calibration point. (The sensor has an
    // output offset, so scaling proportionally through 0 ADC would mis-scale baro changes.)
    float baro_psi_f = 14.7f + ((float)map_atm_adc - (float)MAP_CALIB_ATM_DEFAULT) * MAP_ADC_TO_PSI;
    if (baro_psi_f > 16.5f)
      baro_psi_f = 16.5f;
    if (baro_psi_f < 8.0f)
      baro_psi_f = 8.0f;
    baro_psi_x100 = (uint16_t)(baro_psi_f * 100.0f + 0.5f);

    // Gauge vacuum below atmospheric: delta from barometric reference (0.00 PSI when stopped)
    if (rpm == 0)
    {
      vac_psi = 0.0f;
      vac_psi_x100 = 0;
    }
    else
    {
      float vac_counts = (float)map_atm_adc - map_adc_avg;
      if (vac_counts < 0.0f)
        vac_counts = 0.0f;
      vac_psi = vac_counts * MAP_ADC_TO_PSI;
      if (vac_psi > 15.0f)
        vac_psi = 15.0f;
      vac_psi_x100 = (uint16_t)(vac_psi * 100.0f + 0.5f);
    }

    lastMapReadTime = currentMillis;
  }

  //=================== Read Sensors & Control Fan (Every 500ms) ======================//
  static bool sensor_first_read_done = false;
  if (!sensor_first_read_done || currentMillis - lastSensorTime >= CAN_SENSOR_READ_INTERVAL_MS)
  {
    sensor_first_read_done = true;
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
      dutyCycle_temp = map((int)temp_avg, FAN_TEMP_MIN_ADC, FAN_TEMP_MAX_ADC, FAN_DUTY_MIN, FAN_DUTY_MAX);
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
    fan_duty = (uint8_t)dutyCycle;
    analogWrite(fan, fan_duty);

    // --- OEM-Style Oil Level switch (Mercedes W202 / S11) ---
    // Hardware: 47k pull-up to D6 -> Sensor -> GND.
    // Tested state: LOW = Oil OK, HIGH = Low Oil.
    // Damping: Requires 30s sustained LOW oil reading (HIGH) while engine is running (RPM >= 500)
    // to reject cornering/braking slosh. Requires 10s continuous OK (LOW) to clear warning.
    uint8_t raw_oil = digitalReadFast(oil_level_pin);
    static uint32_t oil_low_accum_ms = 0;
    static uint32_t oil_ok_accum_ms = 0;

    if (rpm >= OIL_CHECK_MIN_RPM)
    {
      if (raw_oil == HIGH) // Sensor reports low oil
      {
        oil_ok_accum_ms = 0;
        if (oil_low_accum_ms < OIL_LOW_PERSIST_MS)
        {
          oil_low_accum_ms += CAN_SENSOR_READ_INTERVAL_MS;
        }
        if (oil_low_accum_ms >= OIL_LOW_PERSIST_MS)
        {
          oil_level = 1; // Assert low oil warning after 30s sustained condition
        }
      }
      else // Sensor reports oil level OK
      {
        if (oil_low_accum_ms >= CAN_SENSOR_READ_INTERVAL_MS)
        {
          oil_low_accum_ms -= CAN_SENSOR_READ_INTERVAL_MS;
        }
        else
        {
          oil_low_accum_ms = 0;
        }

        if (oil_ok_accum_ms < OIL_OK_RECOVERY_MS)
        {
          oil_ok_accum_ms += CAN_SENSOR_READ_INTERVAL_MS;
        }
        if (oil_ok_accum_ms >= OIL_OK_RECOVERY_MS)
        {
          oil_level = 0; // Clear warning after 10s continuous OK
          oil_low_accum_ms = 0;
        }
      }
    }
    else
    {
      // Engine stopped or cranking: pause/decay low oil accumulation to prevent false alerts when parked
      if (oil_low_accum_ms >= CAN_SENSOR_READ_INTERVAL_MS)
      {
        oil_low_accum_ms -= CAN_SENSOR_READ_INTERVAL_MS;
      }
      else
      {
        oil_low_accum_ms = 0;
      }
      oil_ok_accum_ms = 0;
    }

    lastSensorTime = currentMillis;
  }

  //=================== Calculate RPM (Every loop) ======================//
  noInterrupts();
  uint32_t now_us = micros();
  uint32_t p = period;
  uint32_t last_rpm_edge = lastTime;
  interrupts();

  if (now_us - last_rpm_edge > RPM_STALE_TIMEOUT_US || p == 0)
  {
    rpm = 0;
  }
  else if (p >= 2500UL)
  {
    rpm = 30000000UL / p; // 60,000,000 / (p * 2 pulses/rev)
  }
  // If p < 2500UL (noise glitch > 12,000 RPM), retain previous valid rpm reading

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
        noInterrupts();
        if (!inj_active)
        { // Re-verify no pulse started since snapshot
          digitalWriteFast(inj_pin, HIGH);
          injDisable = true;
          inj_just_ended = false;
        }
        interrupts();
      }
    }
  }
  else
  {
    last_inj_check = 0; // Reset timer if condition no longer met
  }

  // Deactivation is instant when throttle is opened or RPM drops below hysteresis limit
  // Require confirmation before disengaging on low RPM to ignore transient tachometer glitches
  static uint8_t low_rpm_confirm_count = 0;
  bool rpm_disengage = false;
  if (rpm < DFCO_DISENGAGE_RPM)
  {
    if (++low_rpm_confirm_count >= 3)
    {
      rpm_disengage = true;
    }
  }
  else
  {
    low_rpm_confirm_count = 0;
  }

  if ((th_Pos == 0 || rpm_disengage) && injDisable)
  {
    injDisable = false;
    low_rpm_confirm_count = 0;
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
        noInterrupts();
        if (!inj_active)
        { // Re-verify no pulse started since snapshot
          digitalWriteFast(inj_pin, HIGH);
          eco_inj_cut_active = true;
          inj_just_ended = false;
        }
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
  // Edge-timed measurement: speed = pulses since last update / exact time between the
  // reference edge and the newest edge. Resolution is limited only by micros() (4 us),
  // instead of 1 pulse per 100 ms window (= 1.46 km/h steps with simple counting).
  if (currentMillis - lastSpdCalculationTime >= SPD_WINDOW_MS)
  {
    static uint32_t spd_ref_pulses = 0;
    static uint32_t spd_ref_edge_us = 0;
    static bool spd_ref_valid = false;

    noInterrupts();
    uint32_t pulses_now = total_spd_pulses;
    uint32_t edge_us = spd_last_edge_us;
    uint32_t spd_now_us = micros();
    interrupts();

    uint32_t new_pulses = pulses_now - spd_ref_pulses;
    if (!spd_ref_valid)
    {
      // Wait for a first edge to use as timing reference
      spd = 0;
      if (new_pulses > 0)
      {
        spd_ref_pulses = pulses_now;
        spd_ref_edge_us = edge_us;
        spd_ref_valid = true;
      }
    }
    else if (new_pulses > 0)
    {
      uint32_t dt_us = edge_us - spd_ref_edge_us;
      if (dt_us > 0)
      {
        // km/h = (pulses / PULSES_PER_KM) km / (dt_us / 3.6e9) h
        float kmh = ((float)new_pulses * 3.6e9f) / ((float)PULSES_PER_KM * (float)dt_us);
        if (kmh <= (float)MAX_SPEED_KMH * 1.5f) // Ignore physically impossible readings (noise)
        {
          spd = (uint32_t)(kmh + 0.5f);
        }
      }
      spd_ref_pulses = pulses_now;
      spd_ref_edge_us = edge_us;
    }
    else
    {
      // No new edge in this window
      uint32_t since_edge_us = spd_now_us - spd_ref_edge_us;
      if (since_edge_us > SPD_STOP_TIMEOUT_US)
      {
        spd = 0;
        spd_ref_valid = false;
      }
      else
      {
        // Real speed is at most 1 pulse per since_edge_us: lets the reading fall smoothly to 0 when stopping
        float kmh_max = 3.6e9f / ((float)PULSES_PER_KM * (float)since_edge_us);
        if ((float)spd > kmh_max)
          spd = (uint32_t)(kmh_max + 0.5f);
      }
    }
    spd_s = (spd > MAX_SPEED_KMH) ? MAX_SPEED_KMH : (uint16_t)spd;

    lastSpdCalculationTime = currentMillis;
  }

  static int alive = 0;
  // Process CAN and auto-recover errors
  for (uint8_t i = 0; i < 10 && mcp2515.readMessage(&canMsgRx) == MCP2515::ERROR_OK; i++)
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

  //================= Send to cabin MCU (ESP32): 0x02 (20ms), 0x05 (20ms), 0x04 (50ms), 0x06 (100ms), Staggered ===============//
  static bool first_06_sent = false;
  if (currentMillis - lastCan02SendTime >= CAN_FAST_SEND_INTERVAL_MS)
  {
    static uint8_t seq_02 = 0;

    uint16_t rpm_s = (uint16_t)rpm;
    uint8_t th_state = (uint8_t)digitalReadFast(th_pin); // 1 = closed (idle), 0 = open

    // CAN ID 0x02: Fast Engine & Motion Telemetry (50 Hz, DLC 8)
    canMsgTx.can_id = 0x02;
    canMsgTx.can_dlc = 8;
    canMsgTx.data[0] = rpm_s & 0xFF;               // RPM low byte
    canMsgTx.data[1] = (rpm_s >> 8) & 0xFF;        // RPM high byte
    canMsgTx.data[2] = spd_s & 0xFF;               // Vehicle speed low byte (km/h)
    canMsgTx.data[3] = (spd_s >> 8) & 0xFF;        // Vehicle speed high byte (km/h)
    canMsgTx.data[4] = injDisable ? 1 : 0;         // DFCO fuel cut active (0 or 1)
    canMsgTx.data[5] = eco_inj_cut_active ? 1 : 0; // Auto Start-Stop injector cut active (0 or 1)
    canMsgTx.data[6] = th_state ? 1 : 0;           // Throttle idle switch (1 = closed/idle, 0 = open)
    canMsgTx.data[7] = seq_02++;
    mcp2515.sendMessage(&canMsgTx);

    lastCan02SendTime = currentMillis;
    lastCanAnyTxTime = currentMillis;
  }
  else if ((currentMillis - lastCan05SendTime >= CAN_FAST_SEND_INTERVAL_MS) &&
           (currentMillis - lastCanAnyTxTime >= CAN_TX_STAGGER_MS))
  {
    static uint8_t seq_05 = 0;

    uint16_t baro_out = (!map_sensor_fault && baro_calibrated) ? baro_psi_x100 : 0;
    uint16_t map_adc_s = (uint16_t)(map_adc_avg + 0.5f);

    // CAN ID 0x05: Manifold Vacuum & Barometric Telemetry (50 Hz, DLC 8)
    canMsgTx.can_id = 0x05;
    canMsgTx.can_dlc = 8;
    canMsgTx.data[0] = vac_psi_x100 & 0xFF;        // Manifold vacuum low byte (0.01 PSI)
    canMsgTx.data[1] = (vac_psi_x100 >> 8) & 0xFF; // Manifold vacuum high byte (0.01 PSI)
    canMsgTx.data[2] = baro_out & 0xFF;            // Barometric baseline low byte (0.01 PSI, 0 if uncalibrated)
    canMsgTx.data[3] = (baro_out >> 8) & 0xFF;     // Barometric baseline high byte (0.01 PSI)
    canMsgTx.data[4] = map_adc_s & 0xFF;           // Filtered MAP ADC low byte (0..1023)
    canMsgTx.data[5] = (map_adc_s >> 8) & 0xFF;    // Filtered MAP ADC high byte (0..1023)
    canMsgTx.data[6] = map_sensor_fault ? 1 : 0;   // MAP sensor fault (0 = OK, 1 = Fault)
    canMsgTx.data[7] = seq_05++;
    mcp2515.sendMessage(&canMsgTx);

    lastCan05SendTime = currentMillis;
    lastCanAnyTxTime = currentMillis;
  }
  else if ((currentMillis - lastCan04SendTime >= CAN_TELEM_SEND_INTERVAL_MS) &&
           (currentMillis - lastCanAnyTxTime >= CAN_TX_STAGGER_MS))
  {
    // Atomic snapshot of cumulative counters (staggered >= 4ms from other frames so MCP2515 TXB0 is
    // guaranteed idle and SPI transactions never run back-to-back in the same loop pass).
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

    lastCan04SendTime = currentMillis;
    lastCanAnyTxTime = currentMillis;
  }
  else if ((!first_06_sent || (currentMillis - lastCan06SendTime >= CAN_ENV_SEND_INTERVAL_MS)) &&
           (currentMillis - lastCanAnyTxTime >= CAN_TX_STAGGER_MS))
  {
    static uint8_t seq_06 = 0;

    uint16_t temp_s = (uint16_t)(temp_avg + 0.5f);
    uint8_t ac_state = (acState_avg >= FAN_AC_MIN_ADC) ? 1 : 0;

    // CAN ID 0x06: Thermal & Auxiliary Status (10 Hz, DLC 8) — Fully unpacked
    canMsgTx.can_id = 0x06;
    canMsgTx.can_dlc = 8;
    canMsgTx.data[0] = temp_s & 0xFF;        // Raw Temp ADC low byte
    canMsgTx.data[1] = (temp_s >> 8) & 0xFF; // Raw Temp ADC high byte
    canMsgTx.data[2] = oil_level ? 1 : 0;    // Filtered oil level warning (0 = OK, 1 = Low)
    canMsgTx.data[3] = ac_state;             // AC request state (0 = OFF, 1 = ON)
    canMsgTx.data[4] = fan_duty;             // Radiator fan PWM duty (0..255)
    canMsgTx.data[5] = 0;                    // Reserved
    canMsgTx.data[6] = 0;                    // Reserved
    canMsgTx.data[7] = seq_06++;
    mcp2515.sendMessage(&canMsgTx);

    first_06_sent = true;
    lastCan06SendTime = currentMillis;
    lastCanAnyTxTime = currentMillis;
  }
  // Serial.println(map_adc_avg);
  wdt_reset();
}
