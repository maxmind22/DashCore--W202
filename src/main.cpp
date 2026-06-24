// fuel low: 1326,  full: 21900 (vals)  volts: low: 0.17, full: 2.73
//============= variables =============//

#include <ESP_8_BIT_GFX.h>
#include <SPI.h>
#include <WiFi.h>
#include <digitalWriteFast.h>
#include <esp_task_wdt.h>
#include <math.h>
#include <mcp2515.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <esp_bt.h>
#include <driver/dac.h>
#include <soc/i2s_struct.h>
#include <driver/rtc_io.h>
#include <esp_sleep.h>

// --- PUSH START PIN DEFINITIONS ---
#define PIN_RELAY_ACC 16   // Terminal 15R (Accessory)
#define PIN_RELAY_IGN 26   // Terminal 15 (POS2/Ignition)
#define PIN_RELAY_START 13 // Terminal 50 (Starter)
#define PIN_BTN_START 33   // Push Button (Active Low, Pull-Up, RTC-capable)
#define PIN_INPUT_BRAKE 36 // Brake Light Sensor (Active High, Opto-isolated, Input-only)
#define PIN_WAKE_UNLOCK 35 // Unlock Pulse Input (Active Low, Opto-isolated, RTC Input-only)

// --- SYSTEM STATES ---
enum SystemState
{
  STATE_SLEEP,
  STATE_STANDBY,
  STATE_ACC,      // POS1 (ACC ON, IGN OFF)
  STATE_IGNITION, // POS2 (ACC ON, IGN ON)
  STATE_CRANKING,
  STATE_RUNNING
};

RTC_DATA_ATTR SystemState currentState = STATE_SLEEP;
unsigned long standbyStartTime = 0;
unsigned long lastButtonPressTime = 0;
bool stoppedToAcc = false;
volatile bool regulatorTaskRunning = true;
unsigned long ignitionEntryTime = 0;

const unsigned long STANDBY_TIMEOUT_MS = 60000;     // 1 Minutes (Production sleep timeout)
const unsigned long ACCESSORY_TIMEOUT_MS = 7200000; // 2 Hours (7200000 ms)
const unsigned long BUTTON_COOLDOWN_MS = 1000;      // 3 Seconds button lockout
const unsigned long MAX_CRANK_TIME_MS = 5000;       // 5 Seconds limit

ESP_8_BIT_GFX tv(true, 8);

#define coolant_level_pin 34
#define FIELD_PIN 12
#define field_relay_pin 14

#define FUEL_X 5
#define FUEL_Y 100
#define FUEL_HEIGHT 100
#define FUEL_WIDTH 15
#define buzzer_pin 4

#define TEMP_X 240
#define TEMP_Y 100
#define TEMP_HEIGHT 100
#define TEMP_TICKS_WIDTH 5
#define TEMP_TICKS_HEIGHT 2
#define TEMP_VALUE_TICK_WIDTH 11
#define TEMP_VALUE_TICK_HEIGHT 6

#define GAUGE_CX 128
#define GAUGE_CY 165
#define GAUGE_R 85 // 75
#define WARNING_X 50
#define WARNING_Y 20

unsigned long lastTime = 0;
unsigned long last_spd_correction = 0;
unsigned long lastSpeedTime = 0;
const int LOW_FUEL_LEVEL = 10;
bool lowBlinkState = false;
bool lowBlinkState2 = false;
unsigned long lastBlinkTime = 0;
unsigned long lastBlinkTime2 = 0;
int fuel_in_temporary = 0;
int filtered = 0;
const int smoother = 3000;
const int blinkInterval = 400; // ms
const int blinkInterval2 = 120;
float smoothVal = 0;
int lastValue = 0;
int raw = 0;
int goodSamples = 0;
int goodSamples2 = 0;
int badSamples = 0;
int badSamples2 = 0;
unsigned long last_fuel_correction = 0;
int temp_out = 0;
int percent = 0;

struct CalibrationPoint
{
  int rawValue;
  float percent;
};

const int NUM_CALIBRATION_POINTS = 11;
const CalibrationPoint calibrationTable[NUM_CALIBRATION_POINTS] = {
    {1326, 0.0f},    // Height 0.0:   0.0% volume (empty)
    {3383, 4.9f},    // Height 0.1:   4.9% volume
    {5440, 10.5f},   // Height 0.2:  10.5% volume
    {7498, 16.8f},   // Height 0.3:  16.8% volume
    {9555, 23.9f},   // Height 0.4:  23.9% volume
    {11613, 35.4f},  // Height 0.5:  35.4% volume
    {13670, 50.2f},  // Height 0.6:  50.2% volume (middle bridge)
    {15727, 64.4f},  // Height 0.7:  64.4% volume
    {17785, 78.0f},  // Height 0.8:  78.0% volume
    {19842, 90.5f},  // Height 0.9:  90.5% volume
    {21900, 100.0f}, // Height 1.0: 100.0% volume (full)
};

float getFuelPercent(float rawValue)
{
  if (rawValue <= calibrationTable[0].rawValue)
    return calibrationTable[0].percent;
  if (rawValue >= calibrationTable[NUM_CALIBRATION_POINTS - 1].rawValue)
    return calibrationTable[NUM_CALIBRATION_POINTS - 1].percent;

  for (int i = 0; i < NUM_CALIBRATION_POINTS - 1; i++)
  {
    if (rawValue >= calibrationTable[i].rawValue && rawValue <= calibrationTable[i + 1].rawValue)
    {
      float x0 = calibrationTable[i].rawValue;
      float x1 = calibrationTable[i + 1].rawValue;
      float y0 = calibrationTable[i].percent;
      float y1 = calibrationTable[i + 1].percent;
      return y0 + (rawValue - x0) * (y1 - y0) / (x1 - x0);
    }
  }
  return 0.0f;
}

//---------- display variables------------------//
//-----------fuel
int v = 11;
int last_v = 0;
int fill = 0;
char fuel_color = 0x00;
//------------ temp
int t = 11;
int last_t = 0;
int fill2 = 0;

volatile int spd = 0;
volatile uint16_t spd_t = 0;
uint16_t raw2;
unsigned long lastPacketTime = 0;
uint8_t oil_level_t = 0;
int oil_level = 0;
int last_clear = 0;
//----------CAN bus variables------------------//
struct can_frame canMsg;     // Used by loop() for RX only
struct can_frame canMsgTx;   // Used by regulatorTask for TX only
MCP2515 mcp2515(5, 8000000); // CS pin 5
// SemaphoreHandle_t spiMutex;         // Protects MCP2515 (SPI) access across cores
//------------------- warning variables ------------------//
unsigned int counter = 0;
int last_spd = -1;
bool coolant_level = false;
int buzzer_state = 0;
volatile int spd_l = 0;
bool fuel = false;
bool cool = false;
bool cool_run = true;
bool speed_on = true;
bool oil = false;
bool hot = false;
bool fuel_run = true;
bool oil_on = true;
bool temp_on = true;
bool conn_on = true;
int overspeed_state = 0;
uint8_t injector_state = 0;
bool inj_on = true;
const int over_speed_on = 500;
const int over_speed_off = 170;
volatile uint8_t health_state = 0; // 0 =  charging malfunction, 100 = working fine
int boot_chime = 0;
uint16_t new_rpm = 0;

Adafruit_ADS1115 adc;

//------------------- regulator variables ------------------//
portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;
volatile float voltage_filtered = 13.6f;
volatile float current_A_filtered = 0.0f; // filtered reading used for control
TaskHandle_t regulatorTaskHandle;
volatile int ads_fuel = 0;
volatile int charge_state = 0;
int chg = 0;
int chg2 = 0;
volatile uint32_t last_charge = 0;
volatile uint16_t rpm = 0;
int field_pwm = 0;
uint16_t local_rpm = 0;

void regulatorTask(void *pvParameters)
{
  esp_task_wdt_add(NULL);
  int consecutive_failures = 0;

  const float voltage_alpha = 0.8f;               // 0.3
  const float current_sensor_offset_mv = 2500.0f; // 2519
  const float current_sensor_mV_per_A = 4.0f;     // 4.0f; // mV per Amp (FS500E2T)
  const float current_limit_upper = 20.000f;      // start pulling back above this
  const float current_alpha = 0.2f;               // previously 0.01
  const float base_Kp = 30.000f;
  const float base_Ki = 5.0f;
  const float Kd = 0.0f;

  static float integral_error = 0.0f;
  static unsigned long last_regulator_time = 0;
  static uint32_t last_fuel_time = 0;
  static float last_error = 0.0f;
  const float v_target = 13.6000f; // Max voltage is 13.6V

  for (;;)
  {
    if (!regulatorTaskRunning)
    {
      esp_task_wdt_delete(NULL);
      regulatorTaskHandle = NULL;
      vTaskDelete(NULL); // Delete self safely
    }
    // uint32_t start = micros();

    //-------------------- Voltage measurement
    int voltage_raw = adc.readADC_SingleEnded(0);
    int16_t current_raw_t = adc.readADC_SingleEnded(1);

    // Glitch protection: only update if reading is sane
    esp_task_wdt_reset(); // Reset WDT after potentially blocking I2C reads

    if (voltage_raw > 0 && current_raw_t > 0)
    {
      consecutive_failures = 0;
      // GAIN_ONE is 0.125mV per bit (0.000125V)
      // Pre-calculated voltage multiplier: 0.000125f * 4.33766718718175f
      float voltage = voltage_raw * 0.0005422084f;
      float new_v = voltage_alpha * voltage + (1.0f - voltage_alpha) * voltage_filtered;

      // Pre-calculated current multiplier: 0.000125V * 1000 = 0.125mV per bit
      float current_mv = current_raw_t * 0.125f;
      float current_raw = (current_mv - current_sensor_offset_mv) / current_sensor_mV_per_A;
      float new_c = current_alpha * current_raw + (1.0f - current_alpha) * current_A_filtered;

      portENTER_CRITICAL(&dataMux);
      voltage_filtered = new_v;
      current_A_filtered = new_c;
      portEXIT_CRITICAL(&dataMux);
    }
    else
    {
      consecutive_failures++;
      if (consecutive_failures > 50)
      {
        // I2C Bus Recovery
        Wire.end();
        vTaskDelay(pdMS_TO_TICKS(10)); // Yield CPU properly instead of blocking delay()
        Wire.begin();
        Wire.setClock(100000);
        adc.begin();
        adc.setGain(GAIN_ONE);
        adc.setDataRate(RATE_ADS1115_250SPS);
        consecutive_failures = 0;
      }
    }

    // --- Evaluate charging system state ---
    bool severe_failure = false;
    bool sensor_error = (consecutive_failures > 5);

    // 1. Emergency hard cut on severe overvoltage, overcurrent, or sensor failure
    if (voltage_filtered >= v_target + 0.6f || current_A_filtered > 40.0f || sensor_error)
    {
      digitalWriteFast(field_relay_pin, HIGH);
      severe_failure = true;
    }
    else
      digitalWriteFast(field_relay_pin, LOW);

    // 2. Logical malfunctions (regulator output doesn't match expected response)
    // - High voltage/current despite commanding low field (runaway/short)
    // - Low voltage and no current despite commanding high field (open circuit/broken belt)
    // We use <= 0.5A instead of 0.0A to account for small ADC noise around zero
    portENTER_CRITICAL(&dataMux);
    local_rpm = rpm;
    portEXIT_CRITICAL(&dataMux);

    bool logical_failure = ((voltage_filtered >= v_target + 0.4f || current_A_filtered >= 30.0f) ||
                            (voltage_filtered <= v_target - 0.1f && current_A_filtered <= 0.0f && local_rpm > 100));

    // Consolidate state hierarchy
    int next_charge_state = 0;
    if (severe_failure || logical_failure)
    {
      next_charge_state = 1; // charging malfunction warning
    }
    else if (voltage_filtered < v_target - 1.4) // If voltage is very low while engine is running.
    {
      next_charge_state = 2; // battery low warning
    }
    else
    {
      next_charge_state = 0; // normal
    }

    portENTER_CRITICAL(&dataMux);
    charge_state = next_charge_state;
    if (charge_state == 0)
    {
      last_charge = millis();
    }
    portEXIT_CRITICAL(&dataMux);
    // --- Target settings ---

    // Calculate dt (time elapsed in seconds) to make PID immune to loop delays
    uint32_t current_micros = micros();
    float dt = (current_micros - last_regulator_time) / 1000000.0f;
    if (last_regulator_time == 0 || dt > 0.1f || dt <= 0.0f)
    {
      dt = 0.020f; // Default to 20ms on first run or severe lag (typical loop time)
    }
    last_regulator_time = current_micros;

    // 1. Seamless CC/CV Error
    // We increase PWM until we hit EITHER 13.6V or our 20A limit.
    // The most restrictive target (smallest error) commands the loop seamlessly.
    // bool over_target = false;
    float err_v = (v_target - voltage_filtered) * 100;
    float err_i = (current_limit_upper - current_A_filtered) * 10;
    float err = (err_v < err_i) ? err_v : err_i;

    float p_term = base_Kp * err;

    if (isfinite(err) && isfinite(dt))
    {
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
    if (voltage_filtered >= v_target + 0.2f)
    {
      float ov_err = ((v_target + 0.2f) - voltage_filtered) * 100;
      safety_pwm = safety_pwm + (int)(200.0f * ov_err);
    }
    // Extra pullback if overcurrent occurs (Safety backup for PID)
    if (current_A_filtered >= current_limit_upper + 10.0f)
    {
      float oi_err = ((current_limit_upper + 10.0f) - current_A_filtered) * 10;
      safety_pwm = safety_pwm + (int)(200.0f * oi_err); // Mild push-back
    }

    // Force field coil off if engine is not running or sensor error occurs
    if (local_rpm == 0 || sensor_error)
    {
      field_pwm = 0;
      integral_error = 0.0f; // Reset integrator
    }
    else
    {
      field_pwm = constrain(safety_pwm, 0, 1023);
    }

    ledcWrite(0, field_pwm);

    // --- Auxiliary Sensors (Not used for regulator control, but read here to avoid I2C contention) ---
    if (current_micros - last_fuel_time >= 100000)
    {
      int new_fuel = adc.readADC_SingleEnded(2);
      portENTER_CRITICAL(&dataMux);
      ads_fuel = new_fuel;
      portEXIT_CRITICAL(&dataMux);
      last_fuel_time = current_micros;
    }

    // --- Stack monitoring ---
    // if (current_micros - last_stack_check >= 1000000) // Every 1 second
    // {
    //   free_stack = uxTaskGetStackHighWaterMark(NULL);
    //   last_stack_check = current_micros;
    // }

    // task_duration = micros() - start;
    // task_duration = micros() - start;
    vTaskDelay(pdMS_TO_TICKS(20)); // Use vTaskDelay instead of vTaskDelayUntil to always yield
    esp_task_wdt_reset();
  }
}

//================== draw static values once ==============//
void drawStaticGauge()
{
  tv.drawRect(FUEL_X, FUEL_Y, FUEL_WIDTH, FUEL_HEIGHT, 0xFF);
  // Draw the continuous arc from -20 to 200 degrees
  for (float a = -20.0; a <= 200.0; a += 0.5)
  {
    float angle = a * PI / 180.0;
    int px = GAUGE_CX + GAUGE_R * cos(angle);
    int py = GAUGE_CY - GAUGE_R * sin(angle);
    tv.drawPixel(px, py, 0xFF);
  }

  // Draw the circular gauge ticks (0 to 220 km/h, 220 degree sweep) and numbers
  for (int i = 0; i <= 220; i += 20)
  {
    float angle = (200.0 - i) * PI / 180.0;
    int x1 = GAUGE_CX + (GAUGE_R - 8) * cos(angle);
    int y1 = GAUGE_CY - (GAUGE_R - 8) * sin(angle);
    int x2 = GAUGE_CX + GAUGE_R * cos(angle);
    int y2 = GAUGE_CY - GAUGE_R * sin(angle);
    tv.drawLine(x1, y1, x2, y2, 0xFF);

    // Draw numbers just inside the gauge
    int tr = GAUGE_R - 16;
    int tx = GAUGE_CX + tr * cos(angle);
    int ty = GAUGE_CY - tr * sin(angle);
    int tw = (i < 10) ? 6 : ((i < 100) ? 12 : 18);
    tv.setCursor(tx - tw / 2, ty - 4);
    tv.print(i);
  }
  tv.fillCircle(GAUGE_CX, GAUGE_CY, 10, 0xFF);
}
void warnings(int percent, int temp_out, int spd, int coolant_level,
              int oil_level, unsigned long now)
{
  buzzer_state = 0;
  static int priority = 0;
  //------------------------------------------
  if (percent <= LOW_FUEL_LEVEL && lowBlinkState && fuel_run)
  {
    // -------- LOW FUEL warning --------//

    tv.setCursor(FUEL_X, FUEL_Y + FUEL_HEIGHT + 3);
    tv.setTextColor(0xFF);
    tv.setTextSize(2);
    tv.print("LOW");
    tv.setTextSize(1);
    fuel_run = false;
    fuel = true;
  }
  else if (!lowBlinkState && fuel)
  {
    // tv.fillRect(FUEL_X + 20, FUEL_Y - 15 + FUEL_HEIGHT, 36, 16, 0x00);
    tv.fillRect(FUEL_X, FUEL_Y + FUEL_HEIGHT + 3, 36, 16, 0x00);
    fuel = 0;
  }
  //------------------------------------------
  if (!coolant_level && lowBlinkState && priority == 0)
  {
    if (cool_run)
    {
      tv.setCursor(WARNING_X + 40, WARNING_Y);
      tv.setTextColor(0xFF);
      tv.print("COOLANT LOW");
      cool_run = false;
      cool = true;
    }
    buzzer_state = 1;
  }
  else if (!lowBlinkState && cool == true)
  {
    tv.fillRect(WARNING_X + 40, WARNING_Y, 66, 8, 0x00);
    cool = false;
  }

  //--------------------------------------------
  if (overspeed_state == 0 &&
      spd >= 58)
  { // -------- over speed warning --------//
    overspeed_state = 1;
    counter = 0;
  }

  if (overspeed_state == 1)
  {
    if (speed_on)
    {
      tv.setCursor(WARNING_X + 30, WARNING_Y + 10);
      tv.setTextColor(0xFF);
      tv.print("OVER SPEED !");
      speed_on = false;
    }
    priority = 1;

    if (lowBlinkState2)
    {
      buzzer_state = 1;
    }
    if (counter > 3)
    {
      overspeed_state = 2;
    }
  }
  else if (overspeed_state == 2 || overspeed_state == 0)
  {
    if (!speed_on)
    {
      tv.fillRect(WARNING_X + 30, WARNING_Y + 10, 72, 8, 0x00);
      speed_on = true;
      priority = 0;
    }
    if (spd < 58)
    {
      overspeed_state = 0;
    }
  }

  //------------------------------------------------
  if (oil_level > 0 && lowBlinkState && priority == 0)
  {
    if (oil_on)
    {
      tv.setCursor(WARNING_X + 30, WARNING_Y + 18);
      tv.setTextColor(0xFF);
      tv.print("LOW ENGINE OIL");
      oil = true;
      oil_on = false;
    }
    buzzer_state = 1;
  }
  else if (!lowBlinkState && oil == true)
  {
    tv.fillRect(WARNING_X + 30, WARNING_Y + 18, 84, 8, 0x00);
    oil = false;
  }
  //------------------------------------------------
  if (temp_out >= 96 && lowBlinkState && priority == 0)
  {
    if (temp_on)
    {
      tv.setCursor(WARNING_X + 30, WARNING_Y + 26);
      tv.setTextColor(0xFF);
      tv.print("ENGINE OVERHEAT");
      hot = true;
      temp_on = false;
    }

    buzzer_state = 1;
  }
  else if (!lowBlinkState && hot == true)
  {
    tv.fillRect(WARNING_X + 30, WARNING_Y + 26, 90, 8,
                0x00); // clear old warning
    hot = false;
  }
  if (now - lastPacketTime > 5000 &&
      conn_on)
  { // -----connection check--------------
    tv.setCursor(WARNING_X, WARNING_Y + 34);
    tv.setTextColor(0xFF);
    tv.print("Front MCU Diconnected");
    conn_on = false;
  }
  else if (!(now - lastPacketTime > 5000) && conn_on == false)
  {
    tv.fillRect(WARNING_X, WARNING_Y + 34, 126, 8, 0x00);
    conn_on = true;
  }
  if (injector_state == 1 && inj_on == true)
  {
    tv.fillCircle(10, 25, 5, 0x1C);
    inj_on = false;
  }
  else if (inj_on == false && injector_state == 0)
  {
    tv.fillCircle(10, 25, 5, 0x00);
    inj_on = true;
  }
  portENTER_CRITICAL(&dataMux);
  int local_charge_state = charge_state;
  uint32_t local_last_charge = last_charge;
  portEXIT_CRITICAL(&dataMux);

  if (local_charge_state == 1 && lowBlinkState && now - local_last_charge > 20000 && priority == 0)
  {
    if (chg == 0)
    {
      tv.setCursor(WARNING_X + 10, WARNING_Y + 10);
      tv.setTextColor(0xFF);
      tv.print("CHARGING SYSTEM FAIL !");
      chg = 1;
    }
    buzzer_state = 1;
  }
  else if (!lowBlinkState && chg == 1)
  {
    tv.fillRect(WARNING_X + 10, WARNING_Y + 10, 140, 8, 0x00);
    chg = 0;
  }
  if (local_charge_state == 2 && lowBlinkState && now - local_last_charge > 10000 && priority == 0 && rpm > 50)
  {
    if (chg2 == 0)
    {
      tv.setCursor(WARNING_X + 50, WARNING_Y + 30);
      tv.setTextColor(0xFF);
      tv.print("BATTERY LOW !");
      chg2 = 1;
    }
    buzzer_state = 1;
  }
  else if (!lowBlinkState && chg2 == 1)
  {
    tv.fillRect(WARNING_X + 50, WARNING_Y + 30, 66, 8, 0x00);
    chg2 = 0;
  }
  //---------- ring boot chime  ---------
  if (now >= 1000 && boot_chime <= 70)
  {
    boot_chime++;
    buzzer_state = 1;
  }
  if (buzzer_state == 1)
  {
    digitalWriteFast(buzzer_pin, HIGH);
  }
  else
  {
    digitalWriteFast(buzzer_pin, LOW);
  }
}

//=================== CAN Diagnostics ==================//
// void checkCanErrors()
// {
//   uint8_t errFlags = mcp2515.getErrorFlags();
//   if (errFlags != 0)
//   {
//     Serial.print("CAN Error Flags: 0x");
//     Serial.println(errFlags, HEX);

//     // Clear overflow flags to resume reception
//     if (errFlags & (MCP2515::EFLG_RX0OVR | MCP2515::EFLG_RX1OVR))
//     {
//       mcp2515.clearRXnOVR();
//     }

//     // Only completely reset the chip if it goes into Bus-Off (fatal state).
//     // Do NOT interfere if it's just in Error Passive (TXEP/RXEP); it will self-recover.
//     // Continuously calling setNormalMode() during Error Passive aborts transmissions
//     // and blocks natural recovery.
//     if (errFlags & MCP2515::EFLG_TXBO)
//     {
//       Serial.println(" - Bus State Critical! Re-initializing...");
//       mcp2515.reset();
//       mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
//       mcp2515.setNormalMode();
//     }
//   }
// }

// --- PUSH START HELPER FUNCTIONS ---
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

void sleepCANController()
{
  mcp2515.setSleepMode();
}

void wakeupCANController()
{
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalOneShotMode();
}

void enterPowerDownSleep()
{
  // --- ORDERED SHUTDOWN: Stop everything safely before deep sleep ---

  // 1. Disarm Watchdog FIRST — the shutdown sequence below may take >1s
  esp_task_wdt_delete(NULL); // Remove loop() task from WDT
  esp_task_wdt_deinit();     // Fully disable WDT hardware

  // 2. Safely stop the regulator FreeRTOS task (running on Core 0, does I2C + SPI)
  if (regulatorTaskHandle != NULL)
  {
    regulatorTaskRunning = false;
    int timeout = 0;
    while (regulatorTaskHandle != NULL && timeout < 100)
    {
      delay(5);
      timeout++;
    }
    if (regulatorTaskHandle != NULL)
    {
      vTaskDelete(regulatorTaskHandle);
      regulatorTaskHandle = NULL;
    }
  }

  // 3. Turn off field coil PWM and detach LEDC
  ledcWrite(0, 0);
  ledcDetachPin(FIELD_PIN);

  // 4. Stop TV display (I2S DMA) — must stop before deep sleep or DMA crash
  stopTVDisplay();
  delay(10); // Let DMA finish any in-flight transfer

  // 5. Put MCP2515 CAN controller to sleep (SPI device)
  sleepCANController();

  // 6. Disconnect I2C bus
  Wire.end();

  // 7. Turn off all relays and float the pins
  setRelays(false, false, false);
  digitalWrite(PIN_RELAY_ACC, LOW);
  digitalWrite(PIN_RELAY_IGN, LOW);
  digitalWrite(PIN_RELAY_START, LOW);
  pinMode(PIN_RELAY_ACC, INPUT);
  pinMode(PIN_RELAY_IGN, INPUT);
  pinMode(PIN_RELAY_START, INPUT);

  // 8. Turn off buzzer
  digitalWrite(buzzer_pin, LOW);

  // 9. Float the field relay pin
  digitalWrite(field_relay_pin, LOW);
  pinMode(field_relay_pin, INPUT);

  // 10. Configure RTC wakeup: Wake on HIGH because unlock pulses to 0V (opto output goes HIGH)
  esp_sleep_enable_ext1_wakeup(1ULL << PIN_WAKE_UNLOCK, ESP_EXT1_WAKEUP_ANY_HIGH);

  // 11. Enter deep sleep — CPU halts here, wakes up via reset
  currentState = STATE_SLEEP;
  esp_deep_sleep_start();
}

void setupPushStartPins()
{
  pinMode(PIN_RELAY_ACC, OUTPUT);
  pinMode(PIN_RELAY_IGN, OUTPUT);
  pinMode(PIN_RELAY_START, OUTPUT);
  setRelays(false, false, false);

  pinMode(PIN_BTN_START, INPUT_PULLUP);
  pinMode(PIN_INPUT_BRAKE, INPUT);
  pinMode(PIN_WAKE_UNLOCK, INPUT);
}

void processPushStart()
{
  unsigned long now = millis();

  // Edge detection for push button
  static bool lastBtnState = HIGH;
  bool currentBtnState = digitalRead(PIN_BTN_START);
  bool btnPressed = (currentBtnState == LOW && lastBtnState == HIGH);
  lastBtnState = currentBtnState;

  bool brakeHeld = (digitalRead(PIN_INPUT_BRAKE) == LOW);
  int currentRpm = rpm;

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
    // Relays: ACC OFF, IGN OFF, START OFF
    setRelays(false, false, false);

    if (btnPressed && (now - lastButtonPressTime >= BUTTON_COOLDOWN_MS))
    {
      lastButtonPressTime = now;
      currentState = STATE_IGNITION; // Go directly to POS2 so brake switch gets power
      ignitionEntryTime = now;       // Capture entry time for immediate brake check
      standbyStartTime = now;        // Reset 2-min timeout
    }
    break;

  case STATE_ACC:
    // Relays: ACC ON, IGN OFF, START OFF (POS1)
    setRelays(true, false, false);

    if (btnPressed && (now - lastButtonPressTime >= BUTTON_COOLDOWN_MS))
    {
      lastButtonPressTime = now;
      // Temporarily turn on IGN to power the brake switch circuit
      setRelays(true, true, false);
      delay(50); // wait 50ms for relay contacts to settle and optocoupler signal to stabilize
      bool brakeIsHeldNow = (digitalRead(PIN_INPUT_BRAKE) == LOW);

      if (brakeIsHeldNow)
      {
        currentState = STATE_CRANKING;
      }
      else
      {
        currentState = STATE_STANDBY; // No brake -> ACC to OFF (STANDBY)
        standbyStartTime = now;       // Reset 2-min timeout
      }
    }
    break;

  case STATE_IGNITION:
    // Relays: ACC ON, IGN ON, START OFF (POS2)
    setRelays(true, true, false);

    // Check if brake is held immediately on entering POS2 (or within first 500ms)
    if (now - ignitionEntryTime < 500)
    {
      if (brakeHeld)
      {
        currentState = STATE_CRANKING;
        break;
      }
    }

    if (btnPressed && (now - lastButtonPressTime >= BUTTON_COOLDOWN_MS))
    {
      lastButtonPressTime = now;
      if (brakeHeld)
      {
        currentState = STATE_CRANKING;
      }
      else
      {
        currentState = STATE_ACC; // 2nd press (without brake) goes to ACC (POS1)
        standbyStartTime = now;   // Reset 2-min timeout
      }
    }
    break;

  case STATE_CRANKING:
    // Non-blocking stage machine for cranking sequence
    static enum { CRANK_PRIME,
                  CRANK_SOLENOID } crankStage = CRANK_PRIME;
    static unsigned long crankStageTime = 0;

    if (crankStage == CRANK_PRIME)
    {
      // Step 1: Go to POS2 (ACC & IGN ON) for fuel pump prime (1000ms)
      setRelays(true, true, false);
      if (crankStageTime == 0)
      {
        crankStageTime = now;
      }
      if (now - crankStageTime >= 1000)
      {
        crankStage = CRANK_SOLENOID;
        crankStageTime = now; // Reset timer for max crank limit
      }
    }
    else if (crankStage == CRANK_SOLENOID)
    {
      // Step 2: Engage starter solenoid (ACC ON, IGN ON, START ON)
      setRelays(true, true, true);

      if (currentRpm > 400)
      {
        // Engine started successfully
        currentState = STATE_RUNNING;
        setRelays(true, true, false); // Disengage starter, keep ACC/IGN on
        crankStage = CRANK_PRIME;     // Reset stages
        crankStageTime = 0;
      }
      else if (now - crankStageTime > MAX_CRANK_TIME_MS)
      {
        // Cranking failed or timed out (5s safety cutoff)
        setRelays(true, false, false); // Disengage starter to ACC for another try (w202 prevent double starting)
        currentState = STATE_ACC;
        standbyStartTime = now; // Reset 2-min timeout
        crankStage = CRANK_PRIME;
        crankStageTime = 0;
      }
    }
    break;

  case STATE_RUNNING:
    // Relays: ACC ON, IGN ON, START OFF (Engine running)
    setRelays(true, true, false);

    // Handle Engine Stall Safety
    if (currentRpm == 0)
    {
      currentState = STATE_IGNITION;
      standbyStartTime = now; // Start 2-minute sleep timeout
    }

    // Handle Engine Stop Button Press (Only if vehicle is stationary)
    if (btnPressed && (now - lastButtonPressTime >= BUTTON_COOLDOWN_MS))
    {
      if (spd == 0)
      { // Safety check: speed must be zero
        lastButtonPressTime = now;
        setRelays(true, false, false); // Keep ACC ON, kill IGN and START
        currentState = STATE_ACC;      // Go to ACC position
        stoppedToAcc = true;           // Mark that we just stopped the engine
        standbyStartTime = now;        // Start 2-minute sleep timeout
      }
    }
    break;
  }
}

void recoverI2CBus(int sdaPin, int sclPin)
{
  pinMode(sdaPin, INPUT_PULLUP);
  pinMode(sclPin, OUTPUT);
  digitalWrite(sclPin, HIGH);
  delay(1);

  // Toggle SCL if SDA is held low by a stuck slave device
  if (digitalRead(sdaPin) == LOW)
  {
    for (int i = 0; i < 9; i++)
    {
      digitalWrite(sclPin, LOW);
      delayMicroseconds(5);
      digitalWrite(sclPin, HIGH);
      delayMicroseconds(5);
      if (digitalRead(sdaPin) == HIGH)
      {
        break; // Device released the bus
      }
    }
  }
}

//=================== setup ===============//
void setup()
{
  setupPushStartPins();

  // Initialize system state to Standby with 2-minute sleep timeout on all boots/resets
  currentState = STATE_STANDBY;
  standbyStartTime = millis();
  regulatorTaskRunning = true;

  recoverI2CBus(21, 22);
  Wire.begin();
  Wire.setClock(100000); // Slower clock for better noise immunity in engine bay
  Wire.setTimeOut(20);   // Abort I2C transaction if it takes > 20ms
  bool adcReady = adc.begin();
  if (!adcReady)
  {
    Serial.println("Failed to initialize ADS1115!");
    // We don't block here because the UI might still be useful,
    // but the regulatorTask will likely fail/crash on ADC reads.
  }
  else
  {
    adc.setGain(GAIN_ONE); // 1x gain for ±4.096V range, adjust if your input exceeds this
    adc.setDataRate(RATE_ADS1115_250SPS);
  }
  WiFi.mode(WIFI_OFF);
  WiFi.disconnect(true);
  btStop();
  tv.begin();
  tv.copyAfterSwap = true;
  //========  track reset reason: debug purpose   ======//
  esp_reset_reason_t reason = esp_reset_reason();
  if (reason != ESP_RST_POWERON && reason != ESP_RST_UNKNOWN && reason != ESP_RST_EXT && reason != ESP_RST_DEEPSLEEP)
  {
    esp_task_wdt_deinit(); // Disable WDT before long delay to prevent reboot loops
    tv.fillScreen(0x00);
    tv.setCursor(10, 50);
    tv.setTextColor(0xFF);
    tv.setTextSize(2);
    tv.print("WARNING: CRASH!");
    tv.setTextSize(1);
    tv.setCursor(10, 80);
    tv.setTextColor(0xFF);
    tv.print("Reset Reason ID: ");
    tv.print(reason);
    tv.setCursor(10, 100);
    if (reason == ESP_RST_PANIC)
      tv.print("- Software Panic (Bug)");
    else if (reason == ESP_RST_INT_WDT)
      tv.print("- Interrupt WDT");
    else if (reason == ESP_RST_TASK_WDT)
      tv.print("- Task WDT (FreeRTOS)");
    else if (reason == ESP_RST_WDT)
      tv.print("- Other Watchdog");
    else if (reason == ESP_RST_BROWNOUT)
      tv.print("- Brownout / Power Drop");
    else
      tv.print("- Other non-power-on reset");

    tv.waitForFrame();
    delay(5000);
  }

  pinModeFast(buzzer_pin, OUTPUT);
  pinModeFast(coolant_level_pin, INPUT);
  pinMode(field_relay_pin, OUTPUT);
  digitalWriteFast(field_relay_pin, LOW); // Start with field relay off
  Serial.begin(250000);
  delay(50); // Let UART stabilize
  Serial.print("\n--- ESP32 BOOTED ---\nReset Reason ID: ");
  Serial.println(reason);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); // Match transmitter
  mcp2515.setNormalOneShotMode();            // mcp2515.setNormalMode();
  esp_task_wdt_deinit();                     // De-init default core WDT config
  esp_task_wdt_init(1, true);                // 1s timeout with panic=true
  esp_task_wdt_add(nullptr);                 // Add current loop task
  ledcSetup(0, 400, 10);                     // channel, freq, resolution
  ledcAttachPin(FIELD_PIN, 0);

  // Configure FreeRTOS task for regulator loop instead of hardware timer
  if (adcReady)
  {
    xTaskCreatePinnedToCore(regulatorTask,   // Task function
                            "RegulatorTask", // Name of task
                            8192,            // Stack size of task
                            nullptr,         // Parameter of the task
                            configMAX_PRIORITIES -
                                2,                // Priority: high but not max, to avoid starving idle task
                            &regulatorTaskHandle, // Task handle
                            0);                   // Pin to Core 0 (Isolate from UI on Core 1)
  }
  lastPacketTime = millis(); // Initialize to avoid immediate timeout warning
}

// ===================== main loop ======================//
void loop()
{
  // uint32_t start = micros();
  unsigned long now = millis();
  health_state = 0; // Assume failure until we complete a successful loop iteration
  if (last_clear < 6)
  {
    tv.fillScreen(0x00);
    drawStaticGauge();
    last_clear++;
  }
  tv.waitForFrame();

  if (now - lastBlinkTime >= blinkInterval)
  {
    lowBlinkState = !lowBlinkState;

    if (lowBlinkState == true)
    {
      fuel_run = true;
      cool_run = true;
      oil_on = true;
      temp_on = true;
    }
    lastBlinkTime = now;
  }
  int over_speed_blink_interval =
      lowBlinkState2 ? over_speed_on : over_speed_off;
  if (now - lastBlinkTime2 >= over_speed_blink_interval)
  {
    lowBlinkState2 = !lowBlinkState2;
    if (lowBlinkState2 == true && counter < 10)
    {
      counter++;
    }
    lastBlinkTime2 = now;
  }

  portENTER_CRITICAL(&dataMux);
  int local_ads_fuel = ads_fuel;
  float local_voltage_filtered = voltage_filtered;
  float local_current_A_filtered = current_A_filtered;
  portEXIT_CRITICAL(&dataMux);

  fuel_in_temporary = local_ads_fuel; // Use pre-fetched value from regulatorTask
  if (fuel_in_temporary < /*2190 max*/ 22000 && fuel_in_temporary > 1200 /*1326 min*/)
  {
    raw = fuel_in_temporary;
  }
  if (lastTime == 0)
  {
    smoothVal = (float)raw;
    filtered = raw;
    lastValue = raw;
  }
  else
  {
    int delta = abs(raw - lastValue);
    if (delta <= 2000)
    { // sample accepted
      filtered = raw;
      goodSamples++;
    }
    else
    {
      filtered = lastValue; // sample rejected, set it to previous good value
      badSamples++;
    }
    if (now - last_fuel_correction >= 15000)
    { // sample error correction
      if (goodSamples < badSamples)
      {
        filtered = raw;
      }
      goodSamples = 0;
      badSamples = 0;
      last_fuel_correction = now;
    }
    lastValue = filtered;
    // tv.setCursor(120, 210);
    // tv.setTextColor(0xff, 0x00);
    // tv.print(field_pwm);
  }
  smoothVal = 0.001 * filtered + (1 - 0.001) * smoothVal; // Exponential moving average for smoothing
  percent = (int)getFuelPercent(smoothVal);
  percent = constrain(percent, 0, 100);

  //-------------------- Coolant level
  coolant_level = digitalReadFast(coolant_level_pin);

  //================================ Read data from engine MCU ==============================================//

  // checkCanErrors();
  MCP2515::ERROR readStatus = mcp2515.readMessage(&canMsg);
  if (readStatus == MCP2515::ERROR_OK)
  {
    if (canMsg.can_id == 0x02)
    {
      raw2 = (uint16_t)((canMsg.data[1] << 8) | canMsg.data[0]);
      spd_t = (uint16_t)(canMsg.data[3] << 8 | canMsg.data[2]);
      oil_level_t = (uint8_t)canMsg.data[6];
      injector_state = canMsg.data[4];
      new_rpm = ((uint16_t)canMsg.data[5] * 100) / 2;
      lastPacketTime = now;
    }
  }
  if (now - lastPacketTime > 2000)
  {
    new_rpm = 0;
  }

  portENTER_CRITICAL(&dataMux);
  rpm = new_rpm;
  portEXIT_CRITICAL(&dataMux);

  //============= send heath signal to front MCU ==================
  health_state = 100;
  static unsigned long lastCanSendTimeMs = 0;
  if (now - lastCanSendTimeMs >= 200)
  {
    canMsgTx.can_id = 0x03;
    canMsgTx.can_dlc = 8;
    canMsgTx.data[0] = health_state;
    canMsgTx.data[1] = 0;
    canMsgTx.data[2] = 0;
    canMsgTx.data[3] = 0;
    canMsgTx.data[4] = 0;
    canMsgTx.data[5] = 0;
    canMsgTx.data[6] = 0;
    canMsgTx.data[7] = 0;
    mcp2515.sendMessage(&canMsgTx);
    lastCanSendTimeMs = now;
  }
  spd_l = map((int)spd_t, 10, 880, 0, 220);
  spd_l = constrain(spd_l, 0, 220);
  int spd_in = (spd_l == 220) ? 0 : spd_l;

  // --------------- filter SPD ----------------
  if (lastTime == 0)
  {
    last_spd = spd_in;
  }
  if (abs(spd_in - last_spd) <= 3)
  { // sample accepted
    spd = spd_in;
    goodSamples2++;
  }
  else
  {
    spd = last_spd; // sample rejected, set it to previous good value
    badSamples2++;
  }
  if (now - last_spd_correction >= 5000)
  { // sample error correction
    if (goodSamples2 < badSamples2)
    {
      spd = spd_in;
    }
    goodSamples2 = 0;
    badSamples2 = 0;
    last_spd_correction = now;
  }

  if (spd != last_spd || lastTime == 0)
  {
    // Erase old needle
    float old_angle = (200.0 - last_spd) * PI / 180.0;
    int old_tip_x = GAUGE_CX + (GAUGE_R - 10) * cos(old_angle);
    int old_tip_y = GAUGE_CY - (GAUGE_R - 10) * sin(old_angle);
    int old_b_x1 = GAUGE_CX + 2 * cos(old_angle - PI / 2.0);
    int old_b_y1 = GAUGE_CY - 2 * sin(old_angle - PI / 2.0);
    int old_b_x2 = GAUGE_CX + 2 * cos(old_angle + PI / 2.0);
    int old_b_y2 = GAUGE_CY - 2 * sin(old_angle + PI / 2.0);
    tv.fillTriangle(old_tip_x, old_tip_y, old_b_x1, old_b_y1, old_b_x2,
                    old_b_y2, 0x00);
    for (int i = 0; i <= 220; i += 20)
    {
      float angle = (200.0 - i) * PI / 180.0;
      // Draw numbers just inside the gauge
      int tr = GAUGE_R - 16;
      int tx = GAUGE_CX + tr * cos(angle);
      int ty = GAUGE_CY - tr * sin(angle);
      int tw = (i < 10) ? 6 : ((i < 100) ? 12 : 18);
      tv.setCursor(tx - tw / 2, ty - 4);
      tv.print(i);
    }
    // Draw new needle
    float new_angle = (200.0 - spd) * PI / 180.0;
    int new_tip_x = GAUGE_CX + (GAUGE_R - 10) * cos(new_angle);
    int new_tip_y = GAUGE_CY - (GAUGE_R - 10) * sin(new_angle);
    int new_b_x1 = GAUGE_CX + 2 * cos(new_angle - PI / 2.0);
    int new_b_y1 = GAUGE_CY - 2 * sin(new_angle - PI / 2.0);
    int new_b_x2 = GAUGE_CX + 2 * cos(new_angle + PI / 2.0);
    int new_b_y2 = GAUGE_CY - 2 * sin(new_angle + PI / 2.0);
    tv.fillTriangle(new_tip_x, new_tip_y, new_b_x1, new_b_y1, new_b_x2,
                    new_b_y2, 0xE0); // Highlight needle
    // Redraw center pin
    tv.fillCircle(GAUGE_CX, GAUGE_CY, 10, 0xFF);

    // tv.setCursor(100, 190);
    // tv.setTextColor(0xFF, 0x00);
    // tv.setTextSize(2);
    // char spdStr[4];
    // snprintf(spdStr, sizeof(spdStr), "%3d", spd);
    // tv.print(spdStr);
    // tv.setTextSize(1);

    last_spd = spd;
  }

  // --- Display voltage ---
  static float voltage_disp = 13.6f;
  float volts = local_voltage_filtered;
  voltage_disp = 0.02f * volts + (1.0f - 0.02f) * voltage_disp;
  tv.setCursor(5, 40);
  tv.setTextColor(0xFF, 0x00);
  char bufV[10];
  snprintf(bufV, sizeof(bufV), "%5.1fV", voltage_disp);
  tv.print(bufV);

  // --- Display current (+ charge / - discharge) ---
  static float current_disp = 0.0f;
  float current_d = local_current_A_filtered;
  current_disp = 0.009f * current_d + (1.0f - 0.009f) * current_disp;
  tv.setCursor(5, 55);
  tv.setTextColor(0xFF, 0x00);
  char bufI[10];
  snprintf(bufI, sizeof(bufI), "%5.0fA", current_disp);
  // tv.fillRect(5, 55, 60, 8, 0x00); // Clear old current value
  tv.print(bufI);

  // display rpm
  tv.setCursor(95, 190);
  tv.setTextColor(0xFF, 0x00);
  tv.setTextSize(2);
  char rpmStr[6];
  snprintf(rpmStr, sizeof(rpmStr), "%5d", (int)rpm);
  tv.print(rpmStr);
  tv.setTextSize(1);

  temp_out = map((int)raw2, 250, 950, 40, 120);
  temp_out = constrain(temp_out, 40, 120);

  // =========================== Send Dynamic data to display
  // ============================
  if (now - lastTime >= 1000)
  {
    fill = map(percent, 0, 100, 0, FUEL_HEIGHT - 4); // ----------for fuel gauge
    fill = constrain(fill, 0, FUEL_HEIGHT - 4);
    v = FUEL_HEIGHT - 2 + FUEL_Y - fill;
    if (percent <= LOW_FUEL_LEVEL)
    {
      fuel_color = 0xE0;
    }
    else
    {
      fuel_color = 0xFF;
    }
    if (last_v != v || lastTime == 0)
    {
      tv.fillRect(FUEL_X + 2, FUEL_Y + 2, FUEL_WIDTH - 4, v - (FUEL_Y + 2),
                  0x00);
      tv.fillRect(FUEL_X + 2, v, FUEL_WIDTH - 4, fill, fuel_color);
      tv.setCursor(FUEL_X, FUEL_Y - 10);
      tv.setTextColor(
          0xFF,
          0x00); // White text on black background to erase old digits cleanly
      char buf[4];
      snprintf(buf, sizeof(buf), "%3d", percent);
      tv.print(buf);
    }

    //--------------------------------------------------------------------------
    fill2 = map(temp_out, 40, 120, 0, TEMP_HEIGHT); // ----------for temp gauge
    int tick_y = 0;
    fill2 = constrain(fill2, 0, TEMP_HEIGHT);
    t = TEMP_HEIGHT + TEMP_Y - fill2;
    if (last_t != t || lastTime == 0)
    {
      // Erase previous needle only
      tv.fillRect(TEMP_X - 5, last_t - 2, TEMP_VALUE_TICK_WIDTH,
                  TEMP_VALUE_TICK_HEIGHT, 0x00);
      tv.drawFastVLine(TEMP_X, TEMP_Y, TEMP_HEIGHT, 0xFF);
      for (int i = 0; i <= 100; i += 25) //---------- draw ticks
      {
        tick_y = TEMP_Y + i;
        tv.fillRect(TEMP_X - 2, tick_y, TEMP_TICKS_WIDTH, TEMP_TICKS_HEIGHT, 0xFF);
      }
      // draw temp_value tick
      tv.fillRect(TEMP_X - 5, t - 2, TEMP_VALUE_TICK_WIDTH,
                  TEMP_VALUE_TICK_HEIGHT, 0xE0);

      // tv.setCursor(TEMP_X - 10, TEMP_Y - 10);
      // tv.setTextColor(0xFF, 0x00); // Erase text automatically
      // char buf[4];
      // snprintf(buf, sizeof(buf), "%3d", temp_out);
      // tv.print(buf);
    }

    lastTime = now;
    last_v = v;
    last_t = t;
  }
  oil_level = (int)oil_level_t;
  warnings(percent, temp_out, spd, coolant_level, oil_level, now);

  // --- Stack monitoring ---
  // static uint32_t last_loop_stack_check = 0;
  // static UBaseType_t loop_free_stack = 0;
  // if (now - last_loop_stack_check >= 1000)
  // {
  //   loop_free_stack = uxTaskGetStackHighWaterMark(NULL);
  //   last_loop_stack_check = now;
  //   // Periodic debug output (once per second, not every frame)
  //   Serial.print("RegStack: ");
  //   Serial.print(free_stack);
  //   Serial.print(" LoopStack: ");
  //   Serial.println(loop_free_stack);
  // }

  // Serial.print("p_term: ");
  // Serial.print(p_term);
  // Serial.print(local_rpm);
  // Serial.print(voltage_filtered);
  // Serial.print("V   current: ");
  // Serial.println(rpm);

  processPushStart();
  esp_task_wdt_reset();
}
