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

ESP_8_BIT_GFX tv(true, 8);

// #define fuel_pin 35
#define coolant_level_pin 34
// #define voltage_pin 33
#define FIELD_PIN 12
// #define CURRENT_PIN 32
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
#define GAUGE_R 75
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
int priority = 0;
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
struct can_frame canMsg;
static MCP2515 mcp2515(5); // CS pin 5
//------------------- warning variables ------------------//
unsigned int counter = 0;
int last_spd = 0;
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

Adafruit_ADS1115 adc;

//------------------- regulator variables ------------------//
volatile uint8_t chargeState = 0;
volatile float voltage = 0.0;
volatile float voltage_filtered = 13.6f; // Initialize to target
const float alpha = 0.3;                 // Low-pass filter coefficient — faster response while
                                         // still smooth (was: 0.2)
// LiFePO4 charge profile
const float voltage_target = 13.6f; // BULK/ABSORPTION target (V)
// const float voltage_float_target = 13.6f; // FLOAT reference (informational)
const float current_float_entry = 3.0f; // A — taper threshold to enter FLOAT
// --- FS500E2T Hall Effect Current Sensor ---
// Supply:      5V single-supply (ratiometric)
// Output:      2.5V @ 0A  |  0.5V @ -600A  |  4.5V @ +600A
// Swing:       4000mV over 1200A range  →  4 mV/A
const float current_sensor_offset_mv = 2501.0f; // 2519
const float current_sensor_mV_per_A = 4.0F; // 4.0f; // mV per Amp (FS500E2T)

// Current ceiling thresholds for soft current limiting
const float current_limit_upper = 20.0f; // start pulling back above this

int field_pwm = 300;

volatile float current_A_filtered = 0.0f; // filtered reading used for control
// Heavy filtering to ignore noise spikes
const float current_alpha = 0.01f;        // previously 0.02
TaskHandle_t regulatorTaskHandle;

float integral_error = 0;
unsigned long last_regulator_time = 0;
const float Kp = 50.0;
// Ki slashed to 100.0 to prevent fast ramp-up on noise/glitches
const float Ki = 100.0;
const float Kd = 0.0;    // Kd inactive for now to keep it simple
int voltage_raw = 0;
volatile float ads_fuel_volts = 0.0f;

void regulatorTask(void *pvParameters)
{

  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(8); // 5ms loop time for stability
  xLastWakeTime = xTaskGetTickCount();

  for (;;)
  {
    //-------------------- Voltage measurement
    voltage_raw = adc.readADC_SingleEnded(0);
    int16_t current_raw_t = adc.readADC_SingleEnded(1);
    
    // Glitch protection: only update if reading is sane
    if (voltage_raw > 0 && current_raw_t > 0) {
        voltage = adc.computeVolts(voltage_raw) * 3.565307203471698;
        voltage_filtered = alpha * voltage + (1.0f - alpha) * voltage_filtered;

        float current_mv = adc.computeVolts(current_raw_t) * 1000;
        float current_raw = (current_mv - current_sensor_offset_mv) / current_sensor_mV_per_A;
        current_A_filtered = current_alpha * current_raw + (1.0f - current_alpha) * current_A_filtered;
    }

    // --- Emergency relay: hard cut on severe overvoltage ---
    if (voltage_filtered >= 14.5f)
    {
      digitalWriteFast(field_relay_pin, LOW);
    }
    else if (voltage_filtered < 13.2f)
    {
      digitalWriteFast(field_relay_pin, HIGH);
    }

    // --- Target settings ---
    float v_target = 13.6f; // Max voltage is 13.6V

    // Calculate dt (time elapsed in seconds) to make PID immune to loop delays
    uint32_t current_micros = micros();
    float dt = (current_micros - last_regulator_time) / 1000000.0f;
    if (last_regulator_time == 0 || dt > 0.1f || dt <= 0.0f)
    {
      dt = 0.008f; // Default to 8ms on first run or severe lag
    }
    last_regulator_time = current_micros;

    // 1. Seamless CC/CV Error
    // We increase PWM until we hit EITHER 13.6V or our 20A limit.
    // The most restrictive target (smallest error) commands the loop seamlessly.
    float err_v = v_target - voltage_filtered;
    float err_i = current_limit_upper - current_A_filtered;
    float err = (err_v < err_i) ? err_v : err_i;

    float p_term = Kp * err;
    if ((field_pwm < 1023 && field_pwm > 0) || (field_pwm >= 1023 && err < 0) ||
        (field_pwm <= 0 && err > 0))
    {
      integral_error += err * dt;
    }
    float i_term = Ki * integral_error;
    if (i_term > 1023)
    {
      i_term = 1023;
      integral_error = 1023.0f / Ki;
    }
    else if (i_term < 0)
    {
      i_term = 0;
      integral_error = 0;
    }
    // float d_term = 0; // Kd * (err - last_error);
    int voltage_pwm = constrain((int)(p_term + i_term), 0, 1023);

    // 2. Absolute Hard Voltage Ceiling (Safety Override)
    // We already have CC/CV in the PID above. These overrides are just to
    // provide an extra push if we cross the absolute limits, but they must
    // not be so aggressive that they cause oscillation.
    int safety_pwm = voltage_pwm;
    // Extra pullback if overvoltage occurs (Safety backup for PID)
    if (voltage_filtered >= 13.65f)
    {
      float ov_err = 13.65f - voltage_filtered;
      safety_pwm = safety_pwm + (int)(200.0f * ov_err);
    }
    // Extra pullback if overcurrent occurs (Safety backup for PID)
    if (current_A_filtered >= current_limit_upper + 2.0f)
    {
      float oi_err = (current_limit_upper + 2.0f) - current_A_filtered;
      safety_pwm = safety_pwm + (int)(20.0f * oi_err); // Mild push-back
    }

    int next_pwm = constrain(safety_pwm, 0, 1023);

    field_pwm = next_pwm;

    // last_error = err;

    // --- State Updates (For UI Display Only) ---
    // "FLOAT" is just when it's fully charged and resting at low current
    // if (voltage_filtered >= 13.5f && current_A_filtered < 3.0f)
    // {
    //   chargeState = 1;
    // }
    // // "BULK" is whenever it needs to actively charge
    // else if (voltage_filtered < 13.4f || current_A_filtered > 5.0f)
    // {
    //   chargeState = 0;
    // }

    field_pwm = constrain(field_pwm, 0, 1023);
    ledcWrite(0, field_pwm);

    // --- Auxiliary Sensors (Not used for regulator control, but read here to avoid I2C contention) ---
    ads_fuel_volts = adc.computeVolts(adc.readADC_SingleEnded(2));

    // task_duration = micros() - start;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

//================== draw static values once ==============//
void drawStaticGauge()
{
  tv.drawRect(FUEL_X, FUEL_Y, FUEL_WIDTH, FUEL_HEIGHT, 0xFF);
  // tv.setCursor(TEMP_X - 31, TEMP_Y - 10);
  // tv.setTextColor(0xFF);
  // tv.print("Temp:");

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

    // Draw numbers just outside the gauge
    int tr = GAUGE_R + 8;
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
  // injector_state = true;
  buzzer_state = 0;
  //------------------------------------------
  if (percent <= LOW_FUEL_LEVEL && lowBlinkState && fuel_run)
  {
    // -------- LOW FUEL warning --------//
    // tv.setCursor(FUEL_X + 20, FUEL_Y - 15 + FUEL_HEIGHT);
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
  if (buzzer_state == 1)
  {
    digitalWriteFast(buzzer_pin, HIGH);
  }
  else
  {
    digitalWriteFast(buzzer_pin, LOW);
  }
}

//=================== setup ===============//
void setup()
{
  Wire.begin();
  Wire.setClock(100000); // Slower clock for better noise immunity in engine bay
  if (!adc.begin())
  {
    Serial.println("Failed to initialize ADS1115!");
    // We don't block here because the UI might still be useful,
    // but the regulatorTask will likely fail/crash on ADC reads.
  }
  adc.setGain(GAIN_TWOTHIRDS);
  adc.setDataRate(RATE_ADS1115_860SPS);
  WiFi.mode(WIFI_OFF);
  WiFi.disconnect(true);
  btStop();
  tv.begin();
  tv.copyAfterSwap = true;
  // pinMode(fuel_pin, INPUT);
  pinModeFast(buzzer_pin, OUTPUT);
  pinModeFast(coolant_level_pin, INPUT);
  // pinMode(voltage_pin, INPUT);
  pinMode(field_relay_pin, OUTPUT);
  digitalWriteFast(field_relay_pin, HIGH);
  Serial.begin(250000);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); // Match transmitter
  mcp2515.setNormalMode();
  esp_task_wdt_deinit();      // De-init default core WDT config
  esp_task_wdt_init(3, true); // 3s timeout with panic=true
  esp_task_wdt_add(NULL);     // Add current loop task
  // analogReadResolution(12);
  // analogSetAttenuation(ADC_11db);
  ledcSetup(0, 400, 10); // channel, freq, resolution
  ledcAttachPin(FIELD_PIN, 0);

  // Configure FreeRTOS task for regulator loop instead of hardware timer
  xTaskCreatePinnedToCore(regulatorTask,   // Task function
                          "RegulatorTask", // Name of task
                          4096,            // Stack size of task
                          NULL,            // Parameter of the task
                          configMAX_PRIORITIES -
                              1,                // Priority of the task (Highest)
                          &regulatorTaskHandle, // Task handle
                          0);                   // Pin to Core 0 (Isolate from UI on Core 1)
}

// ===================== main loop ======================//
void loop()
{
  uint32_t start = micros();
  unsigned long now = millis();
  tv.waitForFrame();
  if (last_clear < 6)
  {
    tv.fillScreen(0);
    drawStaticGauge();
    last_clear++;
  }
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

  fuel_in_temporary = (int)(ads_fuel_volts * 1000); // Use pre-fetched value from regulatorTask
  if (fuel_in_temporary < 3030 && fuel_in_temporary > 197)
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
    if (abs(raw - lastValue) <= 50)
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
      if ((goodSamples - badSamples) < 0)
      {
        filtered = raw;
      }
      goodSamples = 0;
      badSamples = 0;
      last_fuel_correction = now;
    }
    lastValue = filtered;
  }
  smoothVal = ((smoothVal * smoother) + filtered) / (smoother + 1);
  percent = map((int)smoothVal, 215, 1918, 0, 100);
  percent = constrain(percent, 0, 100);

  //-------------------- Coolant level
  coolant_level = digitalReadFast(coolant_level_pin);

  //================================ Read data from engine MCU
  //===============================================//

  while (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK)
  {
    if (canMsg.can_id == 0x02)
    {
      raw2 = (uint16_t)((canMsg.data[1] << 8) |
                        canMsg.data[0]); // Reconstruct 16-bit int from
                                         // data[0-1] (little-endian)
      spd_t = (uint16_t)(canMsg.data[3] << 8 | canMsg.data[2]);
      oil_level_t = (uint8_t)canMsg.data[6];
      injector_state = canMsg.data[4];

      lastPacketTime = now;
    }
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
    if ((goodSamples2 - badSamples2) < 0)
    {
      spd = spd_in;
    }
    goodSamples2 = 0;
    badSamples2 = 0;
    last_spd_correction = now;
  }

  if (spd != last_spd || spd == 0)
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

    tv.setCursor(100, 190);
    tv.setTextColor(0xFF, 0x00);
    tv.setTextSize(2);
    char spdStr[4];
    sprintf(spdStr, "%3d", spd);
    tv.print(spdStr);
    tv.setTextSize(1);

    last_spd = spd;
  }

  // --- Display voltage ---
  static float voltage_disp = 13.6f; // Initialize to target
  float volts = voltage_filtered;
  voltage_disp = 0.2f * volts + (1.0f - 0.2f) * voltage_disp;
  tv.setCursor(5, 40);
  tv.setTextColor(0xFF, 0x00);
  char bufV[10];
  sprintf(bufV, "%5.1fV", voltage_filtered);
  tv.print(bufV);

  // --- Display signed current (+ charge / - discharge) ---
  static float current_disp = 0.0f;
  // Display the filtered current, not the raw ADC value, for a steadier
  // readout.
  float current_d = current_A_filtered;
  current_disp = 0.1f * current_d + (1.0f - 0.1f) * current_disp;
  tv.setCursor(5, 55);
  tv.setTextColor(0xFF, 0x00);
  char bufI[10];
  sprintf(bufI, "%5.1fA", current_d);
  tv.print(bufI);

  // // --- Display charge state ---
  // tv.setCursor(5, 70);
  // tv.setTextColor(0xFF, 0x00);
  // if (chargeState == 0)
  // {
  //   if (current_d > (current_float_entry + 1.0f))
  //   {
  //     tv.print("BULK  "); // Heavy charging
  //   }
  //   else
  //   {
  //     tv.print("ABSORB"); // Tapering current
  //   }
  // }
  // else
  // {
  //   tv.print("FLOAT "); // Rest / self-discharge
  // }

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
      sprintf(buf, "%3d", percent);
      tv.print(buf);
    }

    //--------------------------------------------------------------------------
    fill2 = map(temp_out, 40, 120, 0, TEMP_HEIGHT); // ----------for temp gauge
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
        tv.fillRect(TEMP_X - 2, TEMP_Y + i, TEMP_TICKS_WIDTH, TEMP_TICKS_HEIGHT,
                    0xFF);
      }
      // draw temp_value tick
      tv.fillRect(TEMP_X - 5, t - 2, TEMP_VALUE_TICK_WIDTH,
                  TEMP_VALUE_TICK_HEIGHT, 0xE0);
      tv.setCursor(TEMP_X - 10, TEMP_Y - 10);
      tv.setTextColor(0xFF, 0x00); // Erase text automatically
      char buf[4];
      sprintf(buf, "%3d", temp_out);
      tv.print(buf);
    }

    lastTime = now;
    last_v = v;
    last_t = t;
  }
  oil_level = (int)oil_level_t;
  warnings(percent, temp_out, spd, coolant_level, oil_level, now);

  // ---------------- debug -----------------------------

  Serial.print(voltage_filtered);
  Serial.print("V   ");
  Serial.print(current_A_filtered);
  Serial.println("A");
  // Serial.println(task_duration);

  esp_task_wdt_reset();
}
