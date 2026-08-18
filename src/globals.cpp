#include "globals.h"

ESP_8_BIT_GFX tv(true, 8);
Adafruit_ADS1115 adc;
MCP2515 mcp2515(5, 8000000);
portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;

struct can_frame canMsg;
struct can_frame canMsgTx;

RTC_DATA_ATTR SystemState currentState = STATE_SLEEP;
RTC_DATA_ATTR bool vehicleLockDisabled = false;
RTC_DATA_ATTR bool engineStartDisabled = false;

unsigned long standbyStartTime = 0;
unsigned long lastButtonPressTime = 0;
bool stoppedToAcc = false;
volatile bool regulatorTaskRunning = true;

unsigned long lastTime = 0;
unsigned long last_spd_correction = 0;
unsigned long lastSpeedTime = 0;

bool lowBlinkState = false;
bool lowBlinkState2 = false;
unsigned long lastBlinkTime = 0;
unsigned long lastBlinkTime2 = 0;

int fuel_in_temporary = 0;
int filtered = 0;
const int smoother = 3000;
const int blinkInterval = 400;
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

RTC_DATA_ATTR uint32_t rtc_trip_magic = 0;
uint32_t accumulated_inj_time_us = 0;
uint32_t accumulated_inj_pulses = 0;
float live_inj_duty_cycle = 0.0f;
RTC_DATA_ATTR float total_fuel_liters = 0.0f;
RTC_DATA_ATTR float total_distance_km = 0.0f;
RTC_DATA_ATTR float compounded_r_int = 0.0f;
RTC_DATA_ATTR float total_fuel_saved_liters = 0.0f;
float last_active_inj_pulse_us = 1500.0f;
float inst_val = 0.0f;
float avg_l_100km = 0.0f;

int v = 11;
int last_v = 0;
int fill = 0;
char fuel_color = 0x00;
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
unsigned long resetPrintTime = 0;

unsigned int counter = 0;
int last_spd = -1;
bool coolant_level = false;
int buzzer_state = 0;
int spd_l = 0;
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
volatile uint8_t health_state = 0;
int boot_chime = 0;
uint16_t new_rpm = 0;

volatile float voltage_filtered = 13.6f;
volatile float current_A_filtered = 0.0f;
TaskHandle_t regulatorTaskHandle;
volatile int ads_fuel = 0;
volatile int charge_state = 0;
int chg = 0;
int chg2 = 0;
volatile uint32_t last_charge = 0;
volatile uint32_t last_regulator_heartbeat = 0;
volatile uint16_t rpm = 0;
int field_pwm = 0;
uint16_t local_rpm = 0;
