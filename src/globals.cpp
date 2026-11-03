#include "globals.h"
#include <Adafruit_ADS1X15.h>
#include <ESP_8_BIT_GFX.h>

#if USE_PAL_VIDEO
ESP_8_BIT_GFX tv(false, 8); // PAL (50Hz, 256x240, +16 scanlines)
#else
ESP_8_BIT_GFX tv(true, 8);  // NTSC (60Hz, 256x224)
#endif
Adafruit_ADS1115 adc;
portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;

RTC_DATA_ATTR SystemState currentState = STATE_SLEEP;
volatile bool phoneAuthorized = false;
RTC_DATA_ATTR bool phoneAuthBypassed = false;
RTC_NOINIT_ATTR uint32_t rtc_run_marker;     // Intentionally uninitialized (see globals.h)
RTC_NOINIT_ATTR uint32_t rtc_run_marker_inv; // Bitwise complement for validation

unsigned long standbyStartTime = 0;
bool stoppedToAcc = false;
volatile bool regulatorTaskRunning = true;

bool ecoInjCutActive = false;
unsigned long lastEngineStartTime = 0;

bool lowBlinkState = false;
bool lowBlinkState2 = false;

volatile int16_t temp_out = 40;
uint8_t percent = 0;
bool fuel_level_valid = false;

volatile uint32_t accumulated_inj_time_us = 0;
volatile uint32_t accumulated_inj_pulses = 0;
volatile uint32_t spd_delta_pulses = 0;
volatile float live_inj_duty_cycle = 0.0f;
volatile float live_net_pulse_us = 2000.0f;
RTC_DATA_ATTR double total_fuel_liters = 0.0;
RTC_DATA_ATTR double total_distance_km = 0.0;
RTC_DATA_ATTR float compounded_r_int = 0.0f;
RTC_DATA_ATTR double total_fuel_saved_liters = 0.0;
RTC_DATA_ATTR float last_known_avg_l_100km = 9.0f;
volatile float last_active_inj_pulse_us = 1500.0f;

int16_t spd = 0;
volatile uint16_t spd_t = 0;
volatile unsigned long lastPacketTime = 0;
volatile unsigned long lastFastPacketTime = 0;
volatile unsigned long lastEnvPacketTime = 0;
volatile float vacuum_psi = 0.0f;
RTC_DATA_ATTR float baro_psi = 0.0f;
volatile bool map_sensor_fault = false;
volatile unsigned long lastVacPacketTime = 0;
bool eco_warning_active = false;
volatile uint8_t oil_level_t = 0;
unsigned long resetPrintTime = 0;

uint8_t counter = 0;
bool coolant_level = false;
bool cool_run = true;
bool fuel_run = true;
bool oil_on = true;
bool temp_on = true;
volatile uint8_t injector_state = 0;
volatile uint8_t dfco_cut_state = 0;
volatile uint8_t th_switch_state = 1;

volatile float voltage_filtered = 13.6f;
volatile float current_A_filtered = 0.0f;
TaskHandle_t regulatorTaskHandle = nullptr;
volatile int16_t ads_fuel = 0;
volatile uint8_t charge_state = 0;
volatile uint32_t last_charge = 0;
volatile uint32_t last_regulator_heartbeat = 0;
volatile uint16_t rpm = 0;
