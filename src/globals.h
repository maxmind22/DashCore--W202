#pragma once

#include "config.h"

extern ESP_8_BIT_GFX tv;
extern Adafruit_ADS1115 adc;
extern portMUX_TYPE dataMux;


extern RTC_DATA_ATTR SystemState currentState;
extern RTC_DATA_ATTR bool vehicleLockDisabled;
extern volatile bool phoneAuthorized;
extern RTC_DATA_ATTR bool phoneAuthBypassed;

// Crash-recovery marker: RTC_DATA_ATTR is re-initialized by the bootloader on every
// reset except deep-sleep wake, so a panic/WDT/brownout reset would lose currentState.
// RTC_NOINIT_ATTR is never touched by the bootloader and survives those resets.
#define RTC_RUN_MARKER_MAGIC 0x52554E21UL // "RUN!"
extern RTC_NOINIT_ATTR uint32_t rtc_run_marker;
extern RTC_NOINIT_ATTR uint32_t rtc_run_marker_inv;

extern unsigned long standbyStartTime;
extern unsigned long lastButtonPressTime;
extern bool stoppedToAcc;
extern volatile bool regulatorTaskRunning;

extern bool isEcoRestart;
extern bool ecoInjCutActive;
extern unsigned long lastEngineStartTime;
extern unsigned long autoStopStartTime;
extern unsigned long standstillStartTime;

extern unsigned long lastTime;
extern unsigned long last_spd_correction;
extern unsigned long lastSpeedTime;

extern bool lowBlinkState;
extern bool lowBlinkState2;
extern unsigned long lastBlinkTime;
extern unsigned long lastBlinkTime2;

extern int fuel_in_temporary;
extern int filtered;
extern const int smoother;
extern const int blinkInterval;
extern const int blinkInterval2;
extern float smoothVal;
extern int lastValue;
extern int raw;
extern int goodSamples;
extern int goodSamples2;
extern int badSamples;
extern int badSamples2;
extern unsigned long last_fuel_correction;
extern volatile int temp_out;
extern int percent;
extern float fuel_percent_f;   // Unrounded tank level (0..100 %) used for range calculation
extern bool fuel_level_valid;  // True once the first valid fuel sender sample was received

extern RTC_DATA_ATTR uint32_t rtc_trip_magic;
extern volatile uint32_t accumulated_inj_time_us;
extern volatile uint32_t accumulated_inj_pulses;
extern volatile uint32_t spd_delta_pulses;
extern volatile uint32_t can_packets_lost_02;
extern volatile float live_inj_duty_cycle;
extern volatile float live_net_pulse_us;
extern RTC_DATA_ATTR double total_fuel_liters;
extern RTC_DATA_ATTR double total_distance_km;
extern RTC_DATA_ATTR float compounded_r_int;
extern RTC_DATA_ATTR double total_fuel_saved_liters;
extern volatile float last_active_inj_pulse_us;
extern float inst_val;
extern float avg_l_100km;

extern int v;
extern int last_v;
extern int fill;
extern char fuel_color;
extern int t;
extern int last_t;
extern int fill2;

extern volatile int spd;
extern volatile uint16_t spd_t;
extern volatile uint16_t raw2;
extern volatile unsigned long lastPacketTime;
extern volatile float vacuum_psi;
extern RTC_DATA_ATTR float baro_psi;
extern volatile bool map_sensor_fault;
extern volatile uint8_t ac_switch_state;
extern volatile unsigned long lastVacPacketTime;
extern bool eco_warning_active;
extern volatile uint8_t oil_level_t;
extern int oil_level;
extern int last_clear;
extern unsigned long resetPrintTime;

extern unsigned int counter;
extern int last_spd;
extern bool coolant_level;
extern int buzzer_state;
extern int spd_l;
extern bool fuel;
extern bool cool;
extern bool cool_run;
extern bool speed_on;
extern bool oil;
extern bool hot;
extern bool fuel_run;
extern bool oil_on;
extern bool temp_on;
extern bool conn_on;
extern int overspeed_state;
extern volatile uint8_t injector_state;
extern volatile uint8_t th_switch_state;
extern bool inj_on;
extern const int over_speed_on;
extern const int over_speed_off;
extern volatile uint8_t health_state;
extern int boot_chime;
extern volatile uint16_t new_rpm;

extern volatile float voltage_filtered;
extern volatile float current_A_filtered;
extern TaskHandle_t regulatorTaskHandle;
extern volatile int ads_fuel;
extern volatile int charge_state;
extern int chg;
extern int chg2;
extern volatile uint32_t last_charge;
extern volatile uint32_t last_regulator_heartbeat;
extern volatile uint16_t rpm;
extern volatile int field_pwm;
