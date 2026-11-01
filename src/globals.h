#pragma once

#include "config.h"

class ESP_8_BIT_GFX;
class Adafruit_ADS1115;

extern ESP_8_BIT_GFX tv;
extern Adafruit_ADS1115 adc;
extern portMUX_TYPE dataMux;

extern RTC_DATA_ATTR SystemState currentState;
extern volatile bool phoneAuthorized;
extern RTC_DATA_ATTR bool phoneAuthBypassed;

// Crash-recovery marker: RTC_DATA_ATTR is re-initialized by the bootloader on every
// reset except deep-sleep wake, so a panic/WDT/brownout reset would lose currentState.
// RTC_NOINIT_ATTR is never touched by the bootloader and survives those resets.
#define RTC_RUN_MARKER_MAGIC 0x52554E21UL // "RUN!"
extern RTC_NOINIT_ATTR uint32_t rtc_run_marker;
extern RTC_NOINIT_ATTR uint32_t rtc_run_marker_inv;

extern unsigned long standbyStartTime;
extern bool stoppedToAcc;
extern volatile bool regulatorTaskRunning;

extern bool ecoInjCutActive;
extern unsigned long lastEngineStartTime;

extern bool lowBlinkState;
extern bool lowBlinkState2;

extern volatile int16_t temp_out;
extern uint8_t percent;
extern bool fuel_level_valid; // True once the first valid fuel sender sample was received

extern volatile uint32_t accumulated_inj_time_us;
extern volatile uint32_t accumulated_inj_pulses;
extern volatile uint32_t spd_delta_pulses;
extern volatile float live_inj_duty_cycle;
extern volatile float live_net_pulse_us;
extern RTC_DATA_ATTR double total_fuel_liters;
extern RTC_DATA_ATTR double total_distance_km;
extern RTC_DATA_ATTR float compounded_r_int;
extern RTC_DATA_ATTR double total_fuel_saved_liters;
extern volatile float last_active_inj_pulse_us;

extern int16_t spd;
extern volatile uint16_t spd_t;
extern volatile unsigned long lastPacketTime;
extern volatile unsigned long lastFastPacketTime;
extern volatile unsigned long lastEnvPacketTime;
extern volatile float vacuum_psi;
extern RTC_DATA_ATTR float baro_psi;
extern volatile bool map_sensor_fault;
extern volatile unsigned long lastVacPacketTime;
extern bool eco_warning_active;
extern volatile uint8_t oil_level_t;
extern unsigned long resetPrintTime;

extern uint8_t counter;
extern bool coolant_level;
extern bool cool_run;
extern bool fuel_run;
extern bool oil_on;
extern bool temp_on;
extern volatile uint8_t injector_state;
extern volatile uint8_t dfco_cut_state;
extern volatile uint8_t th_switch_state;

extern volatile float voltage_filtered;
extern volatile float current_A_filtered;
extern TaskHandle_t regulatorTaskHandle;
extern volatile int16_t ads_fuel;
extern volatile uint8_t charge_state;
extern volatile uint32_t last_charge;
extern volatile uint32_t last_regulator_heartbeat;
extern volatile uint16_t rpm;
