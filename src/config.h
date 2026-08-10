#pragma once

#include <Adafruit_ADS1X15.h>
#include <ESP_8_BIT_GFX.h>
#include <Preferences.h>
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>
#include <digitalWriteFast.h>
#include <driver/dac.h>
#include <driver/gpio.h>
#include <driver/rtc_io.h>
#include <esp_bt.h>
#include <esp_sleep.h>
#include <esp_task_wdt.h>
#include <math.h>
#include <mcp2515.h>
#include <soc/i2s_struct.h>

// --- PUSH START PIN DEFINITIONS ---
#define PIN_RELAY_ACC 16        // Terminal 15R (Accessory)
#define PIN_RELAY_IGN 26        // Terminal 15 (POS2/Ignition)
#define PIN_RELAY_START 13      // Terminal 50 (Starter)
#define PIN_BTN_START 33        // Push Button (Active Low, Pull-Up, RTC-capable)
#define PIN_INPUT_BRAKE 36      // Brake Light Sensor (Active High, Opto-isolated, Input-only)
#define PIN_WAKE_UNLOCK 35      // Unlock Pulse Input (Active Low, Opto-isolated, RTC Input-only)
#define PIN_5V_GATE 27          // Controls the 5V Relay (via ULN2003)
#define PIN_3V3_DIGITAL_GATE 17 // Powers the Level Shifter LV and digital 3.3V pull-ups
#define PIN_RELAY_LOCK 32       // Controls the vehicle lock relay (Active High)
#define buzzer_pin 4
#define coolant_level_pin 34
#define FIELD_PIN 12
#define field_relay_pin 14

// --- Threshold Constants ---
#define OVERSPEED_KMH 58
#define OVERHEAT_TEMP_C 96
#define ENGINE_STARTED_RPM 400
#define ENGINE_ACTIVE_RPM_THRESHOLD 200
#define EMERGENCY_OVERCURRENT_A 40.0f
#define FRONT_MCU_TIMEOUT_MS 5000
#define FRONT_MCU_CAN_TIMEOUT_MS 2000
#define FRONT_MCU_CAN_SEND_INTERVAL_US 50000.0f // Front MCU sends every 50ms
#define CHARGE_MALFUNCTION_DELAY_MS 20000
#define BATTERY_LOW_DELAY_MS 10000
#define CAN_HEALTH_SEND_INTERVAL_MS 200
#define REGULATOR_HEARTBEAT_TIMEOUT_MS 500
#define DISPLAY_METRICS_UPDATE_MS 250
#define FUEL_UPDATE_INTERVAL_MS 1000
#define RPM_UPDATE_INTERVAL_MS 500

// Display layout constants
#define FUEL_X 5
#define FUEL_Y 100
#define FUEL_HEIGHT 100
#define FUEL_WIDTH 15

#define TEMP_X 240
#define TEMP_Y 100
#define TEMP_HEIGHT 100
#define TEMP_TICKS_WIDTH 5
#define TEMP_TICKS_HEIGHT 2
#define TEMP_VALUE_TICK_WIDTH 11
#define TEMP_VALUE_TICK_HEIGHT 6

#define GAUGE_CX 128
#define GAUGE_CY 165
#define GAUGE_R 85
#define WARNING_X 50
#define WARNING_Y 20

// Fuel/Trip Constants
const int LOW_FUEL_LEVEL = 10;
const float INJECTOR_FLOW_RATE_CC_MIN = 228.0f; // Rated 228 cc/min @ 3.8 bar OEM regulator pressure
const int NUM_INJECTORS = 4;
const float FUEL_TANK_CAPACITY_LITERS = 62.0f;
const uint32_t MAX_INJ_PULSE_PER_INTERVAL_US = 1000000; // 1,000,000us (1s): handles multiple deferred CAN sends if Front MCU skips sending for up to 1 second
#define RTC_TRIP_MAGIC_KEY 0xCAFE4567

// Timeout Constants
const unsigned long STANDBY_TIMEOUT_MS = 120000;    // 2 Minute
const unsigned long ACCESSORY_TIMEOUT_MS = 3600000; // 1 Hour
const unsigned long BUTTON_COOLDOWN_MS = 500;
const unsigned long BUTTON_LONGPRESS_RESET_MS = 3000;
const unsigned long MAX_CRANK_TIME_MS = 5000;
const unsigned long BRAKE_CHECK_SETTLE_MS = 100; // Window (ms) to energize and continuously sample brake circuit

// --- SYSTEM STATES ---
enum SystemState
{
  STATE_SLEEP,
  STATE_STANDBY,
  STATE_ACC,
  STATE_IGNITION,
  STATE_CRANKING,
  STATE_RUNNING
};

enum ToneState
{
  TONE_IDLE,
  TONE_ON,
  TONE_OFF
};
enum LockRelayState
{
  LOCK_IDLE,
  LOCK_PULSE_ACTIVE
};

struct CalibrationPoint
{
  int rawValue;
  float percent;
};

const int NUM_CALIBRATION_POINTS = 11;
const CalibrationPoint calibrationTable[NUM_CALIBRATION_POINTS] = {
    {1326, 0.0f},
    {3383, 4.9f},
    {5440, 10.5f},
    {7498, 16.8f},
    {9555, 23.9f},
    {11613, 35.4f},
    {13670, 50.2f},
    {15727, 64.4f},
    {17785, 78.0f},
    {19842, 90.5f},
    {21900, 100.0f},
};
