# Custom ESP32/AVR Dual-MCU Automotive Telemetry Dashboard & Smart Alternator Regulator

A production-grade, real-time vehicle telemetry display and software-defined alternator regulator system. This project uses a dual-microcontroller architecture connected via **CAN Bus** to safely monitor engine status, display telemetry on composite video screens, and implement a closed-loop CV/CC (Constant Voltage / Constant Current) alternator field regulator tailored for a LiFePO4 start battery.

---

## 🛠️ System Architecture

The project is split into two physical microcontrollers:

### 1. Front MCU (Engine Bay Controller - AVR/Arduino)
Located in the engine bay, this controller handles raw engine sensor acquisition and primary safety-critical components:
* **RPM & Speed Tracking:** High-frequency pulse counting via hardware interrupts (`rpmISR`, `spdISR`) with software debouncing.
* **Thermal Management:** Reads engine coolant temperature via thermistor and monitors A/C state to dynamically control the electric radiator cooling fan using variable PWM duty cycles.
* **Engine Safety Safeguards:** Implements a safety injector cut-off mechanism under high RPM/throttle conditions.
* **Failsafe System:** Features a physical watchdog timer (`avr/wdt.h`) and monitors heartbeat frames from the display unit. If communication is lost, it immediately cuts field voltage to the regulator for safety.
* **CAN Broadcast:** Packages engine telemetry (speed, RPM, oil level, temperatures) and broadcasts it over a 500kbps CAN Bus every 200ms.

### 2. Display & Regulator MCU (Cabin Controller - ESP32)
Located in the vehicle cabin, this ESP32 runs a real-time OS (FreeRTOS) to manage the display, alternator regulation, and ignition keyless entry on separate cores:
* **Composite Video Dash UI:** Renders a high-performance analog-style speedometer needle, digital speed readout, battery charge/discharge telemetry, fuel levels, and warning systems directly to PAL/NTSC composite video outputs using the `ESP_8_BIT` composite library (leveraging the ESP32’s hardware DACs).
* **Software-Defined Alternator Regulator (Core 0):** A high-priority FreeRTOS task running a closed-loop PID control loop. It samples alternator voltage and current through a high-precision ADS1115 ADC to dynamically drive the alternator field coil via 10-bit PWM. Implements seamless CV/CC regulation (targeting 13.6V max and a 20A current ceiling specifically to protect and optimize charging for a LiFePO4 start battery) with secondary physical relay emergency overrides for overcurrent/overvoltage protection.
* **Smart Keyless Push-to-Start:** Manages the ignition and engine start sequence via a non-blocking state machine driving 3 physical relays (ACC, IGN, Starter).
* **Cabin Alerts & Warnings:** Controls a physical chime buzzer and on-screen HUD flashes for:
  * Low Fuel Level (with noise-rejecting calibration tables)
  * Low Engine Oil / Coolant Levels
  * Engine Overheating (Temp > 96°C)
  * Battery Low / Charging System Malfunction
  * Front MCU Connection Timeout

---

## 🔑 Keyless Push-to-Start System

The ESP32 manages a smart, keyless push-to-start system designed to replicate and modernize the ignition sequence of the Mercedes W202 chassis:

* **Triple-Relay Control System:** Controls Terminal 15R (ACC), Terminal 15 (IGN), and Terminal 50 (Starter Solenoid) via physical relays.
* **Low-Power Deep Sleep (~15µA):** 
  To prevent battery drain while parked, the cabin ESP32 automatically shuts down all peripherals (regulator task, I2S/DMA video, CAN, I2C, and Watchdogs) and enters deep sleep after 2 minutes of inactivity.
  * *Wake-up Mechanism:* Uses an active-high `ext1` wake trigger tied to the vehicle's central locking unlock line. Since the line normally rests at 13.3V (holding the optocoupler ON, pin LOW) and pulses to 0V (optocoupler OFF, pin HIGH) on unlock, the ESP32 wakes up instantly and boots when the car is unlocked.
* **Non-Blocking Crank Sequence:** 
  When the brake is held and the start button is tapped:
  1. ACC & IGN activate.
  2. The system pauses for **500ms** to prime the fuel pump.
  3. The starter solenoid engages.
  4. The starter automatically disengages once the engine RPM exceeds **800 RPM**, or cuts out after a **5-second safety limit** if starting fails.
* **Double Starting Prevention:** If cranking times out, the system automatically falls back to the ACC position (`STATE_ACC`) and cuts the starter/ignition to prevent the user from accidentally grinding the starter gear on a running engine.
* **Engine Stop Flow:** Pressing the button while running (and stationary, speed = 0) shuts off the engine and ignition, placing the vehicle in the ACC (POS1) state. A second tap turns ACC off (OFF position) and starts the 2-minute sleep timer.

---

## 🔌 Hardware / Tech Stack

* **Processor Core:** ESP32 (Cabin Display & regulator) & ATmega328P/AVR (Front Sensor Board)
* **Communication:** MCP2515 CAN Bus Controller (500Kbps over SPI)
* **ADC:** Adafruit ADS1115 (16-bit Sigma-Delta ADC for ultra-stable voltage & current reading in noisy engine environments)
* **Current Sensor:** FS500E2T Hall-effect current sensor
* **Display Output:** Native ESP32 composite video out (RCA composite cable connected directly to GPIO25/DAC1)
* **Libraries Used:** 
  * `ESP_8_BIT Color Composite Video Library` (NTSC/PAL graphics output)
  * `autowp/autowp-mcp2515` (CAN communication)
  * `arminjo/digitalWriteFast` (High-speed GPIO operations)
  * `Adafruit ADS1X15` (I2C high-resolution analog reading)

---

## 📁 Repository Structure

* `/src/main.cpp` - ESP32 codebase containing the FreeRTOS telemetry rendering engine, warning logic, and the alternator PID regulator task.
* `/Front_MCU/main.cpp` - AVR codebase running the engine bay sensor acquisition, engine safety relays, fan PWM control, and CAN transmitter loop.
* `/platformio.ini` - PlatformIO build settings, build flags, and dependency definitions.
