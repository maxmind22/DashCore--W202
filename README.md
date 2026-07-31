# Custom ESP32/Arduino nano Dual-MCU Automotive Telemetry Dashboard, Smart Alternator Regulator, Push Start System

A production-grade, real-time vehicle telemetry display and software-defined alternator regulator system, push start system. This project uses a dual-microcontroller architecture connected via **CAN Bus** to safely monitor engine status, display telemetry on composite video screens, implement a closed-loop CV/CC (Constant Voltage / Constant Current) alternator field regulator tailored for a LiFePO4 start battery and a push start system replacing mechanical key.

---

## 🛠️ System Architecture

The project is split into two physical microcontrollers:

### 1. Front MCU (Engine Bay Controller - Arduino nano)

Located in the engine bay, this controller handles raw engine sensor acquisition and primary safety-critical components:

- **RPM, Fuel consumption & Speed Tracking:** High-frequency pulse counting via hardware interrupts.
- **Thermal Management:** Reads engine coolant temperature and monitors A/C state to dynamically control the electric radiator cooling fan using variable PWM duty cycles.
- **Custom DFCO (Deceleration Fuel Cut Off) Point:** Cuts fuel during deceleration to reduce fuel consumption
- **Failsafe System:** Features a physical watchdog timer (`avr/wdt.h`) and monitors heartbeat frames from the display unit. If communication is lost, it immediately cuts field voltage to the regulator for safety.
- **CAN Broadcast:** Packages engine telemetry (speed, RPM, oil level, temperatures) and broadcasts it over a 500kbps CAN Bus every 50ms.

### 2. Display & Regulator MCU (Cabin Controller - ESP32)

Located in the vehicle cabin, this ESP32 manages the display, alternator regulation, and ignition keyless entry on separate cores:

- **Composite Video Dash UI:** Renders a digital speedometer, digital speed readout, battery charge/discharge telemetry, fuel levels, fuel consumption, and warning systems directly to PAL/NTSC composite video outputs using the `ESP_8_BIT` composite library (leveraging the ESP32’s hardware DACs).
- **Advanced Fuel & Efficiency Telemetry:**
  - **Remaining Distance Range (`REM`):** Continuously calculates remaining driving range (km) based on remaining fuel and real-time average consumption (`avg_l_100km`).
- **Software-Defined Alternator Regulator (Core 0):** A high-priority FreeRTOS task running a closed-loop PID control loop. It samples alternator voltage and current through a high-precision ADS1115 ADC to dynamically drive the alternator field coil via 10-bit PWM. Implements seamless CV/CC regulation (targeting 13.6V max and a 20A current ceiling specifically to protect and optimize charging for a LiFePO4 start battery) with secondary physical relay emergency overrides for overcurrent/overvoltage protection.
- **Smart Keyless Push-to-Start:** Manages the ignition and engine start sequence via a non-blocking state machine driving 3 physical relays (ACC, IGN, Starter).
- **Cabin Alerts & Warnings:** Controls a physical chime buzzer and on-screen HUD flashes for:
  - Low Fuel Level (with noise-rejecting calibration tables)
  - Low Engine Oil / Coolant Levels
  - Engine Overheating (Temp > 96°C)
  - Battery Low / Charging System Malfunction
  - Front MCU Connection Timeout

---

## 🔑 Keyless Push-to-Start System

The ESP32 manages a smart, keyless push-to-start system designed to replicate and modernize the ignition sequence of the Mercedes W202 chassis:

- **Triple-Relay Control System:** Controls Terminal 15R (ACC), Terminal 15 (IGN), and Terminal 50 (Starter Solenoid) via physical relays.
- **Low-Power Deep Sleep (~15µA):**
  To prevent battery drain while parked, the cabin ESP32 automatically shuts down all peripherals (regulator task, I2S/DMA video, CAN, I2C, and Watchdogs) and enters deep sleep:
  - _In Standby (OFF) State:_ After 2 minutes of inactivity.
  - _In ACC (POS1) or IGNITION (POS2) States:_ After **2 hours** of inactivity (preventing battery drain if left on accidentally).
  - _Wake-up Mechanism:_ Uses an active-high `ext1` wake trigger tied to the vehicle's central locking unlock line. Since the line normally rests at 13.3V (holding the optocoupler ON, pin LOW) and pulses to 0V (optocoupler OFF, pin HIGH) on unlock, the ESP32 wakes up instantly and boots when the car is unlocked.
- **Non-Blocking Crank Sequence:**
  Due to the brake switch only receiving power in Position 2 (Ignition ON), the system check flow is:
  1. **First Press:** Activates ACC & IGN (Position 2 / `STATE_IGNITION`), powering the brake switch.
  2. **Brake Detection:** Once in Position 2, the ESP32 checks for the brake signal. If the brake is held (either immediately during the transition or pressed later), the fuel pump primes for **1000ms**, and the starter automatically engages—no second button press required.
  3. **Auto-Disengage:** The starter automatically disengages once the engine RPM exceeds **400 RPM**, or cuts out after a **5-second safety limit** if starting fails.
- **Double Starting Prevention:** If cranking times out, the system automatically falls back to the ACC position (`STATE_ACC`) and cuts the starter/ignition to prevent the user from accidentally grinding the starter gear on a running engine.
- **Rotary Cycle & Stop Flow:**
  - _When Off:_ Tapping the button cycles: OFF (`STATE_STANDBY`) $\rightarrow$ IGNITION (`STATE_IGNITION`) $\rightarrow$ ACC (`STATE_ACC`) $\rightarrow$ OFF.
  - _When Running:_ Tapping the button stops the engine:
    - **With Brake Held:** Stops the engine but keeps ACC active (system goes to `STATE_ACC` so you can listen to radio).
    - **Without Brake:** Stops the engine and shuts down all accessories immediately (system goes to `STATE_STANDBY`).

---

## 🔌 Hardware / Tech Stack

- **Processor Core:** ESP32 (Cabin Display, regulator & Push start system) & Arduino nano (Front Sensor Board)
- **Communication:** MCP2515 CAN Bus Controller (500Kbps over SPI)
- **ADC:** Adafruit ADS1115 (16-bit Sigma-Delta ADC for ultra-stable voltage & current reading in noisy engine environments)
- **Current Sensor:** FS500E2T Hall-effect current sensor
- **Display Output:** Native ESP32 composite video out (RCA composite cable connected directly to GPIO25/DAC1)
- **Libraries Used:**
  - `ESP_8_BIT Color Composite Video Library` (NTSC/PAL graphics output)
  - `autowp/autowp-mcp2515` (CAN communication)
  - `arminjo/digitalWriteFast` (High-speed GPIO operations)
  - `Adafruit ADS1X15` (I2C high-resolution analog reading)

---

## 📁 Repository Structure

- `/src/main.cpp` - ESP32 codebase containing the FreeRTOS telemetry rendering engine, warning logic, and the alternator PID regulator task.
- `/Front_MCU/main.cpp` - AVR codebase running the engine bay sensor acquisition, engine safety relays, fan PWM control, and CAN transmitter loop.
- `/platformio.ini` - PlatformIO build settings, build flags, and dependency definitions.
