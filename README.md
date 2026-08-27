# Custom ESP32/Arduino nano Dual-MCU Automotive Telemetry Dashboard, Smart Alternator Regulator, Push Start System

A production-grade, real-time vehicle telemetry display, software-defined alternator regulator, and smart keyless push-to-start system. This project uses a dual-microcontroller architecture connected via **CAN Bus** to safely monitor engine status, display telemetry on composite video screens, implement a closed-loop CV/CC (Constant Voltage / Constant Current) alternator field regulator tailored for a LiFePO4 start battery, and replace the mechanical ignition lock.

---

## 🛠️ System Architecture

The project is split into two physical microcontrollers:

### 1. Front MCU (Engine Bay Controller - Arduino Nano / ATmega328P)

Located in the engine bay, this controller handles raw engine sensor acquisition and primary safety-critical components:

- **RPM, Fuel Consumption & Speed Tracking:** High-frequency pulse counting via hardware interrupts.
- **Thermal Management:** Reads engine coolant temperature and monitors A/C state to dynamically control the electric radiator cooling fan using variable PWM duty cycles.
- **Custom DFCO (Deceleration Fuel Cut Off):** Cuts fuel during deceleration when RPM > 1500, throttle closed, and engine warm.
- **Failsafe System:** Features a hardware watchdog timer (`avr/wdt.h`), regulator failsafe hysteresis (preventing relay chatter), and heartbeat frame monitoring from the display unit. If communication is lost, it cuts field voltage to the regulator for safety.
- **CAN Broadcast:** Packages engine telemetry (speed, RPM, oil level, temperatures) and broadcasts it over a 500kbps CAN Bus every 50ms.

### 2. Display & Regulator MCU (Cabin Controller - ESP32)

Located in the vehicle cabin, this ESP32 manages the display, alternator regulation, and ignition keyless entry on separate FreeRTOS cores:

- **Composite Video Dash UI:** Renders a digital speedometer, digital speed readout, battery charge/discharge telemetry, fuel levels, fuel consumption, and warning systems directly to PAL/NTSC composite video outputs using the `ESP_8_BIT` composite library (leveraging the ESP32’s hardware DACs).
- **Advanced Fuel & Efficiency Telemetry:**
  - **Remaining Distance Range (`REM`):** Continuously calculates remaining driving range (km) based on remaining fuel and real-time average consumption (`avg_l_100km`).
- **Software-Defined Alternator Regulator (Core 0):** A high-priority FreeRTOS task running a closed-loop PID control loop. It samples alternator voltage and current through a high-precision ADS1115 ADC to dynamically drive the alternator field coil via 10-bit PWM. Implements seamless CV/CC regulation (targeting 13.6V max and a 20A current ceiling specifically to protect and optimize charging for a LiFePO4 start battery) with secondary physical relay emergency overrides for overcurrent/overvoltage protection.
- **Smart Keyless Push-to-Start:** Manages the ignition and engine start sequence via a non-blocking state machine driving 3 physical relays (ACC, IGN, Starter).
- **Cabin Alerts & Warnings:** Non-blocking buzzer tone queue and on-screen HUD flashes for:
  - Low Fuel Level (with noise-rejecting calibration tables)
  - Low Engine Oil / Coolant Levels
  - Engine Overheating (Temp > 96°C)
  - Battery Low / Charging System Malfunction
  - Front MCU Connection Timeout

---

## 🔑 Keyless Push-to-Start & Safety System

The ESP32 manages a smart, keyless push-to-start system designed to replicate and modernize the ignition sequence of the Mercedes W202 chassis:

- **Triple-Relay Control System:** Controls Terminal 15R (ACC), Terminal 15 (IGN), and Terminal 50 (Starter Solenoid) via physical relays.
- **Ultra-Low-Power Deep Sleep with Hardware Pin Hold:**
  To prevent battery drain while parked, the cabin ESP32 automatically shuts down all peripherals (regulator task, I2S/DMA video, CAN, I2C, and Watchdogs) and enters deep sleep:
  - _In Standby (OFF) State:_ After 2 minutes of inactivity.
  - _In ACC (POS1) or IGNITION (POS2) States:_ After **1 hour** of inactivity (preventing battery drain if left on accidentally).
  - _Hardware Pin Hold (`gpio_hold_en`):_ All relay output pins (ACC, IGN, Starter, 5V Gate, Field Relay) are locked LOW during sleep. This prevents floating input states caused by moisture or noise from ghost-engaging the starter or ignition while parked.
  - _Wake-up Mechanism:_ Uses an active-high `ext1` wake trigger tied to the vehicle's central locking unlock line.
- **Non-Blocking Crank & State Machine:**
  - **First Press:** Activates ACC & IGN (Position 2 / `STATE_IGNITION`), powering the brake switch.
  - **Brake Detection:** Checks for the brake signal. If the brake is held, the fuel pump primes for **50ms**, and the starter automatically engages.
  - **Auto-Disengage:** Disengages once engine RPM exceeds **400 RPM**, or cuts out after a **5-second safety limit** if starting fails.
  - **Stall Safety:** Stall detection requires CAN connectivity verification (`lastPacketTime < 2000ms`), ensuring a transient CAN signal loss at highway speed will never cut engine ignition.
- **Double Starting Prevention:** If cranking times out, the system automatically falls back to `STATE_ACC` to prevent gear grinding on a running engine.
- **📱 BLE Phone-as-Keyfob Engine Immobilizer:**
  - **Passive BLE Proximity Detection:** ESP32 scans for authorized phones on boot/wake and on start button presses. No app needed on the phone — standard Bluetooth advertising is used.
  - **Multi-Device Support:**
    - **Android:** Verified via static Bluetooth MAC address matching (`BLE_AUTHORIZED_MACS`).
    - **iPhone:** Verified via **IRK (Identity Resolving Key)** cryptographic resolution of rotating Resolvable Private Addresses (`BLE_AUTHORIZED_IRKS` using AES-128-ECB).
  - **Zero-Interference Auto Radio Shutdown:** The BLE stack and radio are **immediately powered off** once authorization is granted, preventing RF interference with the closed-loop PID alternator regulator or composite video DMA rendering.
  - **HUD Status & Audio Feedback:** Displays `"NO KEY DETECTED"` warning and emits warning beeps when starting is attempted without an authorized phone.
  - **Emergency Bypass:** 6 pulses on the central unlock line temporarily bypasses phone authorization for the current session (confirmed via 800ms chime).

---

## 🔌 Hardware / Tech Stack

- **Processor Core:** ESP32 (Cabin Display, Regulator & Push Start) & Arduino Nano (Engine Bay Sensor Controller)
- **Communication:** MCP2515 CAN Bus Controller (500Kbps over SPI with double-buffered RX drain)
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

The ESP32 codebase is fully modularized for clean separation of concerns:

- `/src/config.h` - System pin mapping, constants, macros, threshold definitions, and state enums.
- `/src/globals.h` / `/src/globals.cpp` - Shared state variables, RTC persistent data, and peripheral objects (`tv`, `adc`, `mcp2515`, `dataMux`).
- `/src/display.h` / `/src/display.cpp` - Composite video HUD graphics routines (`drawStaticGauge()`, `warnings()`).
- `/src/regulator.h` / `/src/regulator.cpp` - Closed-loop PID alternator field regulator task (Core 0) & I2C bus recovery logic.
- `/src/pushstart.h` / `/src/pushstart.cpp` - Keyless push-to-start state machine, non-blocking lock/buzzer timers, unlock handler, and deep sleep management.
- `/src/security.h` / `/src/security.cpp` - BLE phone-as-keyfob scanner, iPhone IRK cryptographic resolution, Android MAC matching, and auto-shutdown.
- `/src/can_comm.h` / `/src/can_comm.cpp` - CAN frame buffer drain (`drainCanRxBuffer`), error flag diagnostic clearing, and heartbeat transmitter.
- `/src/fuel.h` / `/src/fuel.cpp` - Non-linear fuel tank volume interpolation (`getFuelPercent()`) & trip data reset routines.
- `/src/main.cpp` - Clean ESP32 orchestrator containing `setup()` and `loop()`.
- `/tools/get_irk/main.cpp` - One-time setup tool to extract iPhone IRKs and scan Android Bluetooth MAC addresses (`pio run -e get_irk`).
- `/Front_MCU/main.cpp` - ATmega328P engine bay sensor acquisition, radiator fan PWM control, safety relays, and CAN broadcast loop.
- `/HARDWARE.md` - Complete hardware block diagram, netlist, and pinout schematics.
- `/platformio.ini` - PlatformIO build environments for `esp32dev`, `front_mcu`, and `get_irk`.

---

## 📜 License

This project is licensed under the MIT License - see the [LICENSE](file:///Users/mac/Documents/PlatformIO/Projects/Dashboard/LICENSE) file for details.


