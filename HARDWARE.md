# 🔌 Hardware Schematics & Component Breakdown

This document provides a **complete, component-by-component hardware schematic and netlist specification** for both the **ESP32 Cabin Controller** and the **ATmega328P Front MCU**.

---

## 🛠️ Complete System Hardware Block Diagram

```mermaid
flowchart TD
    subgraph ESP32Cabin["Cabin Controller (ESP32-WROOM-32 38-Pin)"]
        direction TB

        %% Microcontroller Core
        ESP32Core["ESP32 Core\nCore 0: PID Regulator & ADS1115 Read\nCore 1: Composite Video, Push-Start & CAN"]

        %% Analog Sensing Subsystem (ADS1115 16-bit I2C ADC @ 0x48)
        ADS["ADS1115 ADC (I2C 0x48)\nAddress Pin ADDR -> GND"]
        ADS -- "Channel A0" --> VoltDiv["Alternator Voltage Divider\n(Resistor Divider to 13.6V Ceiling)"]
        ADS -- "Channel A1" --> HallSensor["FS500E2T Hall Current Sensor\n(2.5V Offset, 4mV/A Scale)"]
        ADS -- "Channel A2" --> FuelSender["Fuel Level Sender Tank Unit\n(Resistor Divider / Calibration Table)"]
        ADS <== "I2C (SDA: GPIO21, SCL: GPIO22)" ==> ESP32Core

        %% Alternator Field Drive Subsystem (IR2110PBF High/Low Driver)
        ESP32Core -- "GPIO12 (FIELD_PIN - 10-bit PWM)" --> IR2110["IR2110PBF Gate Driver IC\nVDD: 3.3V | VCC: 12V | LIN: GPIO12"]
        IR2110 -- "LO (Pin 1) via 10Ω Resistor" --> GateMOS["IRLZ44N / IRF3205 N-Ch MOSFET"]
        GateMOS -- "Drain" --> FieldCoilNeg["Field Coil (-) Terminal"]
        ESP32Core -- "GPIO14 (field_relay_pin)" --> FieldRelayDriver["ULN2003 Driver Ch 6"] --> FieldRelay["Emergency Field Disconnect Relay"]
        FieldRelay -- "Switched +12V" --> FieldCoilPos["Field Coil (+) Terminal"]
        FieldCoilPos <== "UF5408 High-Speed Flyback Diode" ==> FieldCoilNeg

        %% Push-Start & Power Management Outputs (ULN2003 Relay Transistor Array)
        ESP32Core -- "GPIO16 (PIN_RELAY_ACC)" --> ULN["ULN2003 Transistor Array"]
        ESP32Core -- "GPIO26 (PIN_RELAY_IGN)" --> ULN
        ESP32Core -- "GPIO13 (PIN_RELAY_START)" --> ULN
        ESP32Core -- "GPIO27 (PIN_5V_GATE)" --> ULN
        ESP32Core -- "GPIO32 (PIN_RELAY_LOCK)" --> ULN
        ESP32Core -- "GPIO17 (PIN_3V3_DIGITAL_GATE)" --> LevelShifterGate["P-Ch MOSFET Power Switch"]

        ULN --> RelayACC["Terminal 15R (Accessory Relay)"]
        ULN --> RelayIGN["Terminal 15 (Ignition POS2 Relay)"]
        ULN --> RelaySTART["Terminal 50 (Starter Solenoid Relay)"]
        ULN --> Relay5V["5V Rail Power Relay"]
        ULN --> RelayLock["Vehicle Lock Relay"]

        %% Isolated Inputs
        OptoBrake["Brake Switch (+12V Line)"] --> PC817_1["PC817 Optocoupler #1"] --> ESP32Core -- "GPIO36 (VP - Brake Input)"
        OptoUnlock["Central Lock Unlock (+12V Line)"] --> PC817_2["PC817 Optocoupler #2"] --> ESP32Core -- "GPIO35 (RTC Wake Trigger)"
        StartBtn["Push Start Button (Active Low)"] -- "10kΩ Pull-Up & 100nF Cap" --> ESP32Core -- "GPIO33 (RTC Wake Button)"
        CoolantSen["Coolant Level Switch"] -- "Pull-Down Resistor" --> ESP32Core -- "GPIO34 (coolant_level_pin)"

        %% CAN Communication Interface
        ESP32Core <== "SPI (CS: 5, SCK: 18, MISO: 19, MOSI: 23)" ==> MCP2515_Cabin["MCP2515 SPI CAN Controller"]
        MCP2515_Cabin <== "TxD / RxD" ==> TJA1050_Cabin["TJA1050 CAN Transceiver (with 120Ω Term Resistor)"]

        %% Audio & Video Peripherals
        ESP32Core -- "GPIO25 (DAC1)" --> RCAJack["Composite Video RCA (PAL/NTSC - 75Ω Impedance)"]
        ESP32Core -- "GPIO4 (buzzer_pin)" --> BuzzerDriver["NPN Transistor Driver"] --> Buzzer["Audible Alarm Buzzer"]
    end

    subgraph CANBus["500 kbps Differential CAN Bus"]
        TJA1050_Cabin <== "CAN_H / CAN_L Twisted Pair" ==> TJA1050_Front["TJA1050 CAN Transceiver (Front MCU)"]
    end

    subgraph FrontMCU["Engine Bay Controller (ATmega328P / Arduino Nano)"]
        TJA1050_Front <==> MCP2515_Front["MCP2515 CAN Controller"] <== "SPI (CS: D10)" ==> NanoCore["ATmega328P"]
        
        %% Inputs
        RPM_In["Engine RPM Signal"] -->|Opto / Zener Clamp| NanoCore -- "Pin D2 (INT0)"
        Speed_In["ABS Speed Pulse"] -->|Opto / Zener Clamp| NanoCore -- "Pin D3 (INT1)"
        Throttle_In["Throttle Closed Idle Switch"] -->|10k Pull-Up| NanoCore -- "Pin D4 (th_pin)"
        Inj_In["ECU Injector Signal"] -->|PC817 Opto| NanoCore -- "Pin D7 (PCINT23)"
        AC_In["Automatic A/C Signal"] -->|Resistor Divider| NanoCore -- "Pin A1 (ac)"
        Temp_In["Engine Temp NTC"] -->|Divider| NanoCore -- "Pin A0 (tempPin)"
        Oil_In["Oil Level Sensor"] --> NanoCore -- "Pin D6 (oil_level_pin)"

        %% Outputs
        NanoCore -- "Pin D5 PWM Signal" --> ToyotaPWMModule["Toyota OEM Cooling Fan PWM Driver Module"] --> RadFan["Radiator Fan Motor"]
        NanoCore -- "Pin D8 Output" --> DFCO_Relay["High-Side Injector Cutoff Relay"]
        NanoCore -- "Pin D9 Output" --> FrontFieldFailsafe["Safety Field Disconnect Line"]
    end
```

---

## 📌 Exhaustive ESP32 Cabin Controller Pin & Hardware Mapping

| ESP32 Pin | Variable / Net Name | Target Component / Circuit | Description & Circuit Protection |
| :--- | :--- | :--- | :--- |
| **GPIO12** | `FIELD_PIN` | **IR2110PBF Driver IC (Pin 12 - LIN)** | 10-bit PWM output for closed-loop PID field regulation. Inputs into 3.3V-logic compatible `LIN` pin on IR2110PBF. |
| **GPIO14** | `field_relay_pin` | **ULN2003 Driver Ch 6 -> Field Emergency Relay** | Hardware relay drive. Cuts high-side +12V power to the alternator field coil on overvoltage, overcurrent (>40A), or I2C sensor loss. |
| **GPIO16** | `PIN_RELAY_ACC` | **ULN2003 Driver Ch 1 -> Terminal 15R Relay** | Controls accessory power rail (Radio, HVAC, Aux). Active High. |
| **GPIO26** | `PIN_RELAY_IGN` | **ULN2003 Driver Ch 2 -> Terminal 15 Relay** | Controls primary ignition rail. Active High. Held during deep sleep via `gpio_hold_en`. |
| **GPIO13** | `PIN_RELAY_START` | **ULN2003 Driver Ch 3 -> Terminal 50 Relay** | Controls starter solenoid relay. Active High with non-blocking 5s max crank timeout. |
| **GPIO27** | `PIN_5V_GATE` | **ULN2003 Driver Ch 4 -> 5V Main Rail Relay** | Switches the main 5V buck rail on setup; held LOW during deep sleep. |
| **GPIO17** | `PIN_3V3_DIGITAL_GATE`| **P-Channel MOSFET Gate** | Gates +3.3V power to external digital level shifters and pull-ups to eliminate sleep leakage. |
| **GPIO32** | `PIN_RELAY_LOCK` | **ULN2003 Driver Ch 5 -> Lock Relay** | Pulsed Active High output to lock the vehicle doors automatically or on long press. |
| **GPIO33** | `PIN_BTN_START` | **Push-Start Button Switch** | Input-only with RTC capability. Connects to ground when pressed; filtered with 10kΩ pull-up & 100nF ceramic debouncing capacitor. |
| **GPIO35** | `PIN_WAKE_UNLOCK` | **PC817 Optocoupler #2 Collector** | Input-only with RTC capability. Senses vehicle central locking unlock 12V pulse to wake ESP32 from Deep Sleep (`ext1` wake trigger). |
| **GPIO36 (VP)**| `PIN_INPUT_BRAKE` | **PC817 Optocoupler #1 Collector** | Input-only (GPI). Active High logic sensing vehicle 12V brake light switch to allow engine start. |
| **GPIO34** | `coolant_level_pin`| **Engine Coolant Level Probe** | Input-only (GPI). Analog threshold input with pull-down resistor to detect low coolant expansion tank level. |
| **GPIO4** | `buzzer_pin` | **NPN Transistor (2N2222) -> Buzzer** | Drives active/passive piezo alert buzzer for cabin warnings (overheat, low fuel, charge failure). |
| **GPIO25 (DAC1)**| **Composite Video** | **RCA Female Jack** | Native hardware DAC output using `ESP_8_BIT_GFX` library. Impedance-matched 75Ω line output to PAL/NTSC dash display. |
| **GPIO21** | `I2C_SDA` | **ADS1115 16-bit ADC (Pin 4 SDA)** | I2C Data Line. Pulled up to 3.3V via 4.7kΩ resistors. Includes I2C bus recovery routine in software (`recoverI2CBus`). |
| **GPIO22** | `I2C_SCL` | **ADS1115 16-bit ADC (Pin 3 SCL)** | I2C Clock Line (100 kHz). Pulled up to 3.3V via 4.7kΩ resistors. |
| **GPIO5** | `CAN_CS` | **MCP2515 CAN Controller (Pin 16 CS)** | SPI Chip Select for CAN Bus communication. |
| **GPIO18** | `CAN_SCK` | **MCP2515 CAN Controller (Pin 14 SCK)** | SPI Serial Clock line. |
| **GPIO19** | `CAN_MISO` | **MCP2515 CAN Controller (Pin 15 MISO)**| SPI Master In Slave Out line. |
| **GPIO23** | `CAN_MOSI` | **MCP2515 CAN Controller (Pin 12 MOSI)**| SPI Master Out Slave In line. |

---

## 🔍 Detail Schematics for Specific Subsystems

### 1. Toyota OEM PWM Fan Driver Module Circuit (Front MCU Pin D5)

The ATmega328P Pin D5 outputs a 500Hz PWM signal mapped from engine coolant temperature (`temp_avg`) and A/C module state (`acState_avg`) directly into a **Toyota OEM Fan Control Module** (e.g., Toyota/Lexus 89257-30060 / 89257-12010).

```
 ATmega328P Pin D5 (PWM) ---[ 220Ω Resistor ]---> Gate / Duty Signal Line [ Toyota OEM Fan Module ]
                                                            |-- +12V Heavy Battery Cable (Fused 40A)
                                                            |-- Ground Heavy Chassis Cable
                                                            |-- Output to Radiator Fan Motor
```

---

### 2. ADS1115 16-Bit I2C ADC Module (Address 0x48) & Sensor Wiring

```
                         +3.3V Rail
                             |
             +---------------+---------------+
             |               |               |
          [ 4.7kΩ ]       [ 4.7kΩ ]          |
             |               |               |
 GPIO21 ----+--- SDA        |               |
 GPIO22 ---------+------- SCL               |
                             |               |
                         [ ADS1115 ADC IC ]  |
                         [ ADDR Pin -> GND  ] (Sets I2C address 0x48)
                             |       |       |
                 +-----------+       |       +-----------+
                 |                   |                   |
             Channel A0          Channel A1          Channel A2
                 |                   |                   |
        [ Voltage Divider ]    [ FS500E2T Hall ]     [ Fuel Tank Sender ]
    Alternator B+ (Max 13.6V)  Current Sensor Output   Resistor Sensor Array
    Multiplier: 0.0005422V/bit  Offset: 2500mV (4mV/A)  0% -> 100% Linearization
```

---

### 3. IR2110PBF Gate Driver IC & Field MOSFET Switching Subsystem

```
                         +12V Switched Field Supply Line
                                       |
                           [ Field Safety Relay ] (GPIO14 via ULN2003 Ch 6)
                                       |
                                       +------------------------------------+
                                       |                                    |
                                  [ + Field Coil ]                   [ UF5408 / 1N5408 ]
                                       |                             (High-Speed Flyback Diode)
                                  [ - Field Coil ]                         |
                                       |                                    |
                                       +------------------------------------+
                                       |
                                     Drain
                                       |
                                   [ IRLZ44N / IRF3205 N-Ch MOSFET ]
                                   [ Gate Pull-Down: 10kΩ to GND   ]
                                       |
                                     Source
                                       |
                                    System GND

   =========================== IR2110PBF DRIVER IC ===========================

        +3.3V ESP Rail ----> VDD (Pin 11)        VCC (Pin 3) <---- +12V Clean Rail
        ESP32 GPIO12   ----> LIN (Pin 12)        LO  (Pin 1) ----> [ 10Ω Gate Resistor ] ---> MOSFET Gate
        System GND     ----> COM (Pin 2)         VSS (Pin 13) ---> System GND
```

---

### 4. ULN2003 Relay Driver Array Circuit

```
  ESP32 Control Outputs                         ULN2003 Transistor Array                  Relay Coils (12V)
  ---------------------                         ------------------------                  -----------------

  GPIO16 (ACC)    --------------------------->  IN 1  (Pin 1) ----- OUT 1 (Pin 16) ------->  Terminal 15R Relay
  GPIO26 (IGN)    --------------------------->  IN 2  (Pin 2) ----- OUT 2 (Pin 15) ------->  Terminal 15 Relay
  GPIO13 (START)  --------------------------->  IN 3  (Pin 3) ----- OUT 3 (Pin 14) ------->  Terminal 50 Starter Relay
  GPIO27 (5V GATE)--------------------------->  IN 4  (Pin 4) ----- OUT 4 (Pin 13) ------->  5V Power Gate Relay
  GPIO32 (LOCK)   --------------------------->  IN 5  (Pin 5) ----- OUT 5 (Pin 12) ------->  Vehicle Lock Relay
  GPIO14 (FIELD)  --------------------------->  IN 6  (Pin 6) ----- OUT 6 (Pin 11) ------->  Field Safety Relay

  System GND      --------------------------->  GND   (Pin 8)
  +12V Rail       --------------------------->  COM   (Pin 9)  (Internal Flyback Clamp Diodes)
```

---

### 5. Optocoupler Input Conditioning Circuit (Brake GPIO36 & Unlock GPIO35)

```
  +12V Vehicle Signal (Brake / Lock Pulse) 
            |
      [ 1kΩ 1W Resistor ]
            |
         Pin 1 [ PC817 Optocoupler ]
         Pin 2 [ PC817 Optocoupler ] ----> Vehicle Chassis Ground

  +3.3V ESP32 VCC Rail
            |
      [ 10kΩ Pull-Up Resistor ]
            |
            +---> To ESP32 Input Pin (GPIO36 for Brake, GPIO35 for Unlock)
            |
         Pin 4 [ PC817 Optocoupler ]
         Pin 3 [ PC817 Optocoupler ] ----> ESP32 Signal Ground
```

---

### 6. Composite Video Output Circuit (GPIO25 / DAC1)

```
  ESP32 GPIO25 (DAC1 Output) ----[ 75Ω Series Resistor ]----+----> RCA Center Pin (Video Signal)
                                                             |
                                                    [ 75Ω Terminating Load ]
                                                             |
  ESP32 Signal Ground ---------------------------------------+----> RCA Outer Shield (GND)
```

---

### 7. Piezo Buzzer Driver Circuit (GPIO4)

```
  +5V Power Rail -----------------------------------------+
                                                          |
                                                     [ Piezo Buzzer ]
                                                     [ + 1N4007 Diode ]
                                                          |
  ESP32 GPIO4 ----[ 1kΩ Resistor ]----> Base [ 2N2222 NPN Transistor ]
                                       Collector ---------+
                                       Emitter -------------> System GND
```
