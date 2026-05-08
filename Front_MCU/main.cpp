#include <digitalWriteFast.h>
#include <avr/wdt.h>
#include <mcp2515.h>
#include <SPI.h>

#define oil_level_pin 6
#define rpm_pin 2
#define spd_pin 3
#define th_pin 4
#define inj_pin 8
#define regulator_pin 9

const int tempPin = A0; 
const int fan = 5; 
const int ac = A1;
uint8_t oil_level = 0;
unsigned long last_check = 0;

// Variables shared with ISR MUST be volatile
volatile uint32_t lastTime = 0;
volatile uint32_t lastTime2 = 0;
volatile uint32_t period = 0;
volatile uint32_t period2 = 0;

// These are only used in loop(), so they do NOT need to be volatile
uint32_t rpm = 0;
uint32_t spd = 0;
uint16_t spd_s = 0;

int temp_avg = 0;     // Integer for EMA temp
int acState_avg = 0;  // Integer for EMA AC
bool injDisable = false;

// Timers
unsigned long lastSpdCalculationTime = 0;
unsigned long lastCanSendTime = 0;
unsigned long lastSensorTime = 0;

struct can_frame canMsg;
MCP2515 mcp2515(10, 8000000); 

//=============Interrupt Service Routine ===============//
void rpmISR() {
  uint32_t now = micros();
  period = now - lastTime;
  // if (p > 1000) { // Software debounce: 1ms (max 1000Hz)
  //   period = p;
  lastTime = now;
  // // }
}

void spdISR() {
  uint32_t now2 = micros();
  uint32_t p = now2 - lastTime2;
  if (p > 100) { // Software debounce: 2ms (max 500Hz)
    period2 = p;
    lastTime2 = now2;
  }
}

//=================== CAN Diagnostics ==================//
// void checkCanErrors() {
//   uint8_t errFlags = mcp2515.getErrorFlags();
//   if (errFlags != 0) {
//     if (errFlags & (MCP2515::EFLG_RX0OVR | MCP2515::EFLG_RX1OVR)) {
//       mcp2515.clearRXnOVR();
//     }
//     // Only completely reset the chip if it goes into Bus-Off (fatal state).
//     // Do NOT interfere if it's just in Error Passive (TXEP/RXEP); it will self-recover.
//     // Continuously calling setNormalMode() during Error Passive aborts transmissions 
//     // and blocks natural recovery.
//     if (errFlags & MCP2515::EFLG_TXBO) {
//       mcp2515.reset();
//       mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
//       mcp2515.setNormalOneShotMode();
//     }
//   }
// }

void setup() {
  pinMode(fan, OUTPUT);
  pinMode(tempPin, INPUT);
  pinModeFast(rpm_pin, INPUT);
  pinModeFast(spd_pin, INPUT);
  pinModeFast(th_pin, INPUT);
  pinModeFast(inj_pin, OUTPUT);
  pinMode(ac, INPUT);
  pinModeFast(oil_level_pin, INPUT);
  pinModeFast(regulator_pin, OUTPUT);
  digitalWrite(regulator_pin, HIGH);
  // Serial.begin(115200);
  wdt_disable();
  wdt_enable(WDTO_2S);
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(rpm_pin), rpmISR, FALLING);    
  attachInterrupt(digitalPinToInterrupt(spd_pin), spdISR, FALLING); 

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalOneShotMode(); //mcp2515.setNormalMode();
}

void loop() {
  unsigned long currentMillis = millis();
  unsigned long currentMicros = micros();

  //=================== Read Sensors & Control Fan (Every 1s) ======================//
  if (currentMillis - lastSensorTime >= 1000) {
    int temp_t = analogRead(tempPin);
    
    // Initialize averages on first run or use integer-based exponential moving average
    if (temp_avg == 0) temp_avg = temp_t;
    else temp_avg = temp_avg + (temp_t - temp_avg) / 16; 
    
    int dutyCycle_temp = map(temp_avg, 690, 730, 20, 255);
    dutyCycle_temp = constrain(dutyCycle_temp, 0, 255);

    int acState_t = analogRead(ac);
    if (acState_avg == 0) acState_avg = acState_t;
    else acState_avg = acState_avg + (acState_t - acState_avg) / 8;
    
    int dutyCycle_ac = map(acState_avg, 50, 500, 20, 255);
    dutyCycle_ac = constrain(dutyCycle_ac, 0, 255);
    
    int dutyCycle = max(dutyCycle_temp, dutyCycle_ac);
    analogWrite(fan, dutyCycle);

    oil_level = digitalReadFast(oil_level_pin);

    lastSensorTime = currentMillis;
  }

  //=================== Calculate RPM (Every loop) ======================//
  noInterrupts();
  uint32_t p = period;
  interrupts();
  if (p > 0) {
    rpm = 60000000UL / p;
  }

  //=================== Control Injectors =====================//
  int th_Pos = digitalReadFast(th_pin);
  static unsigned long last_inj_check = 0;

  if (rpm > 3000 && th_Pos == 1 && temp_avg > 440) {
    if (last_inj_check == 0) last_inj_check = currentMillis; // Start 1000ms timer
    if (currentMillis - last_inj_check >= 1000) {
      injDisable = true;
    }
  } else {
    last_inj_check = 0; // Reset timer if condition no longer met
  }

  // Deactivation is instant when throttle is released or RPM drops below hysteresis limit
  if (th_Pos == 0 || rpm < 2200) {
    injDisable = false;
  }
  digitalWriteFast(inj_pin, injDisable ? HIGH : LOW);

  //==================== Calculate Vehicle Speed (Every 100ms) ====================//
  if (currentMillis - lastSpdCalculationTime >= 100) {
    noInterrupts();
    uint32_t p2 = period2;
    uint32_t lt2 = lastTime2;
    interrupts();

    // Check if 1,000,000us (1s) has passed without a pulse -> Car stopped
    if (currentMicros - lt2 > 1000000UL) {
      spd = 0;
    } else 
    if (p2 > 0) {
      spd = 600000UL / p2; 
    }
    spd_s = (spd > 0) ? (uint16_t)spd : 0;
    
    lastSpdCalculationTime = currentMillis;
  }

static int alive = 0;
static unsigned long last_can = 0; 
  // Process CAN and auto-recover errors
  // checkCanErrors();
  if(mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK)
  {
    if (canMsg.can_id == 0x03)
    {
      alive = canMsg.data[0];
      last_check = currentMillis;
    }
  }
  
  // Failsafe timeout: if no message in 1000ms, assume dead
  if (currentMillis - last_check > 1000) {
    alive = 0; 
  }

uint8_t injDisable_s = (uint8_t)injDisable;
uint8_t rpm_s = rpm / 100;
  //================= Send to Display MCU (Every 200ms) ===============//
  if (currentMillis - lastCanSendTime >= 200) {
    uint16_t temp_s = (uint16_t)temp_avg;
    canMsg.can_id = 0x02;
    canMsg.can_dlc = 8;
    canMsg.data[0] = temp_s & 0xFF;  // lowByte(temp_s);
    canMsg.data[1] = temp_s >> 8;    // highByte(temp_s);
    canMsg.data[2] = spd_s & 0xFF;
    canMsg.data[3] = spd_s >> 8;
    canMsg.data[4] = injDisable_s;                 
    canMsg.data[5] = rpm_s;
    canMsg.data[6] = oil_level;
    canMsg.data[7] = 0;
    mcp2515.sendMessage(&canMsg);
    
    lastCanSendTime = currentMillis;
  }
  
  //================= Regulator Failsafe ===============//
  if (alive != 100) {
    digitalWriteFast(regulator_pin, LOW);
  } else {
    digitalWriteFast(regulator_pin, HIGH);
  }
  wdt_reset();
}
