#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

// CS pin is D10 on Front MCU
const int CS_PIN = 10;
MCP2515 mcp2515(CS_PIN);

struct can_frame canMsgTx;
struct can_frame canMsgRx;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

  Serial.println(F("\n=========================================="));
  Serial.println(F("     MCP2515 CAN Bus Loopback Test        "));
  Serial.println(F("=========================================="));

  SPI.begin();

  // 1. Reset MCP2515
  MCP2515::ERROR err = mcp2515.reset();
  if (err != MCP2515::ERROR_OK)
  {
    Serial.println(F("[-] MCP2515 Reset FAILED!"));
    Serial.println(F("    Check SPI wiring:"));
    Serial.println(F("      VCC  -> 5V"));
    Serial.println(F("      GND  -> GND"));
    Serial.println(F("      CS   -> D10"));
    Serial.println(F("      MOSI -> D11"));
    Serial.println(F("      MISO -> D12"));
    Serial.println(F("      SCK  -> D13"));
    while (1)
      delay(1000);
  }
  Serial.println(F("[+] MCP2515 Reset OK"));

  // 2. Set Bitrate (8MHz crystal standard on blue boards)
  err = mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  if (err != MCP2515::ERROR_OK)
  {
    Serial.println(F("[-] Set Bitrate FAILED! Try checking crystal frequency (8MHz vs 16MHz)."));
    while (1)
      delay(1000);
  }
  Serial.println(F("[+] Bitrate set: 500kbps @ 8MHz"));

  // 3. Set Internal Loopback Mode
  err = mcp2515.setLoopbackMode();
  if (err != MCP2515::ERROR_OK)
  {
    Serial.println(F("[-] Set Loopback Mode FAILED!"));
    while (1)
      delay(1000);
  }
  Serial.println(F("[+] Loopback Mode ENABLED"));
  Serial.println(F("[*] In Loopback Mode, TX is routed internally to RX inside the MCP2515."));
  Serial.println(F("[*] No external CAN transceiver, CANH/CANL, or termination resistors needed."));
  Serial.println(F("------------------------------------------\n"));

  // Configure initial test frame
  canMsgTx.can_id = 0x02; // Same as Front MCU Telemetry ID
  canMsgTx.can_dlc = 8;
  canMsgTx.data[0] = 0x00;
  canMsgTx.data[1] = 0x00;
  canMsgTx.data[2] = 0x12;
  canMsgTx.data[3] = 0x34;
  canMsgTx.data[4] = 0x56;
  canMsgTx.data[5] = 0x78;
  canMsgTx.data[6] = 0xAB;
  canMsgTx.data[7] = 0xCD;
}

void loop()
{
  static uint32_t packet_num = 0;
  canMsgTx.data[0] = (uint8_t)(packet_num & 0xFF);
  canMsgTx.data[1] = (uint8_t)((packet_num >> 8) & 0xFF);

  Serial.print(F("[TX] Packet #"));
  Serial.print(packet_num);
  Serial.print(F(" (ID: 0x0"));
  Serial.print(canMsgTx.can_id, HEX);
  Serial.print(F(" DLC: "));
  Serial.print(canMsgTx.can_dlc);
  Serial.print(F(" Data: "));
  for (int i = 0; i < canMsgTx.can_dlc; i++)
  {
    if (canMsgTx.data[i] < 0x10)
      Serial.print('0');
    Serial.print(canMsgTx.data[i], HEX);
    Serial.print(' ');
  }
  Serial.print(F(")... "));

  MCP2515::ERROR sendErr = mcp2515.sendMessage(&canMsgTx);
  if (sendErr == MCP2515::ERROR_OK)
  {
    Serial.println(F("SENT OK"));
  }
  else
  {
    Serial.print(F("TX ERROR (Code: "));
    Serial.print(sendErr);
    Serial.println(F(")"));
  }

  // Give internal logic a moment to transfer from TX buffer to RX buffer
  delay(20);

  // Check RX buffer
  if (mcp2515.readMessage(&canMsgRx) == MCP2515::ERROR_OK)
  {
    Serial.print(F("  [RX LOOPBACK] ID: 0x0"));
    Serial.print(canMsgRx.can_id, HEX);
    Serial.print(F(" DLC: "));
    Serial.print(canMsgRx.can_dlc);
    Serial.print(F(" Data: "));
    for (int i = 0; i < canMsgRx.can_dlc; i++)
    {
      if (canMsgRx.data[i] < 0x10)
        Serial.print('0');
      Serial.print(canMsgRx.data[i], HEX);
      Serial.print(' ');
    }
    Serial.println();

    // Verify payload matches
    bool match = (canMsgRx.can_id == canMsgTx.can_id) && (canMsgRx.can_dlc == canMsgTx.can_dlc);
    for (int i = 0; i < 8; i++)
    {
      if (canMsgRx.data[i] != canMsgTx.data[i])
        match = false;
    }

    if (match)
    {
      Serial.println(F("  --> [SUCCESS] Loopback data verified 100%!\n"));
    }
    else
    {
      Serial.println(F("  --> [ERROR] Data corrupted / payload mismatch!\n"));
    }
  }
  else
  {
    Serial.println(F("  --> [FAIL] No message in RX buffer!\n"));
  }

  packet_num++;
  delay(1000);
}
