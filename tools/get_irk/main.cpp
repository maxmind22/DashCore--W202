/**
 * ============================================================================
 * W202 Phone Key Setup & IRK / MAC Extractor Tool
 * ============================================================================
 * 
 * This standalone tool helps you configure phone keys for the W202 Dashboard:
 * 
 * 1. FOR iPHONE & MODERN ANDROID (IRK-based passive detection):
 *    - Flash this environment to an ESP32 (pio run -e get_irk -t upload).
 *    - Open Serial Monitor at 250000 baud.
 *    - On your phone (iPhone or Android), go to Settings -> Bluetooth.
 *    - Tap "MB-BT-Audio-Setup" in the device list.
 *    - When prompted for a PIN / Passcode, enter: 702702
 *    - The Serial Monitor will print your phone's 16-byte IRK (Identity Resolving Key)
 *      formatted ready to copy-paste directly into src/secrets.h / src/config.h!
 * 
 * 2. FOR ANDROID 5.0.1 / LEGACY DEVICES (MAC & HID Auto-reconnect):
 *    - Pairing the device registers the ESP32 as an authorized HID input device.
 *    - The Serial Monitor will also print the phone's hardware/identity MAC address.
 *    - Add the MAC address to BLE_AUTHORIZED_MACS in src/secrets.h / src/config.h.
 * ============================================================================
 */

#include <Arduino.h>
#include <NimBLEDevice.h>

#if defined(CONFIG_NIMBLE_CPP_IDF)
#include "host/ble_hs.h"
#include "host/ble_store.h"
#else
#include "nimble/nimble/host/include/host/ble_hs.h"
#include "nimble/nimble/host/include/host/ble_store.h"
#endif

#if __has_include("../../src/secrets.h")
#include "../../src/secrets.h"
#elif __has_include("secrets.h")
#include "secrets.h"
#endif

#ifndef SETUP_DEVICE_NAME
#ifdef BLE_DEVICE_NAME
#define SETUP_DEVICE_NAME BLE_DEVICE_NAME "-Setup"
#else
#define SETUP_DEVICE_NAME "MB-BT-Audio-Setup"
#endif
#endif

#ifndef SETUP_PAIRING_PIN
#ifdef BLE_PAIRING_PIN
#define SETUP_PAIRING_PIN BLE_PAIRING_PIN
#else
#define SETUP_PAIRING_PIN 702702
#endif
#endif

#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID "87654321-4321-4321-4321-cba987654321"

static int irkIteratorCallback(int obj_type, union ble_store_value *val, void *cookie)
{
  if (obj_type == BLE_STORE_OBJ_TYPE_PEER_SEC && val->sec.irk_present)
  {
    Serial.println("🔑 [IRK KEY EXTRACTED (iPhone & Android RPA Resolution)]");
    Serial.println("Copy and paste this into BLE_AUTHORIZED_IRKS in src/secrets.h / src/config.h:");
    Serial.println("------------------------------------------------------------------");
    Serial.print("    { ");
    for (int i = 0; i < 16; i++)
    {
      Serial.printf("0x%02X%s", val->sec.irk[i], (i == 15) ? "" : ", ");
      if (i == 7)
        Serial.print("\n      ");
    }
    Serial.println(" }");
    Serial.println("------------------------------------------------------------------\n");
    bool *found = (bool *)cookie;
    if (found)
      *found = true;
  }
  return 0;
}

class ServerCallbacks : public NimBLEServerCallbacks
{
  void onConnect(NimBLEServer *pServer, ble_gap_conn_desc *desc) override
  {
    if (desc != nullptr)
    {
      NimBLEAddress peerAddr(desc->peer_ota_addr);
      Serial.printf("\n[+] Device connected! Address: %s\n", peerAddr.toString().c_str());
      Serial.printf("[+] Requesting passkey pairing (PIN: %06u)...\n", SETUP_PAIRING_PIN);
      NimBLEDevice::startSecurity(desc->conn_handle);
    }
    else
    {
      Serial.println("\n[+] Device connected!");
    }
  }

  uint32_t onPassKeyRequest() override
  {
    return SETUP_PAIRING_PIN;
  }

  bool onConfirmPIN(uint32_t pin) override
  {
    return true;
  }

  void onDisconnect(NimBLEServer *pServer) override
  {
    Serial.println("[-] Device disconnected. Advertising restarted.");
    NimBLEDevice::startAdvertising();
  }

  void onAuthenticationComplete(ble_gap_conn_desc *desc) override
  {
    if (desc == nullptr || !desc->sec_state.bonded)
    {
      Serial.println("[!] Pairing failed or not bonded.");
      return;
    }

    NimBLEAddress peerAddr(desc->peer_ota_addr);
    NimBLEAddress idAddr(desc->peer_id_addr);

    Serial.println("\n=======================================================");
    Serial.println("🎉 PAIRING SUCCESSFUL!");
    Serial.println("=======================================================");
    Serial.printf("Connected Device MAC:  %s\n", peerAddr.toString().c_str());
    if (idAddr.toString() != "00:00:00:00:00:00" && idAddr.toString() != peerAddr.toString())
    {
      Serial.printf("Identity MAC (Static): %s\n", idAddr.toString().c_str());
    }

    int numBonds = NimBLEDevice::getNumBonds();
    Serial.printf("Total Bonded Devices:  %d\n\n", numBonds);

    // 1. Output IRK (Identity Resolving Key) for iPhone & Android
    bool irkFound = false;
    ble_store_iterate(BLE_STORE_OBJ_TYPE_PEER_SEC, irkIteratorCallback, &irkFound);

    // 2. Output Static / Identity MAC address (for Android 5.0.1 and older devices)
    Serial.println("📱 [STATIC / IDENTITY MAC ADDRESS (For Android 5.0.1 / Whitelisting)]");
    Serial.println("Add this MAC address to BLE_AUTHORIZED_MACS in src/secrets.h / src/config.h:");
    Serial.println("------------------------------------------------------------");
    Serial.printf("    \"%s\",\n", peerAddr.toString().c_str());
    if (idAddr.toString() != "00:00:00:00:00:00" && idAddr.toString() != peerAddr.toString())
    {
      Serial.printf("    \"%s\",\n", idAddr.toString().c_str());
    }
    Serial.println("------------------------------------------------------------\n");
  }
};

class ScanCallbacks : public NimBLEAdvertisedDeviceCallbacks
{
  void onResult(NimBLEAdvertisedDevice *advertisedDevice) override
  {
    std::string name = advertisedDevice->getName();
    std::string mac = advertisedDevice->getAddress().toString();
    int rssi = advertisedDevice->getRSSI();

    if (name.length() > 0)
    {
      Serial.printf("[SCAN] Name: %-20s | MAC: %s | RSSI: %d dBm\n", name.c_str(), mac.c_str(), rssi);
    }
  }
};

static ServerCallbacks serverCallbacks;
static ScanCallbacks scanCallbacks;

void setup()
{
  Serial.begin(250000);
  delay(1000);

  Serial.println("\n==================================================");
  Serial.println("  W202 Phone Key Setup & IRK / MAC Extractor");
  Serial.println("==================================================");

  // Initialize NimBLE with passcode bonding enabled
  NimBLEDevice::init(SETUP_DEVICE_NAME);
  NimBLEDevice::setSecurityAuth(true, true, true); // bonding, mitm, sc
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);
  NimBLEDevice::setSecurityPasskey(SETUP_PAIRING_PIN);
  NimBLEDevice::setSecurityInitKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
  NimBLEDevice::setSecurityRespKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);

  // Setup GATT Server for Phone Pairing
  NimBLEServer *pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(&serverCallbacks);

  NimBLEService *pService = pServer->createService(SERVICE_UUID);
  NimBLECharacteristic *pChar = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::READ_AUTHEN |
      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC | NIMBLE_PROPERTY::WRITE_AUTHEN);
  pChar->setValue("DashCore Auth Setup");
  pService->start();

  // Start Advertising (Set appearance 0x03C1 / Keyboard so it shows up in iOS Settings -> Bluetooth)
  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setAppearance(0x03C1);
  pAdvertising->setName(SETUP_DEVICE_NAME);
  pAdvertising->start();

  Serial.printf("[*] BLE Advertising as '%s'.\n", SETUP_DEVICE_NAME);
  Serial.printf("[*] Connect your phone via Settings -> Bluetooth and enter PIN: %06u\n", SETUP_PAIRING_PIN);
  Serial.println("[*] Scanning nearby devices for Android MAC addresses...\n");

  // Start background scan to discover Android MAC addresses
  NimBLEScan *pScan = NimBLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(&scanCallbacks, false);
  pScan->setActiveScan(true);
  pScan->setDuplicateFilter(true);
  pScan->setInterval(100);
  pScan->setWindow(50);
  pScan->start(0, nullptr, false); // continuous background scan
}

void loop()
{
  delay(1000);
}
