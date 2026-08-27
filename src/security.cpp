#include "security.h"
#include <NimBLEDevice.h>
#include <mbedtls/aes.h>
#include <ctype.h>

#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID "87654321-4321-4321-4321-cba987654321"

static bool bleInitialized = false;
static bool bleScanning = false;

// Check if a 16-byte key is all zeros (unconfigured)
static bool isZeroKey(const uint8_t key[16])
{
  for (int i = 0; i < 16; i++)
  {
    if (key[i] != 0)
      return false;
  }
  return true;
}

// Case-insensitive MAC string comparison ignoring colons and dashes
static bool matchesMacString(const char *mac1, const char *mac2)
{
  if (!mac1 || !mac2)
    return false;
  while (*mac1 && *mac2)
  {
    while (*mac1 == ':' || *mac1 == '-' || *mac1 == ' ')
      mac1++;
    while (*mac2 == ':' || *mac2 == '-' || *mac2 == ' ')
      mac2++;
    if (!*mac1 || !*mac2)
      break;
    if (tolower((unsigned char)*mac1) != tolower((unsigned char)*mac2))
      return false;
    mac1++;
    mac2++;
  }
  while (*mac1 == ':' || *mac1 == '-' || *mac1 == ' ')
    mac1++;
  while (*mac2 == ':' || *mac2 == '-' || *mac2 == ' ')
    mac2++;
  return (*mac1 == '\0' && *mac2 == '\0');
}

// Single IRK evaluation helper using AES-128-ECB
static bool testIRK(const uint8_t rpa[6], const uint8_t irk[16])
{
  uint8_t plaintext[16] = {0};
  plaintext[13] = rpa[5];
  plaintext[14] = rpa[4];
  plaintext[15] = rpa[3];

  uint8_t ciphertext[16];
  mbedtls_aes_context aes;
  mbedtls_aes_init(&aes);
  mbedtls_aes_setkey_enc(&aes, irk, 128);
  mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, plaintext, ciphertext);
  mbedtls_aes_free(&aes);

  return (ciphertext[15] == rpa[0] &&
          ciphertext[14] == rpa[1] &&
          ciphertext[13] == rpa[2]);
}

// Implements Bluetooth Core Spec Vol 3, Part H, Sec 2.2.2 (RPA Resolution)
static bool resolveRPA(const uint8_t rpa[6], const uint8_t irk[16])
{
  // Resolvable Private Addresses have top two bits of MSB set to 0b01 (0x40 .. 0x7F)
  if ((rpa[5] & 0xC0) != 0x40)
    return false;
  if (isZeroKey(irk))
    return false;

  // Test forward key order (standard)
  if (testIRK(rpa, irk))
    return true;

  // Test reversed key order (for convenience if dumped in reversed byte order)
  uint8_t rev_irk[16];
  for (int i = 0; i < 16; i++)
  {
    rev_irk[i] = irk[15 - i];
  }
  return testIRK(rpa, rev_irk);
}

// Forward declaration
static void onScanEnded(NimBLEScanResults results);

class SecurityServerCallbacks : public NimBLEServerCallbacks
{
  void onConnect(NimBLEServer *pServer, ble_gap_conn_desc *desc) override
  {
    if (desc == nullptr)
      return;

    NimBLEAddress peerAddr(desc->peer_ota_addr);
    std::string devMacStr = peerAddr.toString();

    Serial.printf("[SECURITY] Direct connection from: %s — initiating security/pairing...\n", devMacStr.c_str());
    // Trigger passkey pairing & encryption. Result is handled in onAuthenticationComplete.
    NimBLEDevice::startSecurity(desc->conn_handle);
  }

  void onAuthenticationComplete(ble_gap_conn_desc *desc) override
  {
    if (desc == nullptr)
      return;

    NimBLEAddress peerAddr(desc->peer_ota_addr);
    NimBLEAddress idAddr(desc->peer_id_addr);
    std::string devMacStr = peerAddr.toString();

    if (!desc->sec_state.encrypted || !desc->sec_state.bonded)
    {
      Serial.printf("[SECURITY] Pairing / auth FAILED for %s — disconnecting.\n", devMacStr.c_str());
      NimBLEDevice::getServer()->disconnect(desc->conn_handle);
      return;
    }

    Serial.printf("[SECURITY] Pairing successful for %s (bonded + encrypted)\n", devMacStr.c_str());

    // Check against authorized Android MAC whitelist
    for (size_t i = 0; i < NUM_AUTHORIZED_MACS; i++)
    {
      if (BLE_AUTHORIZED_MACS[i] && strlen(BLE_AUTHORIZED_MACS[i]) >= 12)
      {
        if (matchesMacString(devMacStr.c_str(), BLE_AUTHORIZED_MACS[i]) ||
            (idAddr.toString() != "00:00:00:00:00:00" && matchesMacString(idAddr.toString().c_str(), BLE_AUTHORIZED_MACS[i])))
        {
          Serial.printf("[SECURITY] Authorized Phone Connected & Authenticated! MAC: %s\n", devMacStr.c_str());
          phoneAuthorized = true;
          NimBLEDevice::getScan()->stop();
          NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
          if (pAdv != nullptr)
            pAdv->stop();
          return;
        }
      }
    }

    // Paired successfully with PIN, but MAC is not in whitelist — reject connection
    Serial.printf("[SECURITY] UNAUTHORIZED paired device rejected! MAC: %s\n", devMacStr.c_str());
    NimBLEDevice::getServer()->disconnect(desc->conn_handle);
  }

  void onDisconnect(NimBLEServer *pServer) override
  {
    if (!phoneAuthorized && !isPhoneAuthorized())
    {
      NimBLEDevice::startAdvertising();
    }
  }
};

class SecurityScanCallbacks : public NimBLEAdvertisedDeviceCallbacks
{
  void onResult(NimBLEAdvertisedDevice *advertisedDevice) override
  {
    if (phoneAuthorized || advertisedDevice == nullptr)
      return;

    std::string devMacStr = advertisedDevice->getAddress().toString();
    const uint8_t *nativeAddr = advertisedDevice->getAddress().getNative();
    std::string devName = advertisedDevice->getName();

    if (devName.length() > 0)
    {
      Serial.printf("[BLE HEARD] Name: %-15s | MAC: %s | RSSI: %d dBm\n", devName.c_str(), devMacStr.c_str(), advertisedDevice->getRSSI());
    }
    else
    {
      Serial.printf("[BLE HEARD] MAC: %s | RSSI: %d dBm\n", devMacStr.c_str(), advertisedDevice->getRSSI());
    }

    // Optional RSSI threshold check
#if defined(BLE_MIN_RSSI) && (BLE_MIN_RSSI > -128)
    if (advertisedDevice->getRSSI() < BLE_MIN_RSSI)
    {
      return;
    }
#endif

    // 1. Check against authorized Android MAC addresses
    for (size_t i = 0; i < NUM_AUTHORIZED_MACS; i++)
    {
      if (BLE_AUTHORIZED_MACS[i] && strlen(BLE_AUTHORIZED_MACS[i]) >= 12)
      {
        if (matchesMacString(devMacStr.c_str(), BLE_AUTHORIZED_MACS[i]))
        {
          Serial.printf("[SECURITY] Authorized Android Phone Detected! MAC: %s (RSSI: %d dBm)\n",
                        devMacStr.c_str(), advertisedDevice->getRSSI());
          phoneAuthorized = true;
          NimBLEDevice::getScan()->stop();
          NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
          if (pAdv != nullptr)
            pAdv->stop();
          return;
        }
      }
    }

    // 2. Check against authorized iPhone IRKs
    if (nativeAddr != nullptr)
    {
      for (size_t i = 0; i < NUM_AUTHORIZED_IRKS; i++)
      {
        if (resolveRPA(nativeAddr, BLE_AUTHORIZED_IRKS[i]))
        {
          Serial.printf("[SECURITY] Authorized iPhone IRK Resolved! RPA: %s (RSSI: %d dBm)\n",
                        devMacStr.c_str(), advertisedDevice->getRSSI());
          phoneAuthorized = true;
          NimBLEDevice::getScan()->stop();
          NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
          if (pAdv != nullptr)
            pAdv->stop();
          return;
        }
      }
    }
  }
};

static SecurityServerCallbacks serverCallbacks;
static SecurityScanCallbacks scanCallbacks;

static void onScanEnded(NimBLEScanResults results)
{
  bleScanning = false;
  if (phoneAuthorized)
  {
    Serial.println("[SECURITY] Authorization granted. BLE scan stopped.");
  }
  else
  {
    Serial.println("[SECURITY] Scan ended. No authorized phone detected.");
  }
}

void setupBLESecurity()
{
  if (phoneAuthorized)
    return;

  if (!bleInitialized)
  {
    NimBLEDevice::init(BLE_DEVICE_NAME);
    NimBLEDevice::setSecurityAuth(true, true, true);
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);
    NimBLEDevice::setSecurityPasskey(BLE_PAIRING_PIN);
    NimBLEDevice::setSecurityInitKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
    NimBLEDevice::setSecurityRespKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);

    NimBLEServer *pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(&serverCallbacks);

    NimBLEService *pService = pServer->createService(SERVICE_UUID);
    NimBLECharacteristic *pChar = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::READ_AUTHEN |
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC | NIMBLE_PROPERTY::WRITE_AUTHEN);
    pChar->setValue("DashCore Auth");
    pService->start();

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setName(BLE_DEVICE_NAME);
    pAdvertising->start();

    bleInitialized = true;
  }
  else
  {
    NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
    if (pAdv != nullptr && !pAdv->isAdvertising())
    {
      pAdv->start();
    }
  }

  NimBLEScan *pBLEScan = NimBLEDevice::getScan();
  if (pBLEScan != nullptr)
  {
    pBLEScan->setAdvertisedDeviceCallbacks(&scanCallbacks, true); // wantDuplicates = true for continuous reception
    pBLEScan->setActiveScan(true);                                // Active scan
    pBLEScan->setInterval(100);                                   // 100ms interval
    pBLEScan->setWindow(99);                                      // 99ms window (near 100% duty cycle)

    uint32_t durationSec = BLE_SCAN_TIMEOUT_MS / 1000;
    if (durationSec == 0)
      durationSec = 1;

    Serial.printf("[SECURITY] Starting BLE scan / server for phone (%u sec)...\n", (unsigned int)durationSec);
    bleScanning = true;
    if (!pBLEScan->start(durationSec, onScanEnded, false))
    {
      Serial.println("[SECURITY] ERROR: BLE Scan failed to start!");
      bleScanning = false;
    }
  }
}

void teardownBLESecurity()
{
  if (bleInitialized)
  {
    NimBLEScan *pBLEScan = NimBLEDevice::getScan();
    if (pBLEScan != nullptr && pBLEScan->isScanning())
    {
      pBLEScan->stop();
      pBLEScan->clearResults();
    }
    NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
    if (pAdv != nullptr && pAdv->isAdvertising())
    {
      pAdv->stop();
    }
    bleScanning = false;
  }
}

bool isPhoneAuthorized()
{
  return phoneAuthorized || phoneAuthBypassed;
}

void setPhoneAuthBypass(bool bypass)
{
  phoneAuthBypassed = bypass;
}

void triggerBLERescan(unsigned long durationMs)
{
  if (phoneAuthorized || isPhoneAuthorized())
    return;

  if (isBLEScanning())
    return; // Already scanning

  if (!bleInitialized)
  {
    setupBLESecurity();
    return;
  }

  NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
  if (pAdv != nullptr && !pAdv->isAdvertising())
  {
    pAdv->start();
  }

  NimBLEScan *pBLEScan = NimBLEDevice::getScan();
  if (pBLEScan != nullptr)
  {
    pBLEScan->setAdvertisedDeviceCallbacks(&scanCallbacks, true);
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);

    uint32_t durationSec = durationMs / 1000;
    if (durationSec == 0)
      durationSec = 1;

    Serial.printf("[SECURITY] Re-scanning for phone (%u sec)...\n", (unsigned int)durationSec);
    bleScanning = true;
    if (!pBLEScan->start(durationSec, onScanEnded, false))
    {
      Serial.println("[SECURITY] ERROR: BLE Re-scan failed to start!");
      bleScanning = false;
    }
  }
}

bool isBLEScanning()
{
  if (!bleInitialized)
    return false;
  NimBLEScan *pBLEScan = NimBLEDevice::getScan();
  return (pBLEScan != nullptr && pBLEScan->isScanning());
}
