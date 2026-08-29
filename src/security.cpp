#include "security.h"
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <mbedtls/aes.h>
#include <ctype.h>

#if defined(CONFIG_NIMBLE_CPP_IDF)
#include "host/ble_hs.h"
#include "host/ble_store.h"
#else
#include "nimble/nimble/host/include/host/ble_hs.h"
#include "nimble/nimble/host/include/host/ble_store.h"
#endif

#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID "87654321-4321-4321-4321-cba987654321"

// --- NVS-persisted IRK store (pair once → detected forever) ---
#define MAX_PERSISTED_IRKS 8
#define NVS_IRK_NAMESPACE  "ble_irks"

static uint8_t  persistedIRKs[MAX_PERSISTED_IRKS][16];
static uint8_t  persistedIRKCount = 0;

static bool bleInitialized = false;
static bool bleScanning = false;
static uint8_t dynamicBondedIRK[16] = {0};
static bool dynamicIRKPresent = false;

// Check if a 16-byte key is all zeros (unconfigured)
static bool isZeroKey(const uint8_t key[16])
{
  if (!key)
    return true;
  for (int i = 0; i < 16; i++)
  {
    if (key[i] != 0)
      return false;
  }
  return true;
}

// Centralized authorization helper
static void grantPhoneAuthorization(const char *reason, const char *deviceInfo)
{
  Serial.printf("\n=======================================================\n");
  Serial.printf("🎉 [SECURITY] Authorization Granted: %s\n", reason ? reason : "Phone Authorized");
  if (deviceInfo != nullptr && deviceInfo[0] != '\0')
  {
    Serial.printf("   Device: %s\n", deviceInfo);
  }
  Serial.printf("=======================================================\n\n");

  phoneAuthorized = true;

  NimBLEScan *pScan = NimBLEDevice::getScan();
  if (pScan != nullptr && pScan->isScanning())
  {
    pScan->stop();
  }

  NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
  if (pAdv != nullptr && pAdv->isAdvertising())
  {
    pAdv->stop();
  }
}

// Load all persisted IRKs from NVS into runtime array
static void loadPersistedIRKs()
{
  Preferences prefs;
  prefs.begin(NVS_IRK_NAMESPACE, true); // read-only
  persistedIRKCount = prefs.getUChar("count", 0);
  if (persistedIRKCount > MAX_PERSISTED_IRKS)
    persistedIRKCount = MAX_PERSISTED_IRKS;

  for (uint8_t i = 0; i < persistedIRKCount; i++)
  {
    char key[16];
    snprintf(key, sizeof(key), "irk_%u", i);
    size_t len = prefs.getBytes(key, persistedIRKs[i], 16);
    if (len != 16)
    {
      // Corrupt entry — zero it out
      memset(persistedIRKs[i], 0, 16);
    }
    else
    {
      Serial.printf("[SECURITY] Loaded persisted IRK #%u [REDACTED]\n", i);
    }
  }
  prefs.end();

  if (persistedIRKCount > 0)
    Serial.printf("[SECURITY] %u persisted IRK(s) loaded from NVS.\n", persistedIRKCount);
  else
    Serial.println("[SECURITY] No persisted IRKs in NVS. Pair a phone to auto-save its IRK.");
}

// Check if an IRK already exists in compile-time or persisted lists
static bool isIRKAlreadyKnown(const uint8_t irk[16])
{
  if (isZeroKey(irk))
    return true; // Don't store zero keys

  // Check against compile-time IRKs from secrets.h
  for (size_t i = 0; i < NUM_AUTHORIZED_IRKS; i++)
  {
    if (memcmp(irk, BLE_AUTHORIZED_IRKS[i], 16) == 0)
      return true;
    // Also check byte-reversed
    uint8_t rev[16];
    for (int j = 0; j < 16; j++)
      rev[j] = BLE_AUTHORIZED_IRKS[i][15 - j];
    if (memcmp(irk, rev, 16) == 0)
      return true;
  }

  // Check against already-persisted IRKs
  for (uint8_t i = 0; i < persistedIRKCount; i++)
  {
    if (memcmp(irk, persistedIRKs[i], 16) == 0)
      return true;
    uint8_t rev[16];
    for (int j = 0; j < 16; j++)
      rev[j] = persistedIRKs[i][15 - j];
    if (memcmp(irk, rev, 16) == 0)
      return true;
  }

  return false;
}

// Persist a new IRK to NVS and add to runtime array
static bool persistNewIRK(const uint8_t irk[16])
{
  if (isIRKAlreadyKnown(irk))
    return false; // Already known, skip

  if (persistedIRKCount >= MAX_PERSISTED_IRKS)
  {
    Serial.println("[SECURITY] ⚠️  IRK store full! Shifting out oldest entry.");
    // Shift all entries down, discarding [0] (oldest)
    for (uint8_t i = 0; i < MAX_PERSISTED_IRKS - 1; i++)
      memcpy(persistedIRKs[i], persistedIRKs[i + 1], 16);
    persistedIRKCount = MAX_PERSISTED_IRKS - 1;
  }

  // Add to runtime array
  memcpy(persistedIRKs[persistedIRKCount], irk, 16);
  persistedIRKCount++;

  // Write entire store to NVS and clean any stale keys
  Preferences prefs;
  prefs.begin(NVS_IRK_NAMESPACE, false); // read-write
  prefs.putUChar("count", persistedIRKCount);
  for (uint8_t i = 0; i < persistedIRKCount; i++)
  {
    char key[16];
    snprintf(key, sizeof(key), "irk_%u", i);
    prefs.putBytes(key, persistedIRKs[i], 16);
  }
  for (uint8_t i = persistedIRKCount; i < MAX_PERSISTED_IRKS; i++)
  {
    char key[16];
    snprintf(key, sizeof(key), "irk_%u", i);
    prefs.remove(key);
  }
  prefs.end();

  Serial.printf("\n🔐 [SECURITY] NEW IRK auto-persisted to NVS (slot #%u) [REDACTED]\n", persistedIRKCount - 1);
  Serial.println("   ✅ This phone will be detected passively from now on — no more pairing needed!\n");

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

/**
 * Implements Bluetooth Core Specification Vol 3, Part H, Sec 2.2.2 (RPA Resolution)
 * ah(k, r) = AES-128(k, 0^104 || r) mod 2^24
 *
 * In NimBLE/ESP32 little-endian address representation:
 *   - rpa[5] is MSB, rpa[0] is LSB.
 *   - Most significant 2 bits of rpa[5] must be 0b01 (0x40..0x7F).
 *   - prand (24-bit random part) is rpa[3..5] (rpa[5] is MSB, rpa[3] is LSB).
 *   - hash (24-bit hash part) is rpa[0..2] (rpa[2] is MSB, rpa[0] is LSB).
 */
static bool resolveRPA(const uint8_t rpa[6], const uint8_t irk[16])
{
  if (rpa == nullptr || isZeroKey(irk))
    return false;

  // An RPA has 0b01 in the top 2 bits of its most significant byte (rpa[5] in NimBLE)
  if ((rpa[5] & 0xC0) != 0x40)
    return false;

  // Form r' = 104 zeros || prand (big-endian 128-bit block for standard AES)
  uint8_t plaintext[16] = {0};
  plaintext[13] = rpa[5];
  plaintext[14] = rpa[4];
  plaintext[15] = rpa[3];

  // Standard BLE IRK is little-endian; mbedtls expects big-endian key
  uint8_t key_be[16];
  for (int i = 0; i < 16; i++)
    key_be[i] = irk[15 - i];

  uint8_t ciphertext[16];
  mbedtls_aes_context aes;
  mbedtls_aes_init(&aes);

  // 1. Evaluate with standard Little-Endian IRK (reversed for mbedtls)
  mbedtls_aes_setkey_enc(&aes, key_be, 128);
  mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, plaintext, ciphertext);

  // 24-bit hash is at the least-significant octets (ciphertext[13..15])
  if (ciphertext[13] == rpa[2] &&
      ciphertext[14] == rpa[1] &&
      ciphertext[15] == rpa[0])
  {
    mbedtls_aes_free(&aes);
    return true;
  }

  // 2. Evaluate with direct IRK buffer in case secrets.h IRK was written in big-endian hex
  mbedtls_aes_setkey_enc(&aes, irk, 128);
  mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, plaintext, ciphertext);
  mbedtls_aes_free(&aes);

  if (ciphertext[13] == rpa[2] &&
      ciphertext[14] == rpa[1] &&
      ciphertext[15] == rpa[0])
  {
    return true;
  }

  return false;
}

// Resolve an RPA against all NVS-persisted IRKs
static bool resolveRPAAgainstPersistedIRKs(const uint8_t rpa[6])
{
  for (uint8_t i = 0; i < persistedIRKCount; i++)
  {
    if (resolveRPA(rpa, persistedIRKs[i]))
      return true;
  }
  return false;
}

static int secStoreIteratorCallback(int obj_type, union ble_store_value *val, void *cookie)
{
  if (obj_type == BLE_STORE_OBJ_TYPE_PEER_SEC && val->sec.irk_present)
  {
    persistNewIRK(val->sec.irk);
    memcpy(dynamicBondedIRK, val->sec.irk, 16);
    dynamicIRKPresent = true;
    Serial.println("\n🔑 [SECURITY] Active Bonded Phone IRK extracted from Bond Store [REDACTED]\n");
    bool *found = (bool *)cookie;
    if (found)
      *found = true;
  }
  return 0;
}

class SecurityServerCallbacks : public NimBLEServerCallbacks
{
  void onConnect(NimBLEServer *pServer, ble_gap_conn_desc *desc) override
  {
    if (desc == nullptr)
      return;

    NimBLEAddress peerAddr(desc->peer_ota_addr);
    std::string devMacStr = peerAddr.toString();

    Serial.printf("[SECURITY] Connection received from: %s — initiating security handshake...\n", devMacStr.c_str());
    
    // Do NOT authorize here on raw connection. Trigger encryption/authentication first.
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

    Serial.printf("[SECURITY] Pairing / authentication successful for %s (bonded + encrypted)\n", devMacStr.c_str());

    // Extract live IRK from bond store
    bool irkFound = false;
    ble_store_iterate(BLE_STORE_OBJ_TYPE_PEER_SEC, secStoreIteratorCallback, &irkFound);

    // 1. Check against authorized Android MAC whitelist (now verified through encrypted link)
    for (size_t i = 0; i < NUM_AUTHORIZED_MACS; i++)
    {
      if (BLE_AUTHORIZED_MACS[i] && strlen(BLE_AUTHORIZED_MACS[i]) >= 12)
      {
        if (matchesMacString(devMacStr.c_str(), BLE_AUTHORIZED_MACS[i]) ||
            (idAddr.toString() != "00:00:00:00:00:00" && matchesMacString(idAddr.toString().c_str(), BLE_AUTHORIZED_MACS[i])))
        {
          grantPhoneAuthorization("Authorized Android Phone Connected & Authenticated!", devMacStr.c_str());
          return;
        }
      }
    }

    // 2. Check against authorized IRKs (compile-time from secrets.h)
    for (size_t i = 0; i < NUM_AUTHORIZED_IRKS; i++)
    {
      if (resolveRPA(desc->peer_ota_addr.val, BLE_AUTHORIZED_IRKS[i]) ||
          resolveRPA(desc->peer_id_addr.val, BLE_AUTHORIZED_IRKS[i]) ||
          (dynamicIRKPresent && memcmp(dynamicBondedIRK, BLE_AUTHORIZED_IRKS[i], 16) == 0))
      {
        grantPhoneAuthorization("Authorized Phone Connected & IRK Authenticated!", devMacStr.c_str());
        return;
      }
    }

    // 3. Check against NVS-persisted IRKs (auto-saved from previous pairings)
    if (resolveRPAAgainstPersistedIRKs(desc->peer_ota_addr.val) ||
        resolveRPAAgainstPersistedIRKs(desc->peer_id_addr.val) ||
        (dynamicIRKPresent && isIRKAlreadyKnown(dynamicBondedIRK)))
    {
      grantPhoneAuthorization("Authorized Phone Connected & Persisted IRK Matched!", devMacStr.c_str());
      return;
    }

    // 4. Authenticated with pairing PIN passkey — auto-persist IRK for future passive detection
    grantPhoneAuthorization("Phone Authenticated via PIN Passkey!", devMacStr.c_str());

    if (dynamicIRKPresent)
    {
      persistNewIRK(dynamicBondedIRK);
    }
  }

  uint32_t onPassKeyRequest() override
  {
    return BLE_PAIRING_PIN;
  }

  bool onConfirmPIN(uint32_t pin) override
  {
    bool match = (pin == BLE_PAIRING_PIN);
    if (!match)
    {
      Serial.printf("[SECURITY] PIN mismatch rejected: %06u (expected %06u)\n", (unsigned int)pin, (unsigned int)BLE_PAIRING_PIN);
    }
    return match;
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
          char info[64];
          snprintf(info, sizeof(info), "MAC: %s | RSSI: %d dBm", devMacStr.c_str(), advertisedDevice->getRSSI());
          grantPhoneAuthorization("Authorized Android Phone Detected!", info);
          return;
        }
      }
    }

    // 2. Check against authorized IRKs (compile-time from secrets.h)
    if (nativeAddr != nullptr)
    {
      for (size_t i = 0; i < NUM_AUTHORIZED_IRKS; i++)
      {
        if (resolveRPA(nativeAddr, BLE_AUTHORIZED_IRKS[i]) ||
            (dynamicIRKPresent && resolveRPA(nativeAddr, dynamicBondedIRK)))
        {
          char info[64];
          snprintf(info, sizeof(info), "RPA: %s | RSSI: %d dBm", devMacStr.c_str(), advertisedDevice->getRSSI());
          grantPhoneAuthorization("Authorized Phone IRK Resolved!", info);
          return;
        }
      }

      // 3. Check against NVS-persisted IRKs (auto-saved from previous pairings)
      if (resolveRPAAgainstPersistedIRKs(nativeAddr))
      {
        char info[64];
        snprintf(info, sizeof(info), "RPA: %s | RSSI: %d dBm", devMacStr.c_str(), advertisedDevice->getRSSI());
        grantPhoneAuthorization("Authorized Phone Persisted IRK Resolved!", info);
        return;
      }
    }
  }
};

static SecurityServerCallbacks serverCallbacks;
static SecurityScanCallbacks scanCallbacks;

static void onScanEnded(NimBLEScanResults results)
{
  bleScanning = false;
  NimBLEScan *pBLEScan = NimBLEDevice::getScan();
  if (pBLEScan != nullptr)
  {
    pBLEScan->clearResults();
  }

  if (phoneAuthorized)
  {
    Serial.println("[SECURITY] Authorization granted. BLE scan stopped.");
  }
  else
  {
    Serial.println("[SECURITY] BLE scan period ended without detecting authorized phone.");
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

    // 1. Load auto-persisted IRKs from NVS first to populate runtime array
    loadPersistedIRKs();

    // 2. Restore/sync bonded phone IRKs from NimBLE bond store for passive scanning
    bool irkRestored = false;
    ble_store_iterate(BLE_STORE_OBJ_TYPE_PEER_SEC, secStoreIteratorCallback, &irkRestored);
    if (irkRestored)
    {
      Serial.println("[SECURITY] Bonded Phone IRK restored from NVS for passive scan.");
    }
    else
    {
      Serial.println("[SECURITY] No bonded phone IRK found in NVS. Pair via PIN first.");
    }

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
    pAdvertising->setAppearance(0x03C1);
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
    pBLEScan->setActiveScan(false);                               // Passive scan (stealthy, saves power)
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
  memset(dynamicBondedIRK, 0, sizeof(dynamicBondedIRK));
  dynamicIRKPresent = false;
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
    pBLEScan->setActiveScan(false);
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

