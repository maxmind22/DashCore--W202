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
  for (int i = 0; i < 16; i++)
  {
    if (key[i] != 0)
      return false;
  }
  return true;
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
    char key[8];
    snprintf(key, sizeof(key), "irk_%u", i);
    size_t len = prefs.getBytes(key, persistedIRKs[i], 16);
    if (len != 16)
    {
      // Corrupt entry — zero it out
      memset(persistedIRKs[i], 0, 16);
    }
    else
    {
      Serial.printf("[SECURITY] Loaded persisted IRK #%u: { ", i);
      for (int j = 0; j < 16; j++)
        Serial.printf("0x%02X%s", persistedIRKs[i][j], (j == 15) ? "" : ", ");
      Serial.println(" }");
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
    // Also check byte-reversed (NimBLE sometimes stores IRKs in reversed order)
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

  // Write entire store to NVS
  Preferences prefs;
  prefs.begin(NVS_IRK_NAMESPACE, false); // read-write
  prefs.putUChar("count", persistedIRKCount);
  for (uint8_t i = 0; i < persistedIRKCount; i++)
  {
    char key[8];
    snprintf(key, sizeof(key), "irk_%u", i);
    prefs.putBytes(key, persistedIRKs[i], 16);
  }
  prefs.end();

  Serial.printf("\n🔐 [SECURITY] NEW IRK auto-persisted to NVS (slot #%u):\n   IRK: { ", persistedIRKCount - 1);
  for (int i = 0; i < 16; i++)
    Serial.printf("0x%02X%s", irk[i], (i == 15) ? "" : ", ");
  Serial.println(" }");
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

// Single IRK evaluation helper using AES-128-ECB across standard and swapped byte arrangements
static bool testSingleVariant(const uint8_t prand[3], const uint8_t expected_hash[3], const uint8_t irk[16])
{
  uint8_t plaintext[16] = {0};
  // r' = 104 zeros || prand (big-endian)
  plaintext[13] = prand[0];
  plaintext[14] = prand[1];
  plaintext[15] = prand[2];

  uint8_t ciphertext[16];
  mbedtls_aes_context aes;
  mbedtls_aes_init(&aes);
  mbedtls_aes_setkey_enc(&aes, irk, 128);
  mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, plaintext, ciphertext);
  mbedtls_aes_free(&aes);

  // Check big-endian hash
  if (ciphertext[13] == expected_hash[0] &&
      ciphertext[14] == expected_hash[1] &&
      ciphertext[15] == expected_hash[2])
    return true;

  // Check reversed hash
  if (ciphertext[15] == expected_hash[0] &&
      ciphertext[14] == expected_hash[1] &&
      ciphertext[13] == expected_hash[2])
    return true;

  // Also test little-endian plaintext padding: pt[0..2] = prand, pt[3..15] = 0
  uint8_t pt_le[16] = {0};
  pt_le[0] = prand[0];
  pt_le[1] = prand[1];
  pt_le[2] = prand[2];

  mbedtls_aes_init(&aes);
  mbedtls_aes_setkey_enc(&aes, irk, 128);
  mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, pt_le, ciphertext);
  mbedtls_aes_free(&aes);

  if (ciphertext[0] == expected_hash[0] &&
      ciphertext[1] == expected_hash[1] &&
      ciphertext[2] == expected_hash[2])
    return true;

  if (ciphertext[2] == expected_hash[0] &&
      ciphertext[1] == expected_hash[1] &&
      ciphertext[0] == expected_hash[2])
    return true;

  return false;
}

// Implements Bluetooth Core Spec Vol 3, Part H, Sec 2.2.2 (RPA Resolution)
static bool resolveRPA(const uint8_t rpa[6], const uint8_t irk[16])
{
  if (isZeroKey(irk) || rpa == nullptr)
    return false;

  // An RPA has 0b01 in the top 2 bits of its most significant byte (0x40..0x7F).
  bool rpa5_is_rpa = ((rpa[5] & 0xC0) == 0x40);
  bool rpa0_is_rpa = ((rpa[0] & 0xC0) == 0x40);

  if (!rpa5_is_rpa && !rpa0_is_rpa)
    return false; // Not a resolvable private address

  uint8_t rev_irk[16];
  for (int i = 0; i < 16; i++)
    rev_irk[i] = irk[15 - i];

  // If rpa[5] is MSB: prand is rpa[3..5], hash is rpa[0..2]
  if (rpa5_is_rpa)
  {
    uint8_t prand_fwd[3] = {rpa[5], rpa[4], rpa[3]};
    uint8_t prand_rev[3] = {rpa[3], rpa[4], rpa[5]};
    uint8_t hash_fwd[3]  = {rpa[2], rpa[1], rpa[0]};
    uint8_t hash_rev[3]  = {rpa[0], rpa[1], rpa[2]};

    if (testSingleVariant(prand_fwd, hash_fwd, irk)) return true;
    if (testSingleVariant(prand_fwd, hash_fwd, rev_irk)) return true;
    if (testSingleVariant(prand_fwd, hash_rev, irk)) return true;
    if (testSingleVariant(prand_fwd, hash_rev, rev_irk)) return true;
    if (testSingleVariant(prand_rev, hash_fwd, irk)) return true;
    if (testSingleVariant(prand_rev, hash_fwd, rev_irk)) return true;
    if (testSingleVariant(prand_rev, hash_rev, irk)) return true;
    if (testSingleVariant(prand_rev, hash_rev, rev_irk)) return true;
  }

  // If rpa[0] is MSB: prand is rpa[0..2], hash is rpa[3..5]
  if (rpa0_is_rpa)
  {
    uint8_t prand_fwd[3] = {rpa[0], rpa[1], rpa[2]};
    uint8_t prand_rev[3] = {rpa[2], rpa[1], rpa[0]};
    uint8_t hash_fwd[3]  = {rpa[5], rpa[4], rpa[3]};
    uint8_t hash_rev[3]  = {rpa[3], rpa[4], rpa[5]};

    if (testSingleVariant(prand_fwd, hash_fwd, irk)) return true;
    if (testSingleVariant(prand_fwd, hash_fwd, rev_irk)) return true;
    if (testSingleVariant(prand_fwd, hash_rev, irk)) return true;
    if (testSingleVariant(prand_fwd, hash_rev, rev_irk)) return true;
    if (testSingleVariant(prand_rev, hash_fwd, irk)) return true;
    if (testSingleVariant(prand_rev, hash_fwd, rev_irk)) return true;
    if (testSingleVariant(prand_rev, hash_rev, irk)) return true;
    if (testSingleVariant(prand_rev, hash_rev, rev_irk)) return true;
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
    memcpy(dynamicBondedIRK, val->sec.irk, 16);
    dynamicIRKPresent = true;
    Serial.println("\n🔑 [SECURITY] Active Bonded Phone IRK extracted from Bond Store:");
    Serial.print("   IRK: { ");
    for (int i = 0; i < 16; i++)
    {
      Serial.printf("0x%02X%s", val->sec.irk[i], (i == 15) ? "" : ", ");
    }
    Serial.println(" }\n");
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
    NimBLEAddress idAddr(desc->peer_id_addr);
    std::string devMacStr = peerAddr.toString();

    Serial.printf("[SECURITY] Direct connection from: %s\n", devMacStr.c_str());

    // 1. Check immediately if this MAC is in our authorized Android whitelist
    for (size_t i = 0; i < NUM_AUTHORIZED_MACS; i++)
    {
      if (BLE_AUTHORIZED_MACS[i] && strlen(BLE_AUTHORIZED_MACS[i]) >= 12)
      {
        if (matchesMacString(devMacStr.c_str(), BLE_AUTHORIZED_MACS[i]) ||
            (idAddr.toString() != "00:00:00:00:00:00" && matchesMacString(idAddr.toString().c_str(), BLE_AUTHORIZED_MACS[i])))
        {
          Serial.printf("\n=======================================================\n");
          Serial.printf("🎉 [SECURITY] Whitelisted Android Phone Connected!\n");
          Serial.printf("   MAC: %s\n", devMacStr.c_str());
          Serial.printf("=======================================================\n\n");
          phoneAuthorized = true;
          NimBLEDevice::getScan()->stop();
          NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
          if (pAdv != nullptr)
            pAdv->stop();
          return;
        }
      }
    }

    // 2. Check if this is an authorized IRK device (compile-time from secrets.h)
    for (size_t i = 0; i < NUM_AUTHORIZED_IRKS; i++)
    {
      if (resolveRPA(desc->peer_ota_addr.val, BLE_AUTHORIZED_IRKS[i]) ||
          resolveRPA(desc->peer_id_addr.val, BLE_AUTHORIZED_IRKS[i]) ||
          (dynamicIRKPresent && memcmp(dynamicBondedIRK, BLE_AUTHORIZED_IRKS[i], 16) == 0))
      {
        Serial.printf("\n=======================================================\n");
        Serial.printf("🎉 [SECURITY] Authorized Phone Connected & IRK Resolved!\n");
        Serial.printf("   MAC: %s\n", devMacStr.c_str());
        Serial.printf("=======================================================\n\n");
        phoneAuthorized = true;
        NimBLEDevice::getScan()->stop();
        NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
        if (pAdv != nullptr)
          pAdv->stop();
        return;
      }
    }

    // 3. Check against NVS-persisted IRKs (auto-saved from previous pairings)
    if (resolveRPAAgainstPersistedIRKs(desc->peer_ota_addr.val) ||
        resolveRPAAgainstPersistedIRKs(desc->peer_id_addr.val) ||
        (dynamicIRKPresent && isIRKAlreadyKnown(dynamicBondedIRK)))
    {
      Serial.printf("\n=======================================================\n");
      Serial.printf("🎉 [SECURITY] Authorized Phone Connected & Persisted IRK Resolved!\n");
      Serial.printf("   MAC: %s\n", devMacStr.c_str());
      Serial.printf("=======================================================\n\n");
      phoneAuthorized = true;
      NimBLEDevice::getScan()->stop();
      NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
      if (pAdv != nullptr)
        pAdv->stop();
      return;
    }

    // 4. Truly unrecognized device — initiate passkey pairing / encryption to discover IRK or PIN auth
    Serial.printf("[SECURITY] Unrecognized device %s — initiating security/pairing...\n", devMacStr.c_str());
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

    // Extract live IRK from bond store
    bool irkFound = false;
    ble_store_iterate(BLE_STORE_OBJ_TYPE_PEER_SEC, secStoreIteratorCallback, &irkFound);

    // 1. Check against authorized Android MAC whitelist
    for (size_t i = 0; i < NUM_AUTHORIZED_MACS; i++)
    {
      if (BLE_AUTHORIZED_MACS[i] && strlen(BLE_AUTHORIZED_MACS[i]) >= 12)
      {
        if (matchesMacString(devMacStr.c_str(), BLE_AUTHORIZED_MACS[i]) ||
            (idAddr.toString() != "00:00:00:00:00:00" && matchesMacString(idAddr.toString().c_str(), BLE_AUTHORIZED_MACS[i])))
        {
          Serial.printf("[SECURITY] Authorized Android Phone Connected & Authenticated! MAC: %s\n", devMacStr.c_str());
          phoneAuthorized = true;
          NimBLEDevice::getScan()->stop();
          NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
          if (pAdv != nullptr)
            pAdv->stop();
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
        Serial.printf("[SECURITY] Authorized Phone Connected & IRK Authenticated! MAC: %s\n", devMacStr.c_str());
        phoneAuthorized = true;
        NimBLEDevice::getScan()->stop();
        NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
        if (pAdv != nullptr)
          pAdv->stop();
        return;
      }
    }

    // 3. Check against NVS-persisted IRKs (auto-saved from previous pairings)
    if (resolveRPAAgainstPersistedIRKs(desc->peer_ota_addr.val) ||
        resolveRPAAgainstPersistedIRKs(desc->peer_id_addr.val) ||
        (dynamicIRKPresent && isIRKAlreadyKnown(dynamicBondedIRK)))
    {
      Serial.printf("[SECURITY] Authorized Phone Connected & Persisted IRK Matched! MAC: %s\n", devMacStr.c_str());
      phoneAuthorized = true;
      NimBLEDevice::getScan()->stop();
      NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
      if (pAdv != nullptr)
        pAdv->stop();
      return;
    }

    // 4. Fallback: Authenticated with pairing PIN passkey — auto-persist IRK for future passive detection
    Serial.printf("[SECURITY] Phone Authenticated via PIN Passkey! MAC: %s\n", devMacStr.c_str());
    phoneAuthorized = true;

    // Auto-persist the bonded phone's IRK so it's detected passively from now on
    if (dynamicIRKPresent)
    {
      persistNewIRK(dynamicBondedIRK);
    }

    NimBLEDevice::getScan()->stop();
    NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
    if (pAdv != nullptr)
      pAdv->stop();
  }

  uint32_t onPassKeyRequest() override
  {
    return BLE_PAIRING_PIN;
  }

  bool onConfirmPIN(uint32_t pin) override
  {
    return true;
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
          Serial.printf("\n=======================================================\n");
          Serial.printf("🎉 [SECURITY] Authorized Android Phone Detected!\n");
          Serial.printf("   MAC: %s | RSSI: %d dBm\n", devMacStr.c_str(), advertisedDevice->getRSSI());
          Serial.printf("=======================================================\n\n");
          phoneAuthorized = true;
          NimBLEDevice::getScan()->stop();
          NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
          if (pAdv != nullptr)
            pAdv->stop();
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
          Serial.printf("\n=======================================================\n");
          Serial.printf("🎉 [SECURITY] Authorized Phone IRK Resolved!\n");
          Serial.printf("   RPA: %s | RSSI: %d dBm\n", devMacStr.c_str(), advertisedDevice->getRSSI());
          Serial.printf("=======================================================\n\n");
          phoneAuthorized = true;
          NimBLEDevice::getScan()->stop();
          NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
          if (pAdv != nullptr)
            pAdv->stop();
          return;
        }
      }

      // 3. Check against NVS-persisted IRKs (auto-saved from previous pairings)
      if (resolveRPAAgainstPersistedIRKs(nativeAddr))
      {
        Serial.printf("\n=======================================================\n");
        Serial.printf("🎉 [SECURITY] Authorized Phone Persisted IRK Resolved!\n");
        Serial.printf("   RPA: %s | RSSI: %d dBm\n", devMacStr.c_str(), advertisedDevice->getRSSI());
        Serial.printf("=======================================================\n\n");
        phoneAuthorized = true;
        NimBLEDevice::getScan()->stop();
        NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
        if (pAdv != nullptr)
          pAdv->stop();
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
  if (phoneAuthorized)
  {
    Serial.println("[SECURITY] Authorization granted. BLE scan stopped.");
  }
  else
  {
    Serial.println("[SECURITY] Scan cycle ended. Restarting continuous background scan...");
    NimBLEScan *pBLEScan = NimBLEDevice::getScan();
    if (pBLEScan != nullptr && !phoneAuthorized)
      {
        pBLEScan->start(5, onScanEnded, false);
        bleScanning = true;
      }
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

    // Restore bonded phone IRK from NVS so passive RPA resolution works after reboot
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

    // Load auto-persisted IRKs from previous successful pairings
    loadPersistedIRKs();

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
    pAdvertising->setAppearance(0x0000);
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
