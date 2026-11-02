#include "security.h"
#include "globals.h"
#include "pushstart.h"
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <esp_bt.h>
#include <esp_heap_caps.h>
#include <esp_task_wdt.h>
#include <mbedtls/aes.h>
#include <ctype.h>

#if __has_include("secrets.h")
#include "secrets.h"
#else
// Fallback defaults if secrets.h is not created yet (copy from secrets.example.h)
#define BLE_DEVICE_NAME "MB-BT-Audio"
#define BLE_PAIRING_PIN 123456
static const char *const BLE_AUTHORIZED_MACS[] = {
    "00:00:00:00:00:00"};
static const size_t NUM_AUTHORIZED_MACS = sizeof(BLE_AUTHORIZED_MACS) / sizeof(BLE_AUTHORIZED_MACS[0]);
static const uint8_t BLE_AUTHORIZED_IRKS[][16] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
static const size_t NUM_AUTHORIZED_IRKS = sizeof(BLE_AUTHORIZED_IRKS) / sizeof(BLE_AUTHORIZED_IRKS[0]);
#endif

#if defined(CONFIG_NIMBLE_CPP_IDF)
#include "host/ble_hs.h"
#include "host/ble_store.h"
#else
#include "nimble/nimble/host/include/host/ble_hs.h"
#include "nimble/nimble/host/include/host/ble_store.h"
#endif

#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID "87654321-4321-4321-4321-cba987654321"

#define DEBUG_PRINT_SECRETS 0 // Set 1 to print raw IRK keys and pairing PIN to Serial

// --- BLE teardown & WDT safety timing ---
static constexpr unsigned long BLE_PAIRING_SETTLE_MS   = 100; // Let Core 0 ble_sm_persist_keys() + LL ACK finish before disconnect
static constexpr unsigned long BLE_DISCONNECT_DRAIN_MS = 500; // Max non-blocking wait for LL_TERMINATE_IND completion
static constexpr unsigned long BLE_SYNC_DRAIN_MS       = 200; // Max synchronous disconnect wait (e.g. before deep sleep)
static constexpr unsigned long BLE_STOP_SETTLE_MS       = 60;  // Quiet window for controller to ACK disc_cancel/adv_stop & drain timers
static constexpr uint32_t      BLE_WDT_DEINIT_TIMEOUT_S = 3;  // > NimBLE's 2000ms ble_hs_stop timeout (Core 1 WDT is 1s)
static constexpr uint32_t      BLE_WDT_NORMAL_TIMEOUT_S = 1;  // Matches setup() esp_task_wdt_init(1, true)
static constexpr uint16_t      ESP32_MAX_BLE_CONN_HANDLES = 9; // ESP32 controller supports handles 0..8

// --- NVS-persisted IRK store (pair once → detected forever) ---
#define MAX_PERSISTED_IRKS 8
#define NVS_IRK_NAMESPACE  "ble_irks"

static uint8_t  persistedIRKs[MAX_PERSISTED_IRKS][16];
static uint8_t  persistedIRKCount = 0;

static bool bleInitialized = false;
static bool bleScanning = false;
static bool classicBtMemReleased = false;

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

static volatile bool authTonePending = false;
static volatile bool bleTeardownPending = false;
static volatile bool pairingBondSyncPending = false;

static bool teardownDrainActive = false;
static bool teardownDisconnectSent = false;
static unsigned long teardownStartMs = 0;
static unsigned long lastScanRefreshMs = 0;

static int secStoreIteratorCallback(int obj_type, union ble_store_value *val, void *cookie);

// Temporarily widen Task WDT only if the calling task (Core 1 loopTask) is currently subscribed.
// NimBLEDevice::deinit() / ble_hs_stop() can block up to 2000ms if a peer drops out of range,
// which would otherwise trip the 1s panic WDT configured in setup().
static bool extendLoopTaskWdtForBLE()
{
  if (esp_task_wdt_status(nullptr) == ESP_OK)
  {
    esp_task_wdt_reset();
    esp_task_wdt_init(BLE_WDT_DEINIT_TIMEOUT_S, true);
    esp_task_wdt_reset();
    return true;
  }
  return false;
}

static void restoreLoopTaskWdtAfterBLE(bool extended)
{
  if (extended)
  {
    esp_task_wdt_reset();
    esp_task_wdt_init(BLE_WDT_NORMAL_TIMEOUT_S, true);
    esp_task_wdt_reset();
  }
}

static void stopScanAndAdvertising()
{
  if (!bleInitialized && !NimBLEDevice::getInitialized())
  {
    bleScanning = false;
    return;
  }

  NimBLEScan *pBLEScan = NimBLEDevice::getScan();
  if (pBLEScan != nullptr && pBLEScan->isScanning())
  {
    pBLEScan->stop();
    pBLEScan->clearResults();
  }
  bleScanning = false;

  NimBLEAdvertising *pAdv = NimBLEDevice::getAdvertising();
  if (pAdv != nullptr && pAdv->isAdvertising())
  {
    pAdv->stop();
  }
}

// Thread-safe via NimBLE's internal ble_hs_lock(); avoids copying NimBLEServer's unguarded std::vector across cores.
static size_t terminateAndCountOpenConnections(bool initiateTerminate)
{
  if (!NimBLEDevice::getInitialized())
    return 0;

  size_t openCount = 0;
  for (uint16_t handle = 0; handle < ESP32_MAX_BLE_CONN_HANDLES; handle++)
  {
    if (ble_gap_conn_find(handle, nullptr) == 0)
    {
      openCount++;
      if (initiateTerminate)
      {
        ble_gap_terminate(handle, BLE_ERR_REM_USER_CONN_TERM);
      }
    }
  }
  return openCount;
}

// Centralized authorization helper
static void grantPhoneAuthorization(const char *reason, const char *deviceInfo, bool fromPairing = false)
{
  if (phoneAuthorized || isPhoneAuthorized())
    return;

  if (fromPairing)
  {
    pairingBondSyncPending = true;
  }
  phoneAuthorized = true;
  authTonePending = true;
  bleTeardownPending = true;

  Serial.printf("\n=======================================================\n");
  Serial.printf("🎉 [SECURITY] Authorization Granted: %s\n", reason ? reason : "Phone Authorized");
  if (deviceInfo != nullptr && deviceInfo[0] != '\0')
  {
    Serial.printf("   Device: %s\n", deviceInfo);
  }
  Serial.printf("=======================================================\n\n");
}

void processBLEEvents()
{
  if (authTonePending)
  {
    authTonePending = false;
    playAuthSuccessTone();
  }

  if (bleTeardownPending)
  {
    unsigned long nowMs = millis();

    if (!teardownDrainActive)
    {
      teardownDrainActive = true;
      teardownStartMs = nowMs;
      teardownDisconnectSent = false;

      // Immediately stop active scanning and connectable advertising
      stopScanAndAdvertising();

      if (!pairingBondSyncPending)
      {
        terminateAndCountOpenConnections(true);
        teardownDisconnectSent = true;
      }
    }

    // After SMP pairing, ble_gap_enc_event() fires right BEFORE ble_sm_persist_keys() on Core 0.
    // Give Core 0 and the Link Layer a brief non-blocking window to commit the bond and ACK the
    // final SMP PDU before extracting the new IRK and disconnecting the phone.
    if (pairingBondSyncPending)
    {
      if (nowMs - teardownStartMs < BLE_PAIRING_SETTLE_MS)
      {
        return;
      }
      pairingBondSyncPending = false;
      if (NimBLEDevice::getInitialized())
      {
        bool irkFound = false;
        ble_store_iterate(BLE_STORE_OBJ_TYPE_PEER_SEC, secStoreIteratorCallback, &irkFound);
      }
      teardownStartMs = nowMs;
      terminateAndCountOpenConnections(true);
      teardownDisconnectSent = true;
    }

    if (!teardownDisconnectSent)
    {
      terminateAndCountOpenConnections(true);
      teardownDisconnectSent = true;
    }

    // Drain any open GATT connection non-blockingly across loop() frames so video rendering
    // stays uninterrupted and ble_hs_stop() sees 0 open connections on deinit.
    if (terminateAndCountOpenConnections(false) > 0 &&
        (nowMs - teardownStartMs < BLE_DISCONNECT_DRAIN_MS))
    {
      return;
    }

    // Ensure a minimum quiet window after stopping scan/adv so controller ACKs and timers settle
    if (nowMs - teardownStartMs < BLE_STOP_SETTLE_MS)
    {
      return;
    }

    teardownBLESecurity();
    Serial.printf("[SECURITY] Bluetooth stack deinitialized & radio powered off (free internal heap: %u B).\n\n",
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    return;
  }

  // Periodically restart the continuous scan. With maxResults = 0 NimBLE still keeps an entry
  // for ADV_IND advertisers while it waits for their scan response; entries whose response is
  // never received would otherwise linger forever. stop() flushes them (maxResults == 0).
  if (bleScanning && !phoneAuthorized && !isPhoneAuthorized())
  {
    unsigned long nowMs = millis();
    if (nowMs - lastScanRefreshMs >= BLE_SCAN_REFRESH_MS)
    {
      lastScanRefreshMs = nowMs;
      NimBLEScan *pBLEScan = NimBLEDevice::getScan();
      if (pBLEScan != nullptr && pBLEScan->isScanning())
      {
        pBLEScan->stop(); // Also clears stored results because maxResults == 0
        pBLEScan->clearResults();
        if (!pBLEScan->start(0, nullptr, true)) // is_continue: results already cleared while stopped
        {
          Serial.println("[SECURITY] ERROR: BLE scan refresh failed to restart!");
          bleScanning = false;
        }
      }
    }
  }
}


// Load all persisted IRKs from NVS into runtime array
static void loadPersistedIRKs()
{
  Preferences prefs;
  prefs.begin(NVS_IRK_NAMESPACE, false); // read-write so namespace is created if absent
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
#if DEBUG_PRINT_SECRETS
      Serial.printf("[SECURITY] Loaded persisted IRK #%u: { ", i);
      for (int k = 0; k < 16; k++)
        Serial.printf("0x%02X%s", persistedIRKs[i][k], (k == 15) ? "" : ", ");
      Serial.println(" }");
#else
      Serial.printf("[SECURITY] Loaded persisted IRK #%u: [REDACTED]\n", i);
#endif
    }
  }
  prefs.end();

  if (persistedIRKCount > 0)
    Serial.printf("[SECURITY] %u dynamic IRK(s) loaded from NVS.\n", persistedIRKCount);
  else if (NUM_AUTHORIZED_IRKS > 0)
    Serial.printf("[SECURITY] %u compile-time IRK(s) configured in secrets.h.\n", (unsigned int)NUM_AUTHORIZED_IRKS);
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

#if DEBUG_PRINT_SECRETS
  Serial.printf("\n🔐 [SECURITY] NEW IRK auto-persisted to NVS (slot #%u):\n", persistedIRKCount - 1);
  Serial.print("   { ");
  for (int i = 0; i < 16; i++)
  {
    Serial.printf("0x%02X%s", irk[i], (i == 15) ? "" : ", ");
    if (i == 7)
      Serial.print("\n     ");
  }
  Serial.println(" }");
#else
  Serial.printf("\n🔐 [SECURITY] NEW IRK auto-persisted to NVS (slot #%u): [REDACTED]\n", persistedIRKCount - 1);
#endif
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
#if DEBUG_PRINT_SECRETS
    Serial.println("\n🔑 [SECURITY] Active Bonded Phone IRK extracted from Bond Store:");
    Serial.print("   { ");
    for (int i = 0; i < 16; i++)
    {
      Serial.printf("0x%02X%s", val->sec.irk[i], (i == 15) ? "" : ", ");
      if (i == 7)
        Serial.print("\n     ");
    }
    Serial.println(" }\n");
#else
    Serial.println("\n🔑 [SECURITY] Active Bonded Phone IRK extracted from Bond Store: [REDACTED]\n");
#endif
    bool *found = (bool *)cookie;
    if (found)
      *found = true;
  }
  return 0;
}

static void formatBleAddr(const uint8_t addr[6], char out[18])
{
  snprintf(out, 18, "%02x:%02x:%02x:%02x:%02x:%02x",
           addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
}

static bool isZeroBleAddr(const uint8_t addr[6])
{
  return (addr[0] | addr[1] | addr[2] | addr[3] | addr[4] | addr[5]) == 0;
}

// Check if a given peer address is recognized (via MAC whitelist or IRK RPA resolution)
static bool isAddressAuthorized(const uint8_t rawAddr[6], const char *macStr, char *matchedDesc, size_t descSize)
{
  if (macStr != nullptr && macStr[0] != '\0')
  {
    // 1. Check MAC whitelist
    for (size_t i = 0; i < NUM_AUTHORIZED_MACS; i++)
    {
      if (BLE_AUTHORIZED_MACS[i] && strlen(BLE_AUTHORIZED_MACS[i]) >= 12)
      {
        if (matchesMacString(macStr, BLE_AUTHORIZED_MACS[i]))
        {
          snprintf(matchedDesc, descSize, "MAC Whitelist: %s", macStr);
          return true;
        }
      }
    }
  }

  if (rawAddr != nullptr)
  {
    // 2. Check compile-time IRKs from secrets.h
    for (size_t i = 0; i < NUM_AUTHORIZED_IRKS; i++)
    {
      if (resolveRPA(rawAddr, BLE_AUTHORIZED_IRKS[i]))
      {
        snprintf(matchedDesc, descSize, "Authorized IRK: %s", macStr ? macStr : "");
        return true;
      }
    }

    // 3. Check NVS-persisted / bonded IRKs
    if (resolveRPAAgainstPersistedIRKs(rawAddr))
    {
      snprintf(matchedDesc, descSize, "Persisted IRK: %s", macStr ? macStr : "");
      return true;
    }
  }

  return false;
}

class SecurityServerCallbacks : public NimBLEServerCallbacks
{
  void onConnect(NimBLEServer *pServer, ble_gap_conn_desc *desc) override
  {
    if (desc == nullptr)
      return;

    if (phoneAuthorized || isPhoneAuthorized() || bleTeardownPending)
    {
      pServer->disconnect(desc->conn_handle);
      return;
    }

    char devMacStr[18];
    char idMacStr[18];
    formatBleAddr(desc->peer_ota_addr.val, devMacStr);
    formatBleAddr(desc->peer_id_addr.val, idMacStr);
    Serial.printf("[SECURITY] Connection received from OTA: %s | ID: %s\n", devMacStr, idMacStr);

    // 1. If this phone is already known (in MAC whitelist or matches authorized IRK), authorize IMMEDIATELY!
    char matchDesc[48];
    if (isAddressAuthorized(desc->peer_ota_addr.val, devMacStr, matchDesc, sizeof(matchDesc)) ||
        (!isZeroBleAddr(desc->peer_id_addr.val) &&
         isAddressAuthorized(desc->peer_id_addr.val, idMacStr, matchDesc, sizeof(matchDesc))))
    {
      grantPhoneAuthorization("Authorized Phone Connected & Recognized!", matchDesc);
      // Disconnect immediately so Android does NOT attempt to pair or create a bond!
      pServer->disconnect(desc->conn_handle);
      return;
    }

    // 2. Unrecognized phone — allow connection to proceed for PIN-based pairing
    Serial.printf("[SECURITY] Unrecognized phone %s — awaiting PIN authentication.\n", devMacStr);
  }

  void onAuthenticationComplete(ble_gap_conn_desc *desc) override
  {
    if (desc == nullptr)
      return;

    if (phoneAuthorized || isPhoneAuthorized() || bleTeardownPending)
    {
      NimBLEDevice::getServer()->disconnect(desc->conn_handle);
      return;
    }

    char devMacStr[18];
    char idMacStr[18];
    formatBleAddr(desc->peer_ota_addr.val, devMacStr);
    formatBleAddr(desc->peer_id_addr.val, idMacStr);

    if (!desc->sec_state.encrypted || !desc->sec_state.bonded)
    {
      Serial.printf("[SECURITY] Pairing / auth FAILED for %s (encrypted=%d, bonded=%d) — disconnecting.\n",
                    devMacStr, desc->sec_state.encrypted, desc->sec_state.bonded);
      NimBLEDevice::getServer()->disconnect(desc->conn_handle);
      return;
    }

    Serial.printf("[SECURITY] Pairing / authentication successful for %s (bonded + encrypted)\n", devMacStr);

    char matchDesc[48];
    if (isAddressAuthorized(desc->peer_ota_addr.val, devMacStr, matchDesc, sizeof(matchDesc)) ||
        (!isZeroBleAddr(desc->peer_id_addr.val) &&
         isAddressAuthorized(desc->peer_id_addr.val, idMacStr, matchDesc, sizeof(matchDesc))))
    {
      grantPhoneAuthorization("Authorized Phone Connected & Authenticated!", matchDesc, true);
      return;
    }

    // Authenticated with pairing PIN passkey — processBLEEvents() will sync the newly committed
    // bond IRK to NVS once Core 0's ble_sm_persist_keys() finishes, then disconnect & deinit BLE.
    grantPhoneAuthorization("Phone Authenticated via PIN Passkey!", devMacStr, true);
  }

  uint32_t onPassKeyRequest() override
  {
#if DEBUG_PRINT_SECRETS
    Serial.printf("[SECURITY] Passkey requested by phone — providing PIN: %06u\n", (unsigned int)BLE_PAIRING_PIN);
#else
    Serial.println("[SECURITY] Passkey requested by phone — providing PIN: [REDACTED]");
#endif
    return BLE_PAIRING_PIN;
  }

  bool onConfirmPIN(uint32_t pin) override
  {
    bool match = (pin == BLE_PAIRING_PIN);
#if DEBUG_PRINT_SECRETS
    Serial.printf("[SECURITY] Numeric comparison PIN: %06u — %s\n",
                  (unsigned int)pin, match ? "ACCEPTED" : "REJECTED");
#else
    Serial.printf("[SECURITY] Numeric comparison PIN: [REDACTED] — %s\n", match ? "ACCEPTED" : "REJECTED");
#endif
    return match;
  }

  void onDisconnect(NimBLEServer *pServer) override
  {
    (void)pServer;
    if (!phoneAuthorized && !isPhoneAuthorized() && !bleTeardownPending && !teardownDrainActive)
    {
      NimBLEDevice::startAdvertising();
    }
  }
};

class SecurityScanCallbacks : public NimBLEAdvertisedDeviceCallbacks
{
  void onResult(NimBLEAdvertisedDevice *advertisedDevice) override
  {
    if (phoneAuthorized || isPhoneAuthorized() || bleTeardownPending || advertisedDevice == nullptr)
      return;

    // Optional RSSI threshold check FIRST (before formatting address or running AES-128)
#if defined(BLE_MIN_RSSI) && (BLE_MIN_RSSI > -128)
    int rssi = advertisedDevice->getRSSI();
    if (rssi < BLE_MIN_RSSI)
    {
      return;
    }
#else
    int rssi = advertisedDevice->getRSSI();
#endif

    const uint8_t *nativeAddr = advertisedDevice->getAddress().getNative();
    char devMacStr[18];
    formatBleAddr(nativeAddr, devMacStr);

    char matchDesc[48];
    if (isAddressAuthorized(nativeAddr, devMacStr, matchDesc, sizeof(matchDesc)))
    {
      char info[80];
      snprintf(info, sizeof(info), "%s | RSSI: %d dBm", matchDesc, rssi);
      grantPhoneAuthorization("Authorized Phone Detected via BLE Scan!", info);
      return;
    }
  }
};

static SecurityServerCallbacks serverCallbacks;
static SecurityScanCallbacks scanCallbacks;

// NimBLE 1.4.x defaults to maxResults = 0xFF, which stores EVERY unique advertiser address it
// ever hears as a heap-allocated NimBLEAdvertisedDevice (+ payload vector). A continuous scan
// (duration 0) never flushes them, and phones/watches/tags rotate random addresses, so the list
// grows without bound until the internal heap (already tight next to the composite video
// framebuffers) is exhausted and the ESP32 aborts/reboots. Only the onResult() callback is
// needed, so store nothing.
static void configureScan(NimBLEScan *pBLEScan)
{
  pBLEScan->setAdvertisedDeviceCallbacks(&scanCallbacks, true); // wantDuplicates = true for continuous reception
  pBLEScan->setMaxResults(0);                                   // Callbacks only: never accumulate scan results
  pBLEScan->setActiveScan(true);                                // Active scan catches Android discovery/scan responses
  pBLEScan->setInterval(100);                                   // 100ms interval
  pBLEScan->setWindow(50);                                      // 50ms window (balanced 50% scan / 50% advertising duty cycle)
}

void setupBLESecurity()
{
  if (!classicBtMemReleased && esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE)
  {
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    classicBtMemReleased = true;
  }

  if (isPhoneAuthorized())
    return;

  bleTeardownPending = false;
  teardownDrainActive = false;
  teardownDisconnectSent = false;

  if (!bleInitialized)
  {
    NimBLEDevice::setScanDuplicateCacheSize(10); // Minimum controller duplicate cache (saves internal DRAM)
    bool wdtExtended = extendLoopTaskWdtForBLE();
    NimBLEDevice::init(BLE_DEVICE_NAME);
    restoreLoopTaskWdtAfterBLE(wdtExtended);

    NimBLEDevice::setSecurityAuth(true, true, false);       // Bonding + MITM enforced, SC off for compatibility
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY); // ESP32 "displays" PIN, phone must type it

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
    pServer->advertiseOnDisconnect(false);          // Prevent NimBLEServer from auto-restarting adv during auth teardown
    pServer->setCallbacks(&serverCallbacks, false); // deleteCallbacks = false: serverCallbacks is static BSS

    NimBLEService *pService = pServer->createService(SERVICE_UUID);
    NimBLECharacteristic *pChar = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::READ);
    pChar->setValue("DashCore Auth");
    pService->start();

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
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
    configureScan(pBLEScan);

    Serial.println("[SECURITY] Starting continuous BLE scan & server (until authorized or sleep)...");
    bleScanning = true;
    lastScanRefreshMs = millis();
    if (!pBLEScan->start(0, nullptr, false))
    {
      Serial.println("[SECURITY] ERROR: BLE Scan failed to start!");
      bleScanning = false;
    }
  }
}

void teardownBLESecurity()
{
  bleTeardownPending = false;
  teardownDrainActive = false;
  teardownDisconnectSent = false;

  const bool nimbleUp = NimBLEDevice::getInitialized();
  const esp_bt_controller_status_t btStatus = esp_bt_controller_get_status();

  if (bleInitialized || nimbleUp || btStatus != ESP_BT_CONTROLLER_STATUS_IDLE)
  {
    if (nimbleUp)
    {
      if (pairingBondSyncPending)
      {
        pairingBondSyncPending = false;
        bool irkFound = false;
        ble_store_iterate(BLE_STORE_OBJ_TYPE_PEER_SEC, secStoreIteratorCallback, &irkFound);
      }

      stopScanAndAdvertising();

      // Give controller and host stack a quiet window to complete cancellation of
      // active discovery and advertising, and let the timer task drain pending callouts.
      unsigned long stopSettleStart = millis();
      while (millis() - stopSettleStart < BLE_STOP_SETTLE_MS)
      {
        if (esp_task_wdt_status(nullptr) == ESP_OK)
        {
          esp_task_wdt_reset();
        }
        delay(5);
      }

      // If called synchronously while a link is still active (e.g. enterPowerDownSleep),
      // drain the disconnect cleanly before ble_hs_stop().
      if (terminateAndCountOpenConnections(true) > 0)
      {
        unsigned long drainStartMs = millis();
        while (terminateAndCountOpenConnections(false) > 0 &&
               (millis() - drainStartMs < BLE_SYNC_DRAIN_MS))
        {
          if (esp_task_wdt_status(nullptr) == ESP_OK)
          {
            esp_task_wdt_reset();
          }
          delay(5);
        }
      }
    }
    else
    {
      pairingBondSyncPending = false;
    }

    bool wdtExtended = extendLoopTaskWdtForBLE();
    if (nimbleUp)
    {
      NimBLEDevice::deinit(false);
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED)
    {
      esp_bt_controller_disable();
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED)
    {
      esp_bt_controller_deinit();
    }
    restoreLoopTaskWdtAfterBLE(wdtExtended);

    bleInitialized = false;
    bleScanning = false;
  }
  else
  {
    pairingBondSyncPending = false;
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
  if (bypass)
  {
    if (bleInitialized || NimBLEDevice::getInitialized() ||
        esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_IDLE)
    {
      bleTeardownPending = true;
    }
  }
  else if (!phoneAuthorized)
  {
    bleTeardownPending = false;
    teardownDrainActive = false;
    teardownDisconnectSent = false;
    if (!bleInitialized)
    {
      setupBLESecurity();
    }
    else if (!isBLEScanning())
    {
      triggerBLERescan();
    }
  }
}

void triggerBLERescan()
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
    configureScan(pBLEScan);

    Serial.println("[SECURITY] Re-starting continuous BLE scan & server...");
    bleScanning = true;
    lastScanRefreshMs = millis();
    if (!pBLEScan->start(0, nullptr, false))
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

