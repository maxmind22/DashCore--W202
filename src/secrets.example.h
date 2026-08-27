#pragma once

#include <stddef.h>
#include <stdint.h>

/**
 * ============================================================================
 * SECRETS TEMPLATE (EXAMPLE)
 * ============================================================================
 * Copy this file to "secrets.h" and enter your actual private values.
 * "secrets.h" is ignored by git so your private credentials stay secure.
 * ============================================================================
 */

// 1. BLE Advertising Name (Discrete/Inconspicuous)
#define BLE_DEVICE_NAME "MB-BT-Audio"

// 2. 6-Digit Passkey PIN for BLE Pairing / Bonding (000000..999999)
#define BLE_PAIRING_PIN 123456

// 3. Authorized Android Phone MAC Addresses (case-insensitive string list)
static const char *const BLE_AUTHORIZED_MACS[] = {
    "00:00:00:00:00:00" // Example: "aa:bb:cc:dd:ee:ff"
};
static const size_t NUM_AUTHORIZED_MACS = sizeof(BLE_AUTHORIZED_MACS) / sizeof(BLE_AUTHORIZED_MACS[0]);

// 4. Authorized iPhone Identity Resolving Keys (IRK) - 16 bytes each
// Extract using the IRK extractor helper in tools/get_irk/
static const uint8_t BLE_AUTHORIZED_IRKS[][16] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} // Replace with actual 16-byte IRK
};
static const size_t NUM_AUTHORIZED_IRKS = sizeof(BLE_AUTHORIZED_IRKS) / sizeof(BLE_AUTHORIZED_IRKS[0]);
