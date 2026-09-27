#pragma once

#include "config.h"
#include "globals.h"

// Initialize BLE scanner and start initial scan window (e.g., on boot/wake)
void setupBLESecurity();

// Gracefully stop scan and completely power down BLE radio to avoid interference
void teardownBLESecurity();

// Returns true if phone has been authenticated or if emergency bypass is active
bool isPhoneAuthorized();

// Emergency bypass control (e.g. from 6-pulse central unlock sequence)
void setPhoneAuthBypass(bool bypass);

// Trigger on-demand quick BLE scan (e.g. on start button press if phone not yet found)
void triggerBLERescan(unsigned long durationMs = BLE_RESCAN_TIMEOUT_MS);

// Check if a BLE scan is currently active
bool isBLEScanning();

// Process any deferred BLE events on Core 1 (audio tones, radio shutdown)
void processBLEEvents();

