#pragma once
#include "config.h"
#include "globals.h"

// TWAI (ESP32 internal CAN controller) lifecycle — external transceiver: SN65HVD230
bool canBegin();          // Install + start TWAI driver @ 500 kbps (idempotent)
void canEnd();            // Stop + uninstall driver, park CTX pin recessive (before deep sleep)
bool canIsRunning();

void checkCanErrors(unsigned long now);
void drainCanRxBuffer(unsigned long now);
void sendCanHealthFrame(unsigned long now);
