#pragma once
#include "config.h"
#include "globals.h"

// TWAI (ESP32 internal CAN controller) lifecycle — external transceiver: SN65HVD230
bool canBegin();          // Install + start TWAI driver @ 500 kbps (idempotent)
void canEnd();            // Stop + uninstall driver, park CTX pin recessive (before deep sleep)
bool canIsRunning();

// Dedicated Core 0 CAN RX + recovery task (isolates TWAI interrupts & parsing from Core 1 video V-Sync
// and delivers 50 Hz 0x02 updates directly to Core 0 regulatorTask with microsecond latency)
void startCanRxTask();
void stopCanRxTask();

void checkCanErrors(unsigned long now);
void drainCanRxBuffer(unsigned long now);
void sendCanHealthFrame(unsigned long now);
