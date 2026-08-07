#pragma once
#include "config.h"
#include "globals.h"

void processPushStart();
void setRelays(bool acc, bool ign, bool start);
void setupPushStartPins();
void enterPowerDownSleep();
void startTVDisplay();
void stopTVDisplay();
void sleepCANController();
void wakeupCANController();

void triggerLockPulse();
void updateLockRelay();
void queueTone(int beeps, unsigned long onMs, unsigned long offMs);
void updateToneStateMachine();
bool isTonePlaying();
void playUnlockToggleTone(bool disabled);
void playLockdownToggleTone(bool lockdownActive);
void playAuthWarningTone();
void processUnlockSignals();
