#pragma once
#include "config.h"
#include "globals.h"

void processPushStart(unsigned long now = 0);
void setRelays(bool acc, bool ign, bool start);
void setupPushStartPins();
void enterPowerDownSleep();
void startTVDisplay();
void stopTVDisplay();
void sleepCANController();
void wakeupCANController();

void triggerLockPulse(unsigned long now = 0);
void updateLockRelay(unsigned long now = 0);
void queueTone(int beeps, unsigned long onMs, unsigned long offMs, unsigned long now = 0);
void updateToneStateMachine(unsigned long now = 0);
bool isTonePlaying();
void playUnlockToggleTone(bool disabled);
void playStartStopToggleTone(bool disabled);
void playAuthWarningTone();
void processUnlockSignals(unsigned long now = 0);
