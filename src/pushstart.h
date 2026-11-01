#pragma once

void processPushStart(unsigned long now = 0);
void setupPushStartPins(bool keepRunningRelays = false);
bool isTonePlaying();
void playAuthSuccessTone();
