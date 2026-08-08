#pragma once
#include "config.h"
#include "globals.h"

float getFuelPercent(float rawValue);
float getInjectorDeadTime(float voltage);
void resetFuelTripData(unsigned long now = 0);
