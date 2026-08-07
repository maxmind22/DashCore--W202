#pragma once
#include "config.h"
#include "globals.h"

void drawStaticGauge();
void warnings(int percent, int temp_out, int spd, int coolant_level, int oil_level, unsigned long now);
