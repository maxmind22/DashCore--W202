#pragma once
#include "config.h"
#include "globals.h"

void regulatorTask(void *pvParameters);
void recoverI2CBus(int sdaPin, int sclPin);
