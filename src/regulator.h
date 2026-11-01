#pragma once

void regulatorTask(void *pvParameters);
void stopRegulatorTask();
void recoverI2CBus(int sdaPin, int sclPin);

