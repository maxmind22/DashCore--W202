#include "fuel.h"
#include "display.h"

float getFuelPercent(float rawValue) {
  if (rawValue <= calibrationTable[0].rawValue)
    return calibrationTable[0].percent;
  if (rawValue >= calibrationTable[NUM_CALIBRATION_POINTS - 1].rawValue)
    return calibrationTable[NUM_CALIBRATION_POINTS - 1].percent;

  for (int i = 0; i < NUM_CALIBRATION_POINTS - 1; i++) {
    if (rawValue >= calibrationTable[i].rawValue &&
        rawValue <= calibrationTable[i + 1].rawValue) {
      float x0 = calibrationTable[i].rawValue;
      float x1 = calibrationTable[i + 1].rawValue;
      float y0 = calibrationTable[i].percent;
      float y1 = calibrationTable[i + 1].percent;
      return y0 + (rawValue - x0) * (y1 - y0) / (x1 - x0);
    }
  }
  return 0.0f;
}

void resetFuelTripData(unsigned long now) {
  total_fuel_liters = 0.0f;
  total_distance_km = 0.0f;
  total_fuel_saved_liters = 0.0f;
  avg_l_100km = 0.0f;
  inst_val = 0.0f;

  Preferences prefs;
  prefs.begin("trip_data", false);
  prefs.putFloat("fuel", 0.0f);
  prefs.putFloat("dist", 0.0f);
  prefs.putFloat("saved", 0.0f);
  prefs.end();
  tv.setCursor(WARNING_X + 40, WARNING_Y + 50);
  tv.setTextColor(0xFF);
  tv.print("RESET DONE!");
  resetPrintTime = (now != 0) ? now : millis();
}
