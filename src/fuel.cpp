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
  accumulated_inj_time_us = 0;
  accumulated_inj_pulses = 0;
  spd_delta_pulses = 0;
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

float getInjectorDeadTime(float voltage) {
  // Typical Bosch low-impedance injector dead-time (opening delay) vs voltage.
  // During dead-time the solenoid is energizing but the pintle hasn't opened,
  // so no fuel flows. Adjust these values if you bench-test your injectors.
  struct DTPoint {
    float voltage;
    float us;
  };
  static const DTPoint dt[] = {
      {10.0f, 1800.0f}, {11.0f, 1600.0f}, {12.0f, 1400.0f}, {13.0f, 1100.0f},
      {13.5f, 1000.0f}, {14.0f, 900.0f},  {15.0f, 750.0f},  {16.0f, 650.0f},
  };
  static const int N = sizeof(dt) / sizeof(dt[0]);

  if (voltage <= dt[0].voltage)
    return dt[0].us;
  if (voltage >= dt[N - 1].voltage)
    return dt[N - 1].us;

  for (int i = 0; i < N - 1; i++) {
    if (voltage >= dt[i].voltage && voltage <= dt[i + 1].voltage) {
      float x0 = dt[i].voltage, x1 = dt[i + 1].voltage;
      float y0 = dt[i].us, y1 = dt[i + 1].us;
      return y0 + (voltage - x0) * (y1 - y0) / (x1 - x0);
    }
  }
  return 1000.0f; // Fallback
}
