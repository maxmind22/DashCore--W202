#include "fuel.h"
#include "config.h"
#include "globals.h"
#include <ESP_8_BIT_GFX.h>
#include <Preferences.h>

struct CalibrationPoint {
  int16_t rawValue;
  float percent;
};

static constexpr CalibrationPoint calibrationTable[] = {
    {1326, 0.0f},   {3383, 4.9f},   {5440, 10.5f},  {7498, 16.8f},
    {9555, 23.9f},  {11613, 35.4f}, {13670, 50.2f}, {15727, 64.4f},
    {17785, 78.0f}, {19842, 90.5f}, {21900, 100.0f},
};
static constexpr int NUM_CALIBRATION_POINTS =
    sizeof(calibrationTable) / sizeof(calibrationTable[0]);

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
  total_fuel_liters = 0.0;
  total_distance_km = 0.0;
  total_fuel_saved_liters = 0.0;
  session_fuel_liters = 0.0;
  session_distance_km = 0.0;
  portENTER_CRITICAL(&dataMux);
  accumulated_inj_time_us = 0;
  accumulated_inj_pulses = 0;
  spd_delta_pulses = 0;
  accumulated_dfco_saved_us = 0.0f;
  accumulated_auto_stop_ms = 0;
  accumulated_eco_crank_net_us = 0.0f;
  portEXIT_CRITICAL(&dataMux);

  saveTripDataToNVS();
  tv.setCursor(WARNING_X + 40, WARNING_Y + 50);
  tv.setTextColor(0xFF);
  tv.print("RESET DONE!");
  resetPrintTime = (now != 0) ? now : millis();
}

// Trip totals are stored as double under new keys ("*_d"). The legacy float keys
// ("fuel", "dist", "saved") are only read once to migrate existing data.
static double sanitizeTripValue(double v) {
  return (isnan(v) || isinf(v) || v < 0.0) ? 0.0 : v;
}

void loadTripDataFromNVS() {
  Preferences prefs;
  prefs.begin("trip_data", true); // Open in read-only mode
  if (prefs.isKey("fuel_d")) {
    total_fuel_liters = prefs.getDouble("fuel_d", 0.0);
    total_distance_km = prefs.getDouble("dist_d", 0.0);
    total_fuel_saved_liters = prefs.getDouble("saved_d", 0.0);
  } else {
    total_fuel_liters = prefs.getFloat("fuel", 0.0f);
    total_distance_km = prefs.getFloat("dist", 0.0f);
    total_fuel_saved_liters = prefs.getFloat("saved", 0.0f);
  }
  compounded_r_int = prefs.getFloat("r_int", 0.0f);
  last_known_avg_l_100km = prefs.getFloat("avg_d", DEFAULT_CONSUMPTION_L_100KM);
  measured_idle_l_per_hr = prefs.getFloat("idle_lh", BASELINE_IDLE_FUEL_L_PER_HR);
  float saved_dfco_us = prefs.getFloat("dfco_us", 1500.0f);
  float saved_dfco_map = prefs.getFloat("dfco_map", 0.0f);
  bool has_cl_norm = prefs.isKey("cl_norm_psi");
  float saved_cl_us_psi = has_cl_norm
                              ? prefs.getFloat("cl_norm_psi", BASELINE_CL_INJ_US_PER_PSI)
                              : BASELINE_CL_INJ_US_PER_PSI;
  starter_base_r_mohm = prefs.getFloat("str_r_m", 0.0f);
  starter_base_lat_ms = prefs.getFloat("str_lat", 40.0f);
  starter_base_samples = prefs.getUShort("str_cnt", 0);
  starter_total_cranks = prefs.getUInt("str_tot", 0);
  starter_degrade_strikes = prefs.getUChar("str_strk", 0);
  uint8_t saved_str_flt = prefs.getUChar("str_flt", STARTER_OK);
  prefs.end();

  total_fuel_liters = sanitizeTripValue(total_fuel_liters);
  total_distance_km = sanitizeTripValue(total_distance_km);
  total_fuel_saved_liters = sanitizeTripValue(total_fuel_saved_liters);
  if (isnan(compounded_r_int) || compounded_r_int < 0.0f)
    compounded_r_int = 0.0f;
  if (isnan(starter_base_r_mohm) || isinf(starter_base_r_mohm) ||
      starter_base_r_mohm < STARTER_R_MIN_VALID_MOHM ||
      starter_base_r_mohm > STARTER_R_MAX_VALID_MOHM) {
    starter_base_r_mohm = 0.0f;
    starter_base_samples = 0;
  }
  if (isnan(starter_base_lat_ms) || isinf(starter_base_lat_ms) ||
      starter_base_lat_ms <= 0.0f || starter_base_lat_ms > 500.0f) {
    starter_base_lat_ms = 40.0f;
  }
  starter_fault_code = (saved_str_flt <= STARTER_FAULT_SOLENOID) ? saved_str_flt : STARTER_OK;
  starter_healthy_streak = 0;
  if (isnan(last_known_avg_l_100km) || last_known_avg_l_100km < 3.0f ||
      last_known_avg_l_100km > 30.0f)
    last_known_avg_l_100km = DEFAULT_CONSUMPTION_L_100KM;
  if (isnan(measured_idle_l_per_hr) || isinf(measured_idle_l_per_hr) ||
      measured_idle_l_per_hr < 0.3f || measured_idle_l_per_hr > 3.0f)
    measured_idle_l_per_hr = BASELINE_IDLE_FUEL_L_PER_HR;
  if (isnan(saved_dfco_us) || isinf(saved_dfco_us) ||
      saved_dfco_us < 500.0f || saved_dfco_us > 10000.0f) {
    saved_dfco_us = 1500.0f;
    saved_dfco_map = 0.0f;
  }
  if (isnan(saved_dfco_map) || isinf(saved_dfco_map) ||
      saved_dfco_map < 1.0f || saved_dfco_map > 16.5f) {
    saved_dfco_map = 0.0f;
  }
  bool saved_cl_learned =
      has_cl_norm && !isnan(saved_cl_us_psi) && !isinf(saved_cl_us_psi) &&
      saved_cl_us_psi >= CL_INJ_US_PER_PSI_MIN &&
      saved_cl_us_psi <= CL_INJ_US_PER_PSI_MAX;
  if (!saved_cl_learned) {
    saved_cl_us_psi = BASELINE_CL_INJ_US_PER_PSI;
  }
  portENTER_CRITICAL(&dataMux);
  last_active_inj_pulse_us = saved_dfco_us;
  last_active_dfco_map_psi = saved_dfco_map;
  cl_inj_us_per_psi = saved_cl_us_psi;
  live_inj_us_per_psi = saved_cl_us_psi;
  cl_baseline_learned = saved_cl_learned;
  portEXIT_CRITICAL(&dataMux);
}

void saveTripDataToNVS() {
  portENTER_CRITICAL(&dataMux);
  float dfco_us = last_active_inj_pulse_us;
  float dfco_map = last_active_dfco_map_psi;
  float cl_us_psi = cl_inj_us_per_psi;
  bool cl_learned = cl_baseline_learned;
  portEXIT_CRITICAL(&dataMux);

  Preferences prefs;
  prefs.begin("trip_data", false);
  prefs.putDouble("fuel_d", total_fuel_liters);
  prefs.putDouble("dist_d", total_distance_km);
  prefs.putDouble("saved_d", total_fuel_saved_liters);
  prefs.putFloat("r_int", compounded_r_int);
  prefs.putFloat("avg_d", last_known_avg_l_100km);
  prefs.putFloat("idle_lh", measured_idle_l_per_hr);
  prefs.putFloat("dfco_us", dfco_us);
  prefs.putFloat("dfco_map", dfco_map);
  if (cl_learned) {
    prefs.putFloat("cl_norm_psi", cl_us_psi);
  }
  prefs.putFloat("str_r_m", starter_base_r_mohm);
  prefs.putFloat("str_lat", starter_base_lat_ms);
  prefs.putUShort("str_cnt", starter_base_samples);
  prefs.putUInt("str_tot", starter_total_cranks);
  prefs.putUChar("str_strk", starter_degrade_strikes);
  prefs.putUChar("str_flt", (starter_fault_code <= STARTER_FAULT_SOLENOID) ? starter_fault_code : STARTER_OK);
  prefs.end();
}

float getInjectorDeadTime(float voltage) {
  // Typical Bosch low-impedance injector dead-time (opening delay) vs voltage.
  // During dead-time the solenoid is energizing but the pintle hasn't opened,
  // so no fuel flows. Adjust these values if you bench-test your injectors.
  struct DTPoint {
    float voltage;
    float us;
  };
  static constexpr DTPoint dt[] = {
      {10.0f, 1800.0f}, {11.0f, 1600.0f}, {12.0f, 1400.0f}, {13.0f, 1100.0f},
      {13.5f, 1000.0f}, {14.0f, 900.0f},  {15.0f, 750.0f},  {16.0f, 650.0f},
  };
  static constexpr int N = sizeof(dt) / sizeof(dt[0]);

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
