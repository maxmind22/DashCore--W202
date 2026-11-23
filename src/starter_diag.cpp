#include "starter_diag.h"
#include "globals.h"
#include <Arduino.h>

static void recordStarterStrike(uint8_t fault)
{
  starter_healthy_streak = 0;
  if (starter_degrade_strikes < 255)
  {
    starter_degrade_strikes++;
  }
  if (starter_degrade_strikes >= STARTER_FAULT_STRIKES)
  {
    starter_fault_code = fault;
  }
}

void evaluateStarterHealth(const StarterCrankResult &res, unsigned long now)
{
  if (res.outcome == CRANK_OUTCOME_NONE || res.outcome == CRANK_OUTCOME_ABORT)
  {
    return;
  }

  if (res.sensor_err)
  {
    Serial.println("[STARTER] skipped: ADC sensor error");
    return;
  }

  starter_total_cranks++;

  if (res.outcome == CRANK_OUTCOME_TIMEOUT)
  {
    if (!res.engaged)
    {
      activeStarterAlert = STARTER_FAULT_NO_CRANK;
      starterAlertStartMs = now;
      starterAlertUntilMs = now + STARTER_ALERT_HOLD_MS;
    }
    else if (res.peak_starter_a < STARTER_SLIP_MAX_PEAK_A &&
             (!res.valid_r_eff || res.steady_i_a <= STARTER_SLIP_MAX_STEADY_A))
    {
      activeStarterAlert = STARTER_FAULT_SLIP;
      starterAlertStartMs = now;
      starterAlertUntilMs = now + STARTER_ALERT_HOLD_MS;
    }
  }
  else if (res.outcome == CRANK_OUTCOME_SUCCESS)
  {
    if (activeStarterAlert == STARTER_FAULT_NO_CRANK || activeStarterAlert == STARTER_FAULT_SLIP)
    {
      activeStarterAlert = STARTER_OK;
    }

    bool battery_ok = (res.min_crank_v >= STARTER_MIN_VALID_V) &&
                      (compounded_r_int <= STARTER_MAX_BAT_R_INT_MOHM);
    bool is_warm = res.env_fresh && (res.coolant_temp_c >= STARTER_WARM_TEMP_C);
    bool base_learned = (starter_base_samples >= STARTER_LEARN_MIN_STARTS) &&
                        (starter_base_r_mohm >= STARTER_R_MIN_VALID_MOHM) &&
                        (starter_base_r_mohm <= STARTER_R_MAX_VALID_MOHM);

    float lat_thresh_f = base_learned
                             ? max((float)STARTER_LATENCY_WARN_MS, starter_base_lat_ms + STARTER_LATENCY_DELTA_MS)
                             : (float)STARTER_LATENCY_UNLEARNED_MS;
    bool solenoid_abnormal = res.engaged && battery_ok &&
                             (((float)res.engage_latency_ms >= lat_thresh_f) ||
                              (res.dropout_count >= STARTER_DROPOUT_WARN_COUNT));

    if (is_warm && res.valid_r_eff && battery_ok)
    {
      bool r_plausible = (res.r_eff_mohm >= STARTER_R_MIN_VALID_MOHM &&
                          res.r_eff_mohm <= STARTER_R_MAX_VALID_MOHM);

      if (!base_learned)
      {
        if (res.r_eff_mohm >= STARTER_LEARN_R_MIN_MOHM &&
            res.r_eff_mohm <= STARTER_LEARN_R_MAX_MOHM &&
            !solenoid_abnormal)
        {
          if (starter_base_samples == 0)
          {
            starter_base_r_mohm = res.r_eff_mohm;
            starter_base_lat_ms = (float)res.engage_latency_ms;
          }
          else
          {
            float alpha = 1.0f / (float)(starter_base_samples + 1);
            starter_base_r_mohm = alpha * res.r_eff_mohm + (1.0f - alpha) * starter_base_r_mohm;
            starter_base_lat_ms = alpha * (float)res.engage_latency_ms + (1.0f - alpha) * starter_base_lat_ms;
          }
          starter_base_samples++;
        }
        else if (solenoid_abnormal)
        {
          recordStarterStrike(STARTER_FAULT_SOLENOID);
        }
      }
      else if (r_plausible)
      {
        float r_ratio = res.r_eff_mohm / starter_base_r_mohm;
        uint8_t detected = STARTER_OK;
        if (r_ratio >= STARTER_HIGH_RES_RATIO)
        {
          detected = STARTER_FAULT_HIGH_RES;
        }
        else if (r_ratio <= STARTER_HIGH_DRAG_RATIO)
        {
          detected = STARTER_FAULT_HIGH_DRAG;
        }
        else if (solenoid_abnormal)
        {
          detected = STARTER_FAULT_SOLENOID;
        }

        if (detected != STARTER_OK)
        {
          recordStarterStrike(detected);
        }
        else if (r_ratio > 0.80f && r_ratio < 1.22f && !solenoid_abnormal)
        {
          starter_degrade_strikes = 0;
          if (starter_fault_code != STARTER_OK)
          {
            starter_healthy_streak++;
            if (starter_healthy_streak >= STARTER_RECOVERY_STARTS)
            {
              starter_fault_code = STARTER_OK;
              starter_healthy_streak = 0;
            }
          }
          else if (r_ratio >= 0.85f && r_ratio <= 1.15f &&
                   res.dropout_count == 0 &&
                   res.engage_latency_ms < STARTER_LATENCY_WARN_MS)
          {
            starter_base_r_mohm = 0.02f * res.r_eff_mohm + 0.98f * starter_base_r_mohm;
            starter_base_lat_ms = 0.02f * (float)res.engage_latency_ms + 0.98f * starter_base_lat_ms;
          }
        }
      }
    }
    else if (solenoid_abnormal)
    {
      recordStarterStrike(STARTER_FAULT_SOLENOID);
    }

    if (starter_fault_code != STARTER_OK)
    {
      activeStarterAlert = starter_fault_code;
      starterAlertStartMs = now + 5200UL;
      starterAlertUntilMs = starterAlertStartMs + STARTER_ALERT_HOLD_MS;
    }
  }

  Serial.printf("[STARTER] #%lu out=%u T=%dC Ipk=%.0fA Ist=%.0fA Vst=%.1fV Vmin=%.1fV Reff=%.1fm(base=%.1f n=%u) lat=%ums dur=%ums drop=%u strk=%u flt=%u\n",
                (unsigned long)starter_total_cranks,
                (unsigned)res.outcome,
                (int)res.coolant_temp_c,
                res.peak_starter_a,
                res.steady_i_a,
                res.steady_v,
                res.min_crank_v,
                res.r_eff_mohm,
                starter_base_r_mohm,
                (unsigned)starter_base_samples,
                (unsigned)res.engage_latency_ms,
                (unsigned)res.crank_duration_ms,
                (unsigned)res.dropout_count,
                (unsigned)starter_degrade_strikes,
                (unsigned)starter_fault_code);
}
