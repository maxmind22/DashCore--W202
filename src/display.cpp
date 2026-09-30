#include "display.h"
#include "pushstart.h"
#include "security.h"

void drawStaticGauge()
{
  tv.drawRect(FUEL_X, FUEL_Y, FUEL_WIDTH, FUEL_HEIGHT, 0xFF);
}

void warnings(unsigned long now)
{
  buzzer_state = 0;
  static int priority = 0;
  //------------------------------------------
  if (percent <= LOW_FUEL_LEVEL && lowBlinkState && fuel_run)
  {
    // -------- LOW FUEL warning --------//

    tv.setCursor(FUEL_X, FUEL_Y + FUEL_HEIGHT + 3);
    tv.setTextColor(0xFF);
    tv.setTextSize(2);
    tv.print("LOW");
    tv.setTextSize(1);
    fuel_run = false;
    fuel = true;
  }
  else if (!lowBlinkState && fuel)
  {
    // tv.fillRect(FUEL_X + 20, FUEL_Y - 15 + FUEL_HEIGHT, 36, 16, 0x00);
    tv.fillRect(FUEL_X, FUEL_Y + FUEL_HEIGHT + 3, 36, 16, 0x00);
    fuel = false;
  }
  //------------------------------------------
  if (!coolant_level && lowBlinkState && priority == 0)
  {
    if (cool_run)
    {
      tv.setCursor(WARNING_X + 40, WARNING_Y);
      tv.setTextColor(0xFF);
      tv.print("COOLANT LOW");
      cool_run = false;
      cool = true;
    }
    buzzer_state = 1;
  }
  else if (!lowBlinkState && cool == true)
  {
    tv.fillRect(WARNING_X + 40, WARNING_Y, 66, 8, 0x00);
    cool = false;
  }

  //--------------------------------------------
  if (overspeed_state == 0 &&
      spd >= OVERSPEED_KMH)
  { // -------- over speed warning --------//
    overspeed_state = 1;
    counter = 0;
  }

  if (overspeed_state == 1)
  {
    if (speed_on)
    {
      tv.setCursor(WARNING_X + 30, WARNING_Y + 10);
      tv.setTextColor(0xFF);
      tv.print("OVER SPEED !");
      speed_on = false;
    }
    priority = 1;

    if (lowBlinkState2)
    {
      buzzer_state = 1;
    }
    if (counter > 3)
    {
      overspeed_state = 2;
    }
  }
  else if (overspeed_state == 2 || overspeed_state == 0)
  {
    if (!speed_on)
    {
      tv.fillRect(WARNING_X + 30, WARNING_Y + 10, 72, 8, 0x00);
      speed_on = true;
      priority = 0;
    }
    if (spd < OVERSPEED_KMH)
    {
      overspeed_state = 0;
    }
  }

  //------------------------------------------------
  if (oil_level > 0 && lowBlinkState && priority == 0)
  {
    if (oil_on)
    {
      tv.setCursor(WARNING_X + 30, WARNING_Y + 18);
      tv.setTextColor(0xFF);
      tv.print("LOW ENGINE OIL");
      oil = true;
      oil_on = false;
    }
    buzzer_state = 1;
  }
  else if (!lowBlinkState && oil == true)
  {
    tv.fillRect(WARNING_X + 30, WARNING_Y + 18, 84, 8, 0x00);
    oil = false;
  }
  //------------------------------------------------
  if (temp_out >= OVERHEAT_TEMP_C && lowBlinkState && priority == 0)
  {
    if (temp_on)
    {
      tv.setCursor(WARNING_X + 30, WARNING_Y + 26);
      tv.setTextColor(0xFF);
      tv.print("ENGINE OVERHEAT");
      hot = true;
      temp_on = false;
    }

    buzzer_state = 1;
  }
  else if (!lowBlinkState && hot == true)
  {
    tv.fillRect(WARNING_X + 30, WARNING_Y + 26, 90, 8,
                0x00); // clear old warning
    hot = false;
  }
  if (now - lastPacketTime > FRONT_MCU_TIMEOUT_MS &&
      conn_on)
  { // -----connection check--------------
    tv.setCursor(WARNING_X, WARNING_Y + 34);
    tv.setTextColor(0xFF);
    tv.print("Front MCU Disconnected");
    conn_on = false;
    spd = 0;
  }
  else if (!(now - lastPacketTime > FRONT_MCU_TIMEOUT_MS) && conn_on == false)
  {
    tv.fillRect(WARNING_X, WARNING_Y + 34, 140, 8, 0x00);
    conn_on = true;
  }
  if (injector_state == 1 && inj_on == true)
  {
    tv.fillCircle(10, 25, 5, 0x1C);
    inj_on = false;
  }
  else if (inj_on == false && injector_state == 0)
  {
    tv.fillCircle(10, 25, 5, 0x00);
    inj_on = true;
  }
  portENTER_CRITICAL(&dataMux);
  int local_charge_state = charge_state;
  uint32_t local_last_charge = last_charge;
  portEXIT_CRITICAL(&dataMux);

  if (local_charge_state == 1 && lowBlinkState &&
      now - local_last_charge > CHARGE_MALFUNCTION_DELAY_MS && priority == 0)
  {
    if (chg == 0)
    {
      tv.setCursor(WARNING_X + 10, WARNING_Y + 10);
      tv.setTextColor(0xFF);
      tv.print("CHARGING SYSTEM FAIL !");
      chg = 1;
    }
    buzzer_state = 1;
  }
  else if (!lowBlinkState && chg == 1)
  {
    tv.fillRect(WARNING_X + 10, WARNING_Y + 10, 140, 8, 0x00);
    chg = 0;
  }
  if (local_charge_state == 2 && lowBlinkState &&
      now - local_last_charge > BATTERY_LOW_DELAY_MS && priority == 0 && rpm > ENGINE_ACTIVE_RPM_THRESHOLD)
  {
    if (chg2 == 0)
    {
      tv.setCursor(WARNING_X + 50, WARNING_Y + 30);
      tv.setTextColor(0xFF);
      tv.print("BATTERY LOW !");
      chg2 = 1;
    }
    buzzer_state = 1;
  }

  // -------- Phone Key Detection Warning --------//
  static bool phoneKeyWarningDrawn = false;
  if (!isPhoneAuthorized())
  {
    if (lowBlinkState)
    {
      tv.setCursor(WARNING_X + 25, WARNING_Y + 42);
      tv.setTextColor(0xFF);
      tv.setTextSize(1);
      tv.print("NO KEY DETECTED");
      phoneKeyWarningDrawn = true;
    }
    else if (phoneKeyWarningDrawn)
    {
      tv.fillRect(WARNING_X + 25, WARNING_Y + 42, 100, 8, 0x00);
    }
  }
  else if (phoneKeyWarningDrawn)
  {
    tv.fillRect(WARNING_X + 25, WARNING_Y + 42, 100, 8, 0x00);
    phoneKeyWarningDrawn = false;
  }

  // -------- Vacuum Leak / Engine Health Warning --------//
  // Triggers if manifold vacuum at warm idle remains consistently < 5.0 psi for 5 continuous seconds
  static unsigned long vac_leak_start_ms = 0;
  static bool vac_leak_active = false;
  static bool vac_leak_drawn = false;

  bool vac_valid = (now - lastVacPacketTime <= FRONT_MCU_TIMEOUT_MS);
  bool idle_conditions = (currentState == STATE_RUNNING &&
                          rpm >= VAC_LEAK_MIN_RPM && rpm <= VAC_LEAK_MAX_RPM &&
                          spd <= VAC_LEAK_MAX_SPD_KMH &&
                          temp_out >= VAC_LEAK_MIN_TEMP_C &&
                          vac_valid);

  if (idle_conditions)
  {
    if (vacuum_psi < VAC_LEAK_THRESHOLD_PSI)
    {
      if (vac_leak_start_ms == 0)
      {
        vac_leak_start_ms = now;
      }
      else if (now - vac_leak_start_ms >= VAC_LEAK_PERSIST_MS)
      {
        vac_leak_active = true;
      }
    }
    else if (vacuum_psi >= VAC_LEAK_CLEAR_PSI)
    {
      vac_leak_start_ms = 0;
      vac_leak_active = false;
    }
  }
  else
  {
    if (!vac_leak_active)
    {
      vac_leak_start_ms = 0;
    }
    else if (vacuum_psi >= VAC_LEAK_CLEAR_PSI)
    {
      vac_leak_active = false;
      vac_leak_start_ms = 0;
    }
  }

  if (vac_leak_active && priority == 0)
  {
    if (lowBlinkState)
    {
      if (!vac_leak_drawn)
      {
        tv.setCursor(WARNING_X + 35, WARNING_Y + 50);
        tv.setTextColor(0xFF);
        tv.print("CHECK VACUUM");
        vac_leak_drawn = true;
      }
      buzzer_state = 1;
    }
    else if (vac_leak_drawn)
    {
      tv.fillRect(WARNING_X + 35, WARNING_Y + 50, 78, 8, 0x00);
      vac_leak_drawn = false;
    }
  }
  else if (vac_leak_drawn)
  {
    tv.fillRect(WARNING_X + 35, WARNING_Y + 50, 78, 8, 0x00);
    vac_leak_drawn = false;
  }

  // -------- Air Filter / Intake Restriction Diagnostic --------//
  // Triggers if manifold vacuum at high RPM under wide-open throttle (high inj duty) stays >= 1.8 psi
  static unsigned long air_filter_detect_start_ms = 0;
  static unsigned long air_filter_alert_until_ms = 0;
  static bool air_filter_drawn = false;

  bool high_load_wot = (currentState == STATE_RUNNING &&
                        rpm >= AIR_FILTER_CHECK_MIN_RPM &&
                        live_inj_duty_cycle >= AIR_FILTER_MIN_INJ_DUTY &&
                        vac_valid);

  if (high_load_wot)
  {
    if (vacuum_psi >= AIR_FILTER_RESTRICTION_VAC_PSI)
    {
      if (air_filter_detect_start_ms == 0)
      {
        air_filter_detect_start_ms = now;
      }
      else if (now - air_filter_detect_start_ms >= AIR_FILTER_DETECT_PERSIST_MS)
      {
        air_filter_alert_until_ms = now + AIR_FILTER_ALERT_HOLD_MS;
      }
    }
    else
    {
      air_filter_detect_start_ms = 0;
    }
  }
  else
  {
    air_filter_detect_start_ms = 0;
  }

  bool air_filter_active = (now < air_filter_alert_until_ms);

  if (air_filter_active && !vac_leak_active && priority == 0)
  {
    if (lowBlinkState)
    {
      if (!air_filter_drawn)
      {
        tv.setCursor(WARNING_X + 20, WARNING_Y + 50);
        tv.setTextColor(0xFF);
        tv.print("CHECK AIR FILTER");
        air_filter_drawn = true;
      }
      buzzer_state = 1;
    }
    else if (air_filter_drawn)
    {
      tv.fillRect(WARNING_X + 20, WARNING_Y + 50, 98, 8, 0x00);
      air_filter_drawn = false;
    }
  }
  else if (air_filter_drawn)
  {
    tv.fillRect(WARNING_X + 20, WARNING_Y + 50, 98, 8, 0x00);
    air_filter_drawn = false;
  }

  // -------- Idle Switch Misadjustment / Cable Stretch Diagnostic --------//
  // Detects stretched cable or misadjusted microswitch: engine is warm and idling with deep
  // manifold vacuum (>= 7.5 psi), but the throttle microswitch never clicked closed (th_switch_state == 0).
  // Without the switch closing, DFCO fuel-cut is silently disabled on every deceleration!
  static unsigned long idle_sw_fault_start_ms = 0;
  static bool idle_sw_fault_active = false;
  static bool idle_sw_drawn = false;

  bool idle_sw_conditions = (currentState == STATE_RUNNING &&
                             rpm >= VAC_LEAK_MIN_RPM && rpm <= VAC_LEAK_MAX_RPM &&
                             spd <= VAC_LEAK_MAX_SPD_KMH &&
                             temp_out >= VAC_LEAK_MIN_TEMP_C &&
                             vac_valid);

  if (idle_sw_conditions)
  {
    if (vacuum_psi >= IDLE_SW_FAULT_VAC_PSI && th_switch_state == 0)
    {
      if (idle_sw_fault_start_ms == 0)
      {
        idle_sw_fault_start_ms = now;
      }
      else if (now - idle_sw_fault_start_ms >= IDLE_SW_FAULT_PERSIST_MS)
      {
        idle_sw_fault_active = true;
      }
    }
    else if (th_switch_state == 1)
    {
      idle_sw_fault_start_ms = 0;
      idle_sw_fault_active = false;
    }
  }
  else
  {
    idle_sw_fault_start_ms = 0;
    if (th_switch_state == 1)
    {
      idle_sw_fault_active = false;
    }
  }

  if (idle_sw_fault_active && !vac_leak_active && !air_filter_active && priority == 0)
  {
    if (lowBlinkState)
    {
      if (!idle_sw_drawn)
      {
        tv.setCursor(WARNING_X + 15, WARNING_Y + 50);
        tv.setTextColor(0xFF);
        tv.print("CHECK IDLE SWITCH");
        idle_sw_drawn = true;
      }
      buzzer_state = 1;
    }
    else if (idle_sw_drawn)
    {
      tv.fillRect(WARNING_X + 15, WARNING_Y + 50, 106, 8, 0x00);
      idle_sw_drawn = false;
    }
  }
  else if (idle_sw_drawn)
  {
    tv.fillRect(WARNING_X + 15, WARNING_Y + 50, 106, 8, 0x00);
    idle_sw_drawn = false;
  }

  // -------- Bidirectional Fuel System Diagnostic (Rich FPR Leak vs Lean Fuel Starvation) --------//
  // Normal M111 warm idle net pulse is 1800 - 2400us.
  // 1. Rich / Low Pulse (< 1350us): ECU negative trim pinned; raw fuel entering (torn FPR diaphragm,
  //    dripping/stuck-open injector, blocked fuel return line).
  // 2. Lean / High Pulse (> 3000us): ECU positive trim pinned; fuel starvation (weak/dying fuel pump,
  //    clogged fuel filter, FPR stuck open with low rail pressure, partially clogged/varnished injectors).
  static unsigned long fuel_diag_rich_start_ms = 0;
  static unsigned long fuel_diag_lean_start_ms = 0;
  static uint8_t fuel_diag_fault = 0; // 0 = OK, 1 = Rich (FPR leak), 2 = Lean (Fuel pump / filter)
  static bool fuel_diag_drawn = false;

  bool fuel_diag_conditions = (currentState == STATE_RUNNING &&
                               rpm >= VAC_LEAK_MIN_RPM && rpm <= VAC_LEAK_MAX_RPM &&
                               spd <= VAC_LEAK_MAX_SPD_KMH &&
                               temp_out >= FPR_LEAK_MIN_TEMP_C &&
                               th_switch_state == 1 &&
                               injector_state == 0 &&
                               vac_valid &&
                               vacuum_psi >= FPR_LEAK_MIN_VAC_PSI);

  if (fuel_diag_conditions)
  {
    // Rich Check: Abnormally low pulse width (< 1350us)
    if (live_net_pulse_us > 400.0f && live_net_pulse_us < FPR_LEAK_MAX_PULSE_US)
    {
      if (fuel_diag_rich_start_ms == 0)
      {
        fuel_diag_rich_start_ms = now;
      }
      else if (now - fuel_diag_rich_start_ms >= FPR_LEAK_DETECT_PERSIST_MS)
      {
        fuel_diag_fault = 1;
      }
    }
    else if (live_net_pulse_us >= (FPR_LEAK_MAX_PULSE_US + 200.0f))
    {
      fuel_diag_rich_start_ms = 0;
      if (fuel_diag_fault == 1) fuel_diag_fault = 0;
    }

    // Lean Check: Abnormally high pulse width (> 3000us)
    if (live_net_pulse_us > FUEL_STARV_MIN_PULSE_US)
    {
      if (fuel_diag_lean_start_ms == 0)
      {
        fuel_diag_lean_start_ms = now;
      }
      else if (now - fuel_diag_lean_start_ms >= FPR_LEAK_DETECT_PERSIST_MS)
      {
        fuel_diag_fault = 2;
      }
    }
    else if (live_net_pulse_us <= (FUEL_STARV_MIN_PULSE_US - 200.0f))
    {
      fuel_diag_lean_start_ms = 0;
      if (fuel_diag_fault == 2) fuel_diag_fault = 0;
    }
  }
  else
  {
    fuel_diag_rich_start_ms = 0;
    fuel_diag_lean_start_ms = 0;
  }

  if (fuel_diag_fault > 0 && !vac_leak_active && !air_filter_active && !idle_sw_fault_active && priority == 0)
  {
    if (lowBlinkState)
    {
      if (!fuel_diag_drawn)
      {
        tv.setCursor(WARNING_X + 22, WARNING_Y + 50);
        tv.setTextColor(0xFF);
        if (fuel_diag_fault == 1)
        {
          tv.print("CHECK FPR LEAK");
        }
        else
        {
          tv.print("CHECK FUEL PUMP");
        }
        fuel_diag_drawn = true;
      }
      buzzer_state = 1;
    }
    else if (fuel_diag_drawn)
    {
      tv.fillRect(WARNING_X + 20, WARNING_Y + 50, 96, 8, 0x00);
      fuel_diag_drawn = false;
    }
  }
  else if (fuel_diag_drawn)
  {
    tv.fillRect(WARNING_X + 20, WARNING_Y + 50, 96, 8, 0x00);
    fuel_diag_drawn = false;
  }

  // -------- Auto Start-Stop Active Indicator --------//
  static bool ecoStopDrawn = false;
  if (currentState == STATE_AUTO_STOP)
  {
    if (!ecoStopDrawn)
    {
      tv.setCursor(WARNING_X + 35, WARNING_Y + 60);
      tv.setTextColor(0x1C); // Green in 8-bit palette
      tv.print("[A] ECO STOP");
      ecoStopDrawn = true;
    }
  }
  else if (ecoStopDrawn)
  {
    tv.fillRect(WARNING_X + 35, WARNING_Y + 60, 90, 8, 0x00);
    ecoStopDrawn = false;
  }

  //---------- ring boot chime  ---------
  if (now >= 1000 && boot_chime <= 70)
  {
    boot_chime++;
    buzzer_state = 1;
  }
  if (!isTonePlaying())
  {
    digitalWriteFast(buzzer_pin, buzzer_state ? HIGH : LOW);
  }
}
