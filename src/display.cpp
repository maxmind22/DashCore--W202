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
  if (buzzer_state == 1)
  {
    digitalWriteFast(buzzer_pin, HIGH);
  }
  else if (!isTonePlaying())
  {
    digitalWriteFast(buzzer_pin, LOW);
  }
}
