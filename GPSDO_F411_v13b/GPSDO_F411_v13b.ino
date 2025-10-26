// GPSDO_F411_v11h.ino

// ======================================================
// Includes + globals
// ======================================================

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "clock_custom.h"
#include "meas_if.h"
#include "efc.h"
#include "stm32f4xx.h"

// UART2 (PA3 RX, PA2 TX)
HardwareSerial SerialUART(PA3, PA2);

// LCD
LiquidCrystal_I2C lcd(0x27, 20, 4);

// ------------------------------------------------------
// Bookkeeping voor (re)starten TDC
// ------------------------------------------------------
static uint32_t g_lastSampleMs = 0;   // wanneer zagen we laatst een sample?
static uint32_t g_lastStartMs  = 0;   // wanneer hebben we TDC laatst gestart?

// ======================================================
// SETUP — CLOCK FIRST ✅
// ======================================================
void setup() {

  // ✅ Bevroren opstartvolgorde
  clock_custom();
  SystemCoreClock = 96000000;
  HAL_InitTick(TICK_INT_PRIORITY);

  // LED
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, HIGH);

  // UART
  SerialUART.begin(115200);
  SerialUART.flush();
  SerialUART.println("\n[GPSDO] F411 v11h start");
  delay(100);  // terminal sync

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("GPSDO F411 v11h");

  // Interfaces
  meas_if_init();
  efc_init();

  SerialUART.println("[MAIN] Init complete — starting regulation loop");

  // Start eerste TDC meting
  delay(200);              // TDC stabilisatie na ENABLE
  meas_if_start();
  g_lastStartMs = millis();
}


// ======================================================
// LCD HELPER FUNCTIES ✅
// ======================================================
static void lcdDrawProgressBar(uint8_t row, float p) {
  if(p<0)p=0;
  if(p>1)p=1;
  uint8_t total = 16;
  uint8_t filled = (uint8_t)(p * total + 0.5f);
  lcd.setCursor(3, row);
  for (uint8_t i=0; i<total; i++) lcd.print(i<filled ? '#' : '.');
  char buf[8];
  snprintf(buf, sizeof(buf), " %3d%%", (int)(p*100));
  lcd.print(buf);
}

static void lcdPrintStateLine(const efc_status_t& s) {
  lcd.setCursor(0,0);
  if(s.state == EFC_STARTUP) lcd.print("Calibrating slope... ");
  else if(s.state == EFC_PI) lcd.print("PI stabilizing...   ");
  else                       lcd.print("LOCKED (Stable)     ");
}


// ======================================================
// LOOP (1 Hz)
// ======================================================
void loop() {

  static uint32_t secCounter = 0;
  tdc_result_t r;

  // ---- Probeer sample lezen ----
  if (meas_if_read(&r)) {

    g_lastSampleMs = millis();
    secCounter++;

    // Regellus
    efc_update(r);
    efc_status_t s = efc_getStatus();

    // ✅ RS232 — volledig (leesbaar, geen printf)
    SerialUART.print("[TDC] #");
    SerialUART.print((unsigned long)secCounter);

    SerialUART.print("  dt=");
    SerialUART.print(r.delta_t_ns * 1000.0, 1);
    SerialUART.print(" ps");

    SerialUART.print("  dt_avg=");
    SerialUART.print(r.delta_t_avg_ns * 1000.0, 1);
    SerialUART.print(" ps");

    SerialUART.print("  df=");
    SerialUART.print(r.delta_f_mHz * 1e6, 3); // mHz → pHz
    SerialUART.print(" pHz");

    SerialUART.print("  df_avg=");
    SerialUART.print(r.delta_f_avg_mHz * 1e6, 3); // mHz → pHz
    SerialUART.print(" pHz");

    SerialUART.print("  V=");
    SerialUART.print(s.efc_voltage_V, 6);
    SerialUART.print(" V");

    SerialUART.print("  S=");
    SerialUART.print((int)s.state);

    SerialUART.print("  slope=");
    SerialUART.print(s.slope_Hz_per_V, 5);
    SerialUART.println(" Hz/V");
    // --- CSV (v13b) ---
    static double prev_dt_ps = NAN;
    bool wrap = false;
    double dt_ps_now = r.delta_t_ns * 1000.0;
    if (!isnan(prev_dt_ps)) {
      double dd = dt_ps_now - prev_dt_ps;
      if (dd > 300000000.0 || dd < -300000000.0) wrap = true; // ~0.3 s jump indicates wrap
    }
    prev_dt_ps = dt_ps_now;

    if (secCounter == 1) {
      SerialUART.println("[CSV] t_sec,dt_ps,dt_avg_ps,df_pHz,df_avg_pHz,V_EFC,S,S_name,wrap,CCR1,slope_Hz_per_V");
    }
    SerialUART.print("[CSV] ");
    SerialUART.print((unsigned long)secCounter);
    SerialUART.print(",");
    SerialUART.print(dt_ps_now, 1);
    SerialUART.print(",");
    SerialUART.print(r.delta_t_avg_ns * 1000.0, 1);
    SerialUART.print(",");
    SerialUART.print(r.delta_f_mHz * 1e6, 3);
    SerialUART.print(",");
    SerialUART.print(r.delta_f_avg_mHz * 1e6, 3);
    SerialUART.print(",");
    SerialUART.print(s.efc_voltage_V, 6);
    SerialUART.print(",");
    SerialUART.print((int)s.state);
    SerialUART.print(",");
    const char* sname = (s.state==0?"STARTUP":(s.state==1?"PI":"LOCKED"));
    SerialUART.print(sname);
    SerialUART.print(",");
    SerialUART.print(wrap ? 1 : 0);
    SerialUART.print(",");
    #ifdef TIM5
      SerialUART.print((unsigned long)TIM5->CCR1);
    #else
      SerialUART.print(0);
    #endif
    SerialUART.print(",");
    SerialUART.println(s.slope_Hz_per_V, 6);


    // ---- LCD updates ----
    lcdPrintStateLine(s);

    // Regel 2: Absolute frequentie
double f_Hz  = 10e6 + (r.delta_f_avg_mHz * 1e-3);   // mHz → Hz
double f_MHz = f_Hz / 1e6;

lcd.setCursor(0,1);
// maak de hele regel eerst leeg om spook-characters te vermijden
lcd.print("                    ");
lcd.setCursor(0,1);
lcd.print("F=");

// Robuust formatteren met dtostrf (breedte 13, 9 decimals)
// Geeft bijvoorbeeld: "10.000000123"
char fbuf[24];
dtostrf(f_MHz, 13, 9, fbuf);
lcd.print(fbuf);
lcd.print(" MHz");


    // Regel 4: EFC
    lcd.setCursor(0,3);
    lcd.print("V=");
    lcd.print(s.efc_voltage_V, 6);
    lcd.print(" S=");
    lcd.print((int)s.state);
    lcd.print("   ");

    // ---- BELANGRIJK: start de VOLGENDE meting ----
    meas_if_start();
    g_lastStartMs = millis();
  }

  // ---- Fail-safe: als we >1.5 s geen sample zagen, retrigger start ----
  if ((millis() - g_lastSampleMs) > 1500 && (millis() - g_lastStartMs) > 500) {
    SerialUART.println("[TDC] no data >1.5s — retrigger START");
    meas_if_start();
    g_lastStartMs = millis();
  }

  delay(1000);
}
