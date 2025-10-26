// efc.cpp 


#include <Arduino.h>
#include "efc.h"
#include "meas_if.h"   // voor tdc_result_t

// UART uit .ino
extern HardwareSerial SerialUART;

// ==============================
// Interne status
// ==============================
static efc_status_t S;
static double integ_V = 0.0;
static uint32_t last_update_ms = 0;

// ==============================
// Tuning- en grenswaarden
// (gelijk aan jouw v11h; later te finetunen)
// ==============================
static constexpr double V_MIN = 0.0;
static constexpr double V_MAX = 3.3;

// PI (conservatief; later fijnslijpen)
static constexpr double KP = 0.008;
static constexpr double KI = 0.020;

// Max. spanningsverandering (V/s)
static constexpr double SLEW_V_PER_S = 0.20;

// Lock-criteria (op fout in Hz)
static constexpr double LOCK_BAND_HZ = 0.05;       // |df_avg| < 0.05 Hz (ruim)
static constexpr uint32_t LOCK_HOLD_MS = 10000;    // 10 s binnen band

// Startup-sweep (rond 1.65 V, ±SWEEP_V/2)
static constexpr double   SWEEP_V  = 0.8;          // totale sweep
static constexpr uint32_t SWEEP_MS = 16000;        // 16 s

// Little helper
static inline double clamp(double x, double lo, double hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// ==============================
// Lage-level EFC-uitsturing
// TIM5 CH1 (PA0) — 50 kHz PWM
// ==============================
static void efc_analog_set(double v) {
  if (v < V_MIN) v = V_MIN;
  if (v > V_MAX) v = V_MAX;
  S.efc_voltage_V = v;

  // Duty = v / 3.3
  uint32_t arr = TIM5->ARR;
  if (arr == 0) { // guard (zou normaal al init zijn in clock_custom)
    TIM5->CCR1 = 0;
    return;
  }
  uint32_t ccr = (uint32_t)((v / 3.3) * (arr + 1));
  if (ccr > arr) ccr = arr;
  TIM5->CCR1 = ccr;
}

// ==============================
// API
// ==============================
void efc_init() {
  // Start midden in het bereik
  efc_analog_set(1.65);

  // Init status
  S.state         = EFC_STARTUP;
  S.lock_ready    = false;
  S.efc_voltage_V = 1.65;
  S.slope_Hz_per_V= 0.0;
  S.V_lock_V      = 1.65;
  S.start_time_ms = millis();
  S.progress_01   = 0.0f;

  integ_V         = 0.0;
  last_update_ms  = millis();

  SerialUART.println("[EFC] init: STARTUP slope-calib");
}

efc_status_t efc_getStatus() { return S; }

void efc_clearEEPROM() {
  // In deze code bewaren we geen EEPROM, maar we resetten wel runtime
  S.slope_Hz_per_V = 0.0;
  S.lock_ready     = false;
  S.state          = EFC_STARTUP;
  S.progress_01    = 0.0f;
  S.V_lock_V       = 1.65;
  integ_V          = 0.0;
  S.start_time_ms  = millis();
  efc_analog_set(1.65);

  SerialUART.println("[EFC] OCXO slope gewist → STARTUP opnieuw");
}

// ==============================
// Hoofd-update
// ==============================
void efc_update(const tdc_result_t& r) {
  // Delta-tijd in seconden
  uint32_t now = millis();
  double dt_s = (now - last_update_ms) / 1000.0;
  if (dt_s <= 0.0) dt_s = 1.0;
  last_update_ms = now;

  // ==========================
  // S0: STARTUP — slope-bepaling
  // ==========================
  // ===== S0: STARTUP — slope over twee plateaus (low, high) =====
if (S.state == EFC_STARTUP) {
  const double   Vmid         = 1.65;
  const double   Vspan        = SWEEP_V;             // totale sweep (bv. 0.8 V)
  const double   Vlow         = clamp(Vmid - Vspan/2, V_MIN, V_MAX);
  const double   Vhigh        = clamp(Vmid + Vspan/2, V_MIN, V_MAX);
  const uint32_t HOLD_MS      = SWEEP_MS/2;          // bv. 8000 ms per plateau

  uint32_t t_elapsed = now - S.start_time_ms;
  S.progress_01 = (float)min(1.0, t_elapsed / (double)SWEEP_MS);

  // Plateau-selectie
  bool onLow  = (t_elapsed < HOLD_MS);
  double Vcmd = onLow ? Vlow : Vhigh;
  efc_analog_set(Vcmd);

  // Accu’s voor plateau-gemiddelden (df in Hz)
  static double sumLow=0.0, sumHigh=0.0;
  static uint16_t nLow=0, nHigh=0;

  double df_Hz = r.delta_f_avg_mHz * 1e-3;  // mHz → Hz

  if (onLow)  { sumLow  += df_Hz; nLow++;  }
  else        { sumHigh += df_Hz; nHigh++; }

  // Klaar na tweede plateau
  if (t_elapsed >= SWEEP_MS) {
    double fL = (nLow  ? sumLow  / nLow  : 0.0);
    double fH = (nHigh ? sumHigh / nHigh : 0.0);
    double dV = (Vhigh - Vlow);
    double slope = 1.0; // fallback

    if (dV > 1e-6) {
      slope = (fH - fL) / dV;      // Hz/V
      // sanity-band voor OCXO
      if (!(fabs(slope) > 0.01 && fabs(slope) < 50.0)) slope = 1.0;
    }

    S.slope_Hz_per_V = slope;

    // Reset accumulators voor evt. volgende STARTUP-run
    sumLow = sumHigh = 0.0; nLow = nHigh = 0;

    // Door naar PI
    S.V_lock_V      = Vmid;           // lock rond midden van de sweep
    S.state         = EFC_PI;
    S.lock_ready    = false;
    S.start_time_ms = now;
    S.progress_01   = 0.0f;
    integ_V         = 0.0;

    SerialUART.print("[EFC] slope (Hz/V)=");
    SerialUART.println(S.slope_Hz_per_V, 3);
  }
  return;
}


  // ==========================
  // S1/S2: PI-regeling
  // ==========================
  // Fout in Hz (EMA-waarde)
  double err_Hz = r.delta_f_avg_mHz * 1e-3;  // mHz → Hz

  // PI-actie (op frequentiefout)
  double P = KP * err_Hz;
  double I_candidate = integ_V + KI * err_Hz * dt_s;

  // Command rond lock-setpoint
  double v_unc = S.V_lock_V - (P + I_candidate);

  // Slew-limit toepassen
  double dv_max = SLEW_V_PER_S * dt_s;
  double v_prev = S.efc_voltage_V;
  double v_cmd  = clamp(v_unc, V_MIN, V_MAX);
  v_cmd         = clamp(v_cmd, v_prev - dv_max, v_prev + dv_max);

  // Anti-windup
  bool saturated = (v_cmd <= V_MIN + 1e-6) || (v_cmd >= V_MAX - 1e-6);
  if (!saturated) {
    integ_V = I_candidate;
  }

  // Uitsturen
  efc_analog_set(v_cmd);

  // ==========================
  // Lockdetectie
  // ==========================
  static uint32_t inband_ms = 0;
  if (fabs(err_Hz) < LOCK_BAND_HZ) {
    // in band
    uint32_t add = (uint32_t)(dt_s * 1000.0);
    // saturatie/overflow vermijden
    if (UINT32_MAX - inband_ms < add) inband_ms = LOCK_HOLD_MS;
    else inband_ms += add;

    if (inband_ms >= LOCK_HOLD_MS) {
      S.state = EFC_LOCKED;
      S.lock_ready = true;
    }
  } else {
    inband_ms   = 0;
    S.state     = EFC_PI;
    S.lock_ready= false;
  }

  // Voor UI-progress
  if (S.state == EFC_PI) {
    double pd = (double)inband_ms / (double)LOCK_HOLD_MS;
    if (pd < 0.0) pd = 0.0; if (pd > 1.0) pd = 1.0;
    S.progress_01 = (float)pd;
  } else if (S.state == EFC_LOCKED) {
    S.progress_01 = 1.0f;
  }
}
