// efc.h


#ifndef EFC_H
#define EFC_H

#include <Arduino.h>
#include "meas_if.h"   // voor tdc_result_t

// -----------------------------------------------------------
// EFC-regeltoestand (bevroren mapping)
// -----------------------------------------------------------
typedef enum : uint8_t {
    EFC_STARTUP = 0,   // S0: Sweep / slope-calibratie
    EFC_PI      = 1,   // S1: PI-regeling (op zoek naar lock)
    EFC_LOCKED  = 2    // S2: Locked & stable
} efc_state_t;

// -----------------------------------------------------------
// EFC runtime status
// -----------------------------------------------------------
typedef struct {
    efc_state_t state;      // Huidige toestand (0/1/2)
    bool        lock_ready; // true zodra lock bereikt (S2)

    double      efc_voltage_V;     // actuele EFC-uitsturing (V)
    double      slope_Hz_per_V;    // OCXO-slope (Hz/V)
    double      V_lock_V;          // referentiespanning V bij lockpunt

    uint32_t    start_time_ms;     // timestamp voor startup/lock-check
    float       progress_01;       // 0.0–1.0 (voor voortgangsbalk)
} efc_status_t;

// -----------------------------------------------------------
// Publieke API
// -----------------------------------------------------------
void efc_init();
efc_status_t efc_getStatus();
void efc_update(const tdc_result_t& r);
void efc_clearEEPROM();

#endif
