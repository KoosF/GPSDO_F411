#pragma once
#include <stdint.h>
extern volatile double dt_ps;        // phase (ps)
extern volatile double dt_avg_ps;    // avg phase (ps)
extern volatile double df_pHz;       // freq error (pHz)
extern volatile double df_avg_pHz;   // avg freq error (pHz)
extern volatile uint32_t wrap_count; // wrap
extern volatile double slope_Hz_per_V; // Hz/V
void meas_if_init(void);
void meas_updateFromHardware(void);
