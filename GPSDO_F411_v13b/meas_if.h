
#ifndef MEAS_IF_H
#define MEAS_IF_H
#include <Arduino.h>
#include <SPI.h>

// TDC7200 pins
#define TDC_CS      PA4
#define TDC_ENABLE  PB1
#define TDC_INTB    PB0
#define TDC_SPI     SPI

typedef struct {
  uint32_t time1, time2, clock_count1, cal1, cal2;
  double   tof2_ns, delta_t_ns, delta_f_mHz;
  double   delta_t_avg_ns, delta_f_avg_mHz;
} tdc_result_t;

void meas_if_init(void);
void meas_if_start(void);
bool meas_if_read(tdc_result_t *out);
#endif
