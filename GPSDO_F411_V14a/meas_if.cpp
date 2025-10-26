#include "meas_if.h"
#include <Arduino.h>
#include <SPI.h>
static const uint8_t PIN_CS=PA4, PIN_INTB=PB0, PIN_EN=PB1;
static const uint8_t REG_CONFIG1=0x00, REG_CONFIG2=0x01, REG_INT_STATUS=0x02, REG_INT_MASK=0x03, REG_CALIB1=0x1B, REG_CALIB2=0x1C, REG_TIME2=0x11;
static const uint8_t CMD_READ=0x00, CMD_WRITE=0x40;
volatile double dt_ps=NAN, dt_avg_ps=NAN, df_pHz=NAN, df_avg_pHz=NAN; volatile uint32_t wrap_count=0; volatile double slope_Hz_per_V=1.0;
static double ema_dt=NAN, ema_df=NAN, prev_dt_ps=NAN; static const double ALPHA=0.2;
static inline void CS_L(){digitalWrite(PIN_CS,LOW);} static inline void CS_H(){digitalWrite(PIN_CS,HIGH);}
static uint8_t rd8(uint8_t r){CS_L(); SPI.transfer(CMD_READ|(r&0x3F)); uint8_t v=SPI.transfer(0); CS_H(); return v;}
static void wr8(uint8_t r,uint8_t v){CS_L(); SPI.transfer(CMD_WRITE|(r&0x3F)); SPI.transfer(v); CS_H();}
static uint32_t rd24(uint8_t r){CS_L(); SPI.transfer(CMD_READ|(r&0x3F)); uint32_t a=SPI.transfer(0),b=SPI.transfer(0),c=SPI.transfer(0); CS_H(); return (a<<16)|(b<<8)|c;}
static double lsb_ps(){
  uint32_t c1=rd24(REG_CALIB1), c2=rd24(REG_CALIB2); if(c2<=c1) return NAN; const double Tcal_s=80e-9; return Tcal_s/((double)(c2-c1))*1e12;
}
static bool read_TOF2_ps(double& tof2_ps){ uint32_t t2=rd24(REG_TIME2); double L=lsb_ps(); if(!isfinite(L)) return False; tof2_ps=(double)t2*L; return true; }
static void start_meas(){
  uint8_t cfg1=(2<<6) | (1<<0) | (1<<5); // MODE2, MEAS_EN, START
  wr8(REG_CONFIG1,cfg1); wr8(REG_CONFIG2,1); // NUM_STOP=1
}
void meas_if_init(void){
  pinMode(PIN_CS,OUTPUT); digitalWrite(PIN_CS,HIGH); pinMode(PIN_INTB,INPUT_PULLUP); pinMode(PIN_EN,OUTPUT); digitalWrite(PIN_EN,HIGH);
  SPI.begin(); SPI.beginTransaction(SPISettings(4000000,MSBFIRST,SPI_MODE0));
  wr8(REG_INT_MASK,0x00); (void)rd8(REG_INT_STATUS); start_meas();
}
void meas_updateFromHardware(void){
  uint32_t t0=millis(); bool done=false; while(millis()-t0<40){ if(digitalRead(PIN_INTB)==LOW){done=true;break;} delayMicroseconds(200);} if(!done){ start_meas(); return; }
  (void)rd8(REG_INT_STATUS);
  double tof2_ps; if(!read_TOF2_ps(tof2_ps)){ start_meas(); return; }
  const double one_ms_ps=1e9; double phase_ps=tof2_ps - one_ms_ps; dt_ps=phase_ps;
  if(!isfinite(ema_dt)) ema_dt=dt_ps; else ema_dt=ALPHA*dt_ps+(1.0-ALPHA)*ema_dt; dt_avg_ps=ema_dt;
  if(isfinite(prev_dt_ps)){ double dphi_s=(dt_ps - prev_dt_ps)*1e-12; double df_Hz = -dphi_s; df_pHz=df_Hz*1e12; if(!isfinite(ema_df)) ema_df=df_pHz; else ema_df=ALPHA*df_pHz+(1.0-ALPHA)*ema_df; df_avg_pHz=ema_df; }
  prev_dt_ps=dt_ps;
  if(digitalRead(PIN_INTB)==LOW) wrap_count++;
  start_meas();
}
