
#include <Arduino.h>
#include <SPI.h>
#include "meas_if.h"

extern HardwareSerial SerialUART;

enum : uint8_t {
  REG_CONFIG1=0x00, REG_CONFIG2=0x01, REG_INT_STATUS=0x02, REG_INT_MASK=0x03,
  REG_TIME1=0x10, REG_CLOCK_COUNT2=0x13, REG_TIME3=0x14, REG_CALIBRATION1=0x1B, REG_CALIBRATION2=0x1C
};

static inline uint8_t cmd_header(uint8_t addr, bool write, bool autoinc) {
  uint8_t h=(addr&0x3F); if(write)h|=0x40; if(autoinc)h|=0x80; return h; }
static inline void cs_low(){ digitalWrite(TDC_CS,LOW);} static inline void cs_high(){ digitalWrite(TDC_CS,HIGH); }
static void tdc_write_u8(uint8_t reg, uint8_t val){ cs_low(); TDC_SPI.transfer(cmd_header(reg,true,false)); TDC_SPI.transfer(val); cs_high(); }
static uint8_t tdc_read_u8(uint8_t reg){ cs_low(); TDC_SPI.transfer(cmd_header(reg,false,false)); uint8_t v=TDC_SPI.transfer(0x00); cs_high(); return v; }
static uint32_t tdc_read_u24(uint8_t reg){ cs_low(); TDC_SPI.transfer(cmd_header(reg,false,true)); uint8_t b2=TDC_SPI.transfer(0), b1=TDC_SPI.transfer(0), b0=TDC_SPI.transfer(0); cs_high(); return ((uint32_t)b2<<16)|((uint32_t)b1<<8)|b0; }
static inline uint32_t rd23(uint8_t reg24){ return (tdc_read_u24(reg24)&0x7FFFFF); }

// EMA smoothing
static double last_delta_t_ns = 0.0;
static double ema_dt_ns = 0.0, ema_df_mHz = 0.0;
static bool   ema_init = false;
static constexpr double ALPHA = 0.20;   // 0..1

void meas_if_init(void) {
    SerialUART.println("[TDC] meas_if_init()");

    pinMode(TDC_CS, OUTPUT);
    pinMode(TDC_ENABLE, OUTPUT);
    pinMode(TDC_INTB, INPUT_PULLUP);
    cs_high();
    digitalWrite(TDC_ENABLE, LOW);
    delay(3);
    digitalWrite(TDC_ENABLE, HIGH);
    delay(5);

    TDC_SPI.begin();
    TDC_SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
    tdc_write_u8(REG_INT_MASK, 0x07);

    const uint8_t config2 = (1u<<6)|(0u<<3)|(0x1);
    tdc_write_u8(REG_CONFIG2, config2);

    const uint8_t cfg1 = (1u<<1);
    tdc_write_u8(REG_CONFIG1, cfg1);

    tdc_write_u8(REG_INT_STATUS,0x1F);

    SerialUART.println("[TDC] init done");
        // ✅ Forceer vrijgave TDC-interrupts voor eerste meting
    tdc_write_u8(REG_INT_STATUS, 0x1F);
}


void meas_if_start(void){
  tdc_write_u8(REG_INT_STATUS,0x1F);
  uint8_t cfg1=tdc_read_u8(REG_CONFIG1); cfg1|=0x01; // START_MEAS
  tdc_write_u8(REG_CONFIG1,cfg1);
}

bool meas_if_read(tdc_result_t *out){
  if(!out) return false;
  if(digitalRead(TDC_INTB)==HIGH) return false; // no data yet

  const uint32_t time1=rd23(REG_TIME1);
  const uint32_t time3=rd23(REG_TIME3);
  const uint32_t clk2=(tdc_read_u24(REG_CLOCK_COUNT2)&0xFFFF);
  const uint32_t cal1=rd23(REG_CALIBRATION1);
  const uint32_t cal2=rd23(REG_CALIBRATION2);
  tdc_write_u8(REG_INT_STATUS,0x1F);

  const double Tclk_s=1.0/8e6;
  const double calCount=((int32_t)cal2-(int32_t)cal1)/9.0;
  if(calCount<=0.0||clk2==0) return false;

  const double normLSB_s=Tclk_s/calCount;
  const double TOF2_s=normLSB_s*((int32_t)time1-(int32_t)time3)+(double)clk2*Tclk_s;

  out->tof2_ns    = TOF2_s*1e9;
  out->delta_t_ns = (TOF2_s - 1e-3)*1e9;            // wrap/offset model

  const double df_mHz=(out->delta_t_ns-last_delta_t_ns)*1e-6;
  out->delta_f_mHz = df_mHz;
  last_delta_t_ns  = out->delta_t_ns;

  // --- EMA smoothing ---
  if (!ema_init) {
    ema_dt_ns  = out->delta_t_ns;
    ema_df_mHz = out->delta_f_mHz;
    ema_init   = true;
  } else {
    ema_dt_ns  = (1.0-ALPHA)*ema_dt_ns  + ALPHA*out->delta_t_ns;
    ema_df_mHz = (1.0-ALPHA)*ema_df_mHz + ALPHA*out->delta_f_mHz;
  }
  out->delta_t_avg_ns  = ema_dt_ns;
  out->delta_f_avg_mHz = ema_df_mHz;

  return true;
}
