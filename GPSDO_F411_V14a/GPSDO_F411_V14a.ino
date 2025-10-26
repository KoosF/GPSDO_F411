/* FROZEN HEADER ... (see spec in our chat) */
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "clock_custom.h"
#include "meas_if.h"
#include "efc.h"
HardwareSerial SerialUART(PA3, PA2);
LiquidCrystal_I2C lcd(0x27, 20, 4);
static uint32_t g_lastSampleMs=0, g_lastStartMs=0;
/* END FROZEN HEADER */

static const uint32_t CSV_BAUD=115200;
enum State:uint8_t{S0_CAL=0,S1_WARMUP=1,S2_LOCK=2}; static State state=S1_WARMUP;
static const float Kp=0.10f, Ki=0.001f;
static const uint32_t MIN_WARMUP_SEC=600, DUTY_NEAR_CAL_HOLD_SEC=30, DF_AVG_LOCK_HOLD_SEC=60;
static const float DUTY_NEAR_CAL=0.01f, DF_AVG_LOCK_UHZ=0.05f, SLEW_DUTY_PER_SEC=0.0005f;
static const float DUTY_MIN_S2=0.30f, DUTY_MAX_S2=0.70f;
static float duty_cal=0.50f, V_cal=2.50f;
static uint32_t t_sec=0, last_page_switch=0; static uint8_t page_idx=0;
extern volatile double dt_ps,dt_avg_ps,df_pHz,df_avg_pHz; extern volatile uint32_t wrap_count; extern volatile double slope_Hz_per_V;
static inline float clampf(float x,float lo,float hi){return x<lo?lo:(x>hi?hi:x);} 
static const char* state_name(State s){switch(s){case S0_CAL:return "S0_CAL";case S1_WARMUP:return "S1_WARMUP";case S2_LOCK:return "S2_LOCK";default:return "UNK";}}
static void csv_print_header(); static void csv_print_row(); static void lcd_show_pageA(); static void lcd_show_pageB(); static void lcd_show_pageC();
static void state_log_change(State s,const char* n); static void handle_serial_commands(); static void pi_reset_to_cal();

void setup(){
  clock_custom(); SystemCoreClock=96000000; HAL_InitTick(TICK_INT_PRIORITY);
  pinMode(PC13,OUTPUT); digitalWrite(PC13,HIGH);
  SerialUART.begin(CSV_BAUD); SerialUART.flush(); SerialUART.println("\nGPSDO_F411_V14a"); delay(100);
  if(SystemCoreClock!=96000000){ SerialUART.println(F("FATAL: CLOCK NOT 96MHz")); while(1){ digitalWrite(PC13,!digitalRead(PC13)); delay(200);} }
  Wire.begin(); lcd.init(); lcd.backlight();
  efc_init(); efc_set_duty(0.80f);
  meas_if_init();
  CalData cd; if(eeprom_load(cd)){duty_cal=cd.duty_cal; V_cal=cd.V_cal; slope_Hz_per_V=cd.slope_Hz_per_V; SerialUART.println(F("EEPROM_CAL_LOADED"));}
  else { duty_cal=0.50f; V_cal=2.50f; slope_Hz_per_V=1.0f; SerialUART.println(F("EEPROM_CAL_INVALID")); }
  lcd.clear(); lcd.setCursor(0,0); lcd.print("GPSDO F411 v14a");
  lcd.setCursor(0,1); lcd.print("Start: duty="); lcd.print(efc_get_duty(),6);
  lcd.setCursor(0,2); lcd.print("V_OCXO="); lcd.print(efc_get_v_ocxo(),5); lcd.print("V");
  lcd.setCursor(0,3); lcd.print("S: S1 WARMUP");
  csv_print_header();
}

void loop(){
  static uint32_t last_ms=0; uint32_t now=millis(); if(now-last_ms<1000){ handle_serial_commands(); return; } last_ms=now; t_sec++;
  handle_serial_commands();
  meas_updateFromHardware();
  switch(state){
    case S1_WARMUP:{
      float duty=efc_get_duty(); float delta=duty_cal-duty; float step=clampf(delta,-SLEW_DUTY_PER_SEC,SLEW_DUTY_PER_SEC);
      duty=clampf(duty+step,DUTY_MIN_HW_CONST,DUTY_MAX_HW_CONST); efc_set_duty(duty);
      static uint32_t near_cal_hold=0, lock_hold=0;
      if(fabs(duty-duty_cal)<DUTY_NEAR_CAL) near_cal_hold++; else near_cal_hold=0;
      if(!isnan(df_avg_pHz) && fabs(df_avg_pHz)<(DF_AVG_LOCK_UHZ*1e6)) lock_hold++; else lock_hold=0;
      bool warmup_time_ok=(t_sec>=MIN_WARMUP_SEC), near_cal_ok=(near_cal_hold>=DUTY_NEAR_CAL_HOLD_SEC), df_ok=(lock_hold>=DF_AVG_LOCK_HOLD_SEC);
      bool in_hw_band=(duty>=DUTY_MIN_HW_CONST && duty<=DUTY_MAX_HW_CONST);
      if(warmup_time_ok && near_cal_ok && df_ok && in_hw_band){
        if(!eeprom_has_valid()){ V_cal=efc_get_v_ocxo(); duty_cal=clampf(efc_get_duty(),DUTY_MIN_S2,DUTY_MAX_S2); CalData cd={duty_cal,V_cal,(float)slope_Hz_per_V,0}; eeprom_save(cd); SerialUART.println(F("EEPROM_CAL_SAVED")); }
        state=S2_LOCK; pi_reset_to_cal(); state_log_change(S2_LOCK,"S2_LOCK");
      }
    }break;
    case S2_LOCK:{
      if(!isnan(df_avg_pHz)){
        static double I=0.0; const double df_Hz=df_avg_pHz*1e-12; const double e=-df_Hz;
        const double uP=Kp*e; I+=Ki*e; double u=uP+I; double d_duty = -u / EFC_V_SUPPLY;
        float duty=efc_get_duty() + (float)d_duty; bool clamp_hit=false;
        if(duty<DUTY_MIN_S2){duty=DUTY_MIN_S2; clamp_hit=true;}
        if(duty>DUTY_MAX_S2){duty=DUTY_MAX_S2; clamp_hit=true;}
        if(clamp_hit){ I*=0.9; SerialUART.println(F("CLAMP_HIT:S2")); }
        efc_set_duty(duty);
      }
    }break;
    case S0_CAL: default:{ state=S1_WARMUP; state_log_change(S1_WARMUP,"S1_WARMUP"); }break;
  }
  uint32_t rotate_period=(state==S2_LOCK)?5:3;
  if((t_sec-last_page_switch)>=rotate_period){ last_page_switch=t_sec; page_idx=(page_idx+1)%3; }
  switch(page_idx){ case 0:lcd_show_pageA();break; case 1:lcd_show_pageB();break; case 2:lcd_show_pageC();break; }
  csv_print_row();
}

static void handle_serial_commands(){ while(SerialUART.available()>0){ int c=SerialUART.read(); if(c=='R'){ eeprom_reset(); SerialUART.println(F("EEPROM_CAL_RESET")); } } }

static void lcd_show_pageA(){
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("A: EFC / Duty");
  lcd.setCursor(0,1); lcd.print("V_OCXO: "); lcd.print(efc_get_v_ocxo(),5); lcd.print(" V");
  lcd.setCursor(0,2); lcd.print("duty:   "); lcd.print(efc_get_duty(),6);
  lcd.setCursor(0,3); lcd.print("V_pwm:  "); lcd.print(efc_get_v_pwm(),4); lcd.print(" V  S:");
  lcd.print(state==S1_WARMUP?"S1":(state==S2_LOCK?"S2":"S0"));
}
static void lcd_show_pageB(){
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("B: Freq / Warmup");
  lcd.setCursor(0,1); lcd.print("d f_av: ");
  if(isnan(df_avg_pHz)) lcd.print("---"); else { double uHz=df_avg_pHz*1e-6; lcd.print(uHz,3); lcd.print(" uHz"); }
  float err=fabs(efc_get_duty()-duty_cal); float prog=1.0f - clampf(err/0.20f,0.0f,1.0f); int bars=(int)(prog*10+0.5f);
  lcd.setCursor(0,2); lcd.print("Warmup:["); for(int i=0;i<10;i++) lcd.print(i<bars?'#':'-'); lcd.print("] ");
  lcd.setCursor(0,3); lcd.print("t:"); lcd.print(t_sec); lcd.print(" s   S:"); lcd.print(state==S1_WARMUP?"S1":(state==S2_LOCK?"S2":"S0"));
}
static void lcd_show_pageC(){
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("C: Phase/Jitter");
  lcd.setCursor(0,1); lcd.print("dt:   "); if(isnan(dt_ps)) lcd.print("---"); else lcd.print(dt_ps,1); lcd.print(" ps");
  lcd.setCursor(0,2); lcd.print("dt_av:"); if(isnan(dt_avg_ps)) lcd.print("---"); else lcd.print(dt_avg_ps,1); lcd.print(" ps");
  lcd.setCursor(0,3); lcd.print("wrap:"); lcd.print(wrap_count); lcd.print("   LOCK: ");
  if(state==S2_LOCK){ lcd.print("\xE2\x9C\x94 "); lcd.print("\xE2\x96\xAA"); } else { lcd.print("\xE2\x9C\x96 "); lcd.print("\xE2\x97\x8F"); }
}
static void csv_print_header(){
  SerialUART.println(F("t_sec,dt_ps,dt_avg_ps,df_pHz,df_avg_pHz,duty,V_pwm_V,V_OCXO_V,S,S_name,wrap,CCR1,slope_Hz_per_V"));
}
static void csv_print_row(){
  const float duty=efc_get_duty(), V_pwm=efc_get_v_pwm(), V_ocxo=efc_get_v_ocxo(); const uint32_t CCR1=efc_get_ccr1();
  SerialUART.print(t_sec); SerialUART.print(',');
  if(isnan(dt_ps)) SerialUART.print(' '); else SerialUART.print(dt_ps,1); SerialUART.print(',');
  if(isnan(dt_avg_ps)) SerialUART.print(' '); else SerialUART.print(dt_avg_ps,1); SerialUART.print(',');
  if(isnan(df_pHz)) SerialUART.print(' '); else SerialUART.print(df_pHz,0); SerialUART.print(',');
  if(isnan(df_avg_pHz)) SerialUART.print(' '); else SerialUART.print(df_avg_pHz,0); SerialUART.print(',');
  SerialUART.print(duty,6); SerialUART.print(','); SerialUART.print(V_pwm,4); SerialUART.print(','); SerialUART.print(V_ocxo,5); SerialUART.print(',');
  SerialUART.print((int)state); SerialUART.print(','); SerialUART.print(state_name(state)); SerialUART.print(',');
  SerialUART.print(wrap_count); SerialUART.print(','); SerialUART.print(CCR1); SerialUART.print(','); SerialUART.println(slope_Hz_per_V,5);
}
static void state_log_change(State newS,const char* name){
  double uHz=isnan(df_avg_pHz)?NAN:df_avg_pHz*1e-6;
  SerialUART.print(F("STATE_CHANGE: ")); SerialUART.print(name);
  SerialUART.print(F(" (t=")); SerialUART.print(t_sec); SerialUART.print(F("s, duty=")); SerialUART.print(efc_get_duty(),6);
  SerialUART.print(F(", V_OCXO=")); SerialUART.print(efc_get_v_ocxo(),5); SerialUART.print('V'); SerialUART.print(F(", \xCE\x94f_avg="));
  if(isnan(uHz)) SerialUART.print("---"); else { SerialUART.print(uHz,3); SerialUART.print("\xC2\xB5Hz"); } SerialUART.println(')');
}
static void pi_reset_to_cal(){ float d=clampf(efc_get_duty(),DUTY_MIN_S2,DUTY_MAX_S2); efc_set_duty(d); }
