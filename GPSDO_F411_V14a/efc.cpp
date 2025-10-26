#include "efc.h"
#include <Arduino.h>
#include "stm32f4xx_hal.h"
#include <EEPROM.h>
static const uint32_t PWM_HZ=50000; static const uint32_t TIM5_PSC=0; static const uint32_t TIM5_ARR=(uint32_t)(96000000.0/PWM_HZ)-1;
static float g_duty=0.80f; static bool g_cal_valid=false;
static uint32_t crc32_acc(const uint8_t* d,size_t n){ uint32_t crc=0xFFFFFFFFu; for(size_t i=0;i<n;i++){ crc^=d[i]; for(int b=0;b<8;b++) crc=(crc>>1) ^ (0xEDB88320u & (-(int)(crc&1))); } return ~crc;}
static inline float clampf(float x,float lo,float hi){return x<lo?lo:(x>hi?hi:x);}
void efc_init(void){
  __HAL_RCC_TIM5_CLK_ENABLE(); pinMode(PA0,OUTPUT);
  TIM_HandleTypeDef t={0}; t.Instance=TIM5; t.Init.Prescaler=TIM5_PSC; t.Init.CounterMode=TIM_COUNTERMODE_UP; t.Init.Period=TIM5_ARR; t.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1; t.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_DISABLE; HAL_TIM_PWM_Init(&t);
  TIM_OC_InitTypeDef c={0}; c.OCMode=TIM_OCMODE_PWM1; c.Pulse=(uint32_t)((TIM5_ARR+1)*g_duty+0.5f); c.OCPolarity=TIM_OCPOLARITY_HIGH; c.OCFastMode=TIM_OCFAST_DISABLE; HAL_TIM_PWM_ConfigChannel(&t,&c,TIM_CHANNEL_1); HAL_TIM_PWM_Start(&t,TIM_CHANNEL_1);
}
void efc_set_duty(float duty){ duty=clampf(duty,0.0f,1.0f); g_duty=duty; TIM5->CCR1=(uint32_t)((TIM5_ARR+1)*duty+0.5f); }
float efc_get_duty(void){ return g_duty; } float efc_get_v_pwm(void){ return PWM_V_SUPPLY*g_duty; } float efc_get_v_ocxo(void){ return EFC_V_SUPPLY*(1.0f-g_duty); } uint32_t efc_get_ccr1(void){ return TIM5->CCR1; }
static const int EEPROM_ADDR=0;
bool eeprom_load(CalData& out){ EEPROM.get(EEPROM_ADDR,out); uint32_t calc=crc32_acc((uint8_t*)&out,sizeof(CalData)-sizeof(uint32_t)); g_cal_valid=(out.crc==calc && out.duty_cal>=DUTY_MIN_HW_CONST && out.duty_cal<=DUTY_MAX_HW_CONST); return g_cal_valid; }
void eeprom_save(const CalData& in){ CalData wr=in; wr.crc=crc32_acc((uint8_t*)&wr,sizeof(CalData)-sizeof(uint32_t)); EEPROM.put(EEPROM_ADDR,wr); g_cal_valid=true; }
void eeprom_reset(void){ CalData z={0}; z.crc=0; EEPROM.put(EEPROM_ADDR,z); g_cal_valid=false; }
bool eeprom_has_valid(void){ return g_cal_valid; }
