#pragma once
#include <stdint.h>
#include <stdbool.h>
static const float PWM_V_SUPPLY=3.3f, EFC_V_SUPPLY=5.0f;
static const float DUTY_MIN_HW_CONST=0.20f, DUTY_MAX_HW_CONST=0.80f;
void efc_init(void); void efc_set_duty(float duty); float efc_get_duty(void); float efc_get_v_pwm(void); float efc_get_v_ocxo(void); uint32_t efc_get_ccr1(void);
typedef struct{ float duty_cal; float V_cal; float slope_Hz_per_V; uint32_t crc; } CalData;
bool eeprom_load(CalData& out); void eeprom_save(const CalData& in); void eeprom_reset(void); bool eeprom_has_valid(void);
