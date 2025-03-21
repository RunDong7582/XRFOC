/**
 *  @file:      XRFOC_Lib/mcu_conf/stm32/mc_stm32_conf.c
 *  @brief:     The Periphal Configuration for STM32 chip series.
 *  @author:    RunDong7582
 *  @date  :    2025 2/19 -> 2025 3/21 16:11
 *  @version:   XRFOC v0.1
 */


#include "../../include/mc_adaptor.h"

extern float voltage_power_supply;

static int pwm_config  ( struct mc_adaptor_stm32_periphal *self);
static int pwm_dutyset ( struct mc_adaptor_stm32_periphal *self, float duty);
static int adc_config  ( struct mc_adaptor_stm32_periphal *self); 
static int adc_read    ( struct mc_adaptor_stm32_periphal *self);

int mc_adaptor_stm32_pwm_init ( struct mc_adaptor_stm32_periphal *self )
{
}


void PWM_SET(float Ua, float Ub, float Uc) 
{
    /* constrain the three-phase voltage to [0, voltage_power_supply]  */
    Ua = _constrain(Ua, 0.0f, voltage_power_supply);
    Ub = _constrain(Ub, 0.0f, voltage_power_supply);
    Uc = _constrain(Uc, 0.0f, voltage_power_supply);

    /*  calculate duty  */
    /*  constrain the duty to [0, 1] */
    float dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
    float dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
    float dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);
  
//     //写入PWM到PWM 0 1 2 通道
//     ledcWrite(3, dc_a * 255);
//     ledcWrite(4, dc_b * 255);
//     ledcWrite(5, dc_c * 255);
}
