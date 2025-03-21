#ifndef __MC_ADAPTOR_H
#define __MC_ADAPTOR_H

/**
 *  @file:      XRFOC_Lib/include/mc_adaptor.h
 *  @brief:     Multi platform support.
 *  @author:    RunDong7582
 *  @date  :    2025 3/21 16:47
 *  @version:   XRFOC v0.1
 */

/* ----  Periphal-Oriented typedef ---- */
typedef int (*mc_adaptor_pwm_config_fn_t)     (void *mcu);
typedef int (*mc_adaptor_pwm_duty_set_fn_t)   (void *mcu, float point);
typedef int (*mc_adaptor_adc_config_fn_t)     (void *mcu);
typedef int (*mc_adaptor_adc_curr_read_fn_t)  (void *mcu);

struct mc_adaptor_i {
    mc_adaptor_pwm_config_fn_t    pwm_config;
    mc_adaptor_pwm_duty_set_fn_t  pwm_dutyset;      
    mc_adaptor_adc_config_fn_t    adc_config;
    mc_adaptor_adc_curr_read_fn_t adc_read;      
};

static inline int mc_adaptor_pwm_config   (void *mcu) 
{
    return (*(struct mc_adaptor_i **)mcu)->pwm_config(mcu);
}

static inline int mc_adaptor_pwm_duty_set (void *mcu, float duty) 
{
    return (*(struct mc_adaptor_i **)mcu)->pwm_dutyset(mcu, duty);
}

static inline int mc_adaptor_adc_config   (void *mcu) 
{
    return (*(struct mc_adaptor_i **)mcu)->adc_config(mcu);
}

static inline int mc_adaptor_adc_curr_read (void *mcu) 
{
    return (*(struct mc_adaptor_i **)mcu)->adc_read(mcu);
}

/* ------- MCU Periphal & GPIO -------- */ 
#ifdef STM32

#include "stm32_f0.c"
#define M0_PWMA_TIM   TIM1
#define M0_PWMA_CH    TIM_CHANNEL_1

#define CURRENT_A_ADC_CH   ADC_CHANNEL_0
#define CURRENT_B_ADC_CH   ADC_CHANNEL_1

#elif defined(ESP32)
#include "esp32_pwm.c"
#define M1_PWM_A 26
#define M1_PWM_B 27 
#define M1_PWM_C 14

#define PWM_Freq 30000
#define PWM_Arr_bit 8

#endif

#endif 