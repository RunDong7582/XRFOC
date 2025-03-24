#ifndef __MC_ADAPTOR_H
#define __MC_ADAPTOR_H

#ifdef __cplusplus
 extern "C" {
#endif

/**
 *  @file:      XRFOC_Lib/include/mc_adaptor.h
 *  @brief:     Multi platform support.
 *  @author:    RunDong7582
 *  @date  :    2025 3/21 16:47
 *  @version:   XRFOC v0.1
 */

#include <stdint.h>

 
#define STM32
#define STM32F0

#ifdef  STM32
    #ifdef  STM32F0
    #include "stm32f0xx_hal.h"
    #include "../CMSIS/include/stm32f030x8.h"
    #elif   STM32F1
    #include "stm32_f1xx_hal.h"
    #elif   STM32F4
    #include "stm32_f4xx_hal.h"
    #elif   STM32G0
    #include "stm32_g0xx_hal.h"
    #elif   STM32G4
    #include "stm32_g4xx_hal.h"
    #endif

#elif defined(ESP32)

#endif

/* ------- MCU Periphal & GPIO -------- */
#ifdef      STM32

#define Motor_Enable_Pin        GPIO_PIN_4
#define Motor_Enable_GPIO_Port  GPIOC
#define     PWM_TIM         TIM1
#define     PWM_A_CH        TIM_CHANNEL_1
// #define     PWM_A_CH_N      TIM_CHANNEL_2

#define     PWM_B_CH        TIM_CHANNEL_2
// #define     PWM_B_CH_N      TIM_CHANNEL_4

#define     PWM_C_CH        TIM_CHANNEL_3
// #define     PWM_C_CH_N      TIM_CHANNEL_6


#define     TIM_CNT         TIM6

#define     CURRENT_A_ADC_CH   ADC_CHANNEL_0
#define     CURRENT_B_ADC_CH   ADC_CHANNEL_1
#define     CURRENT_C_ADC_CH   ADC_CHANNEL_2

#endif 

#ifdef  ESP32
#define M1_PWM_A 26
#define M1_PWM_B 27 
#define M1_PWM_C 14
#endif


/* ----  Periphal-Oriented typedef ---- */
typedef int (*mc_adaptor_button_init_fn_t)      (void *mcu);
typedef int (*mc_adaptor_button_on_off_fn_t)    (void *mcu, int state);

typedef int (*mc_adaptor_pwm_init_fn_t)         (void *mcu, uint32_t freq, uint8_t resolution);
typedef int (*mc_adaptor_pwm_set_duty_fn_t)     (void *mcu, uint8_t channel, float duty);

typedef int (*mc_adaptor_adc_init_fn_t)         (void *mcu);
typedef int (*mc_adaptor_adc_read_fn_t)         (void *mcu, uint8_t channel, float *value);

typedef int (*mc_adaptor_timer_init_fn_t)       (void *mcu, uint32_t freq);
typedef int (*mc_adaptor_timer_start_fn_t)      (void *mcu);

typedef int (*mc_adaptor_uart_init_fn_t)        (void *mcu, uint32_t baudrate);

struct mc_adaptor_i {
    /* BUTTON operation */
    mc_adaptor_button_init_fn_t     button_init;
    mc_adaptor_button_on_off_fn_t   button_on_off;
    /* PWM  operation */
    mc_adaptor_pwm_init_fn_t        pwm_init;
    mc_adaptor_pwm_set_duty_fn_t    pwm_set_duty;      
    /* ADC  operation */
    mc_adaptor_adc_init_fn_t        adc_init;
    mc_adaptor_adc_read_fn_t        adc_read;
    /* TIMER operation */
    mc_adaptor_timer_init_fn_t      timer_init;
    mc_adaptor_timer_start_fn_t     timer_start;
    /* Uart operation */
    mc_adaptor_uart_init_fn_t       uart_init;
};

/* 内联函数封装调用 */
static inline int mc_button_init (void *mcu) {
    return ((struct mc_adaptor_i *)mcu)->button_init(mcu);
}

static inline int mc_button_on_off (void *mcu, int state) {
    return ((struct mc_adaptor_i *)mcu)->button_on_off(mcu, state);
}

static inline int mc_pwm_init (void *mcu, uint32_t freq, uint8_t resolution) {
    return ((struct mc_adaptor_i *)mcu)->pwm_init(mcu, freq, resolution);
}

static inline int mc_pwm_set_duty (void *mcu, uint8_t channel, float duty) {
    return ((struct mc_adaptor_i *)mcu)->pwm_set_duty(mcu, channel, duty);
}

static inline int mc_adc_init (void *mcu) {
    return ((struct mc_adaptor_i *)mcu)->adc_init(mcu);
}

static inline int mc_adc_read (void *mcu, uint8_t channel, float *value) {
    return ((struct mc_adaptor_i *)mcu)->adc_read(mcu, channel, value);
}

static inline int mc_timer_init (void *mcu, uint32_t freq) {
    return ((struct mc_adaptor_i *)mcu)->timer_init(mcu, freq);
}

static inline int mc_timer_start (void *mcu) {
    return ((struct mc_adaptor_i *)mcu)->timer_start(mcu);
}

static inline int mc_uart_init (void *mcu, uint32_t baudrate) {
    return ((struct mc_adaptor_i *)mcu)->uart_init(mcu, baudrate);
}

// static inline int mc_adaptor_pwm_config   (void *mcu) 
// {
//     return (*(struct mc_adaptor_i **)mcu)->pwm_config(mcu);
// }

// static inline int mc_adaptor_pwm_duty_set (void *mcu, float duty) 
// {
//     return (*(struct mc_adaptor_i **)mcu)->pwm_dutyset(mcu, duty);
// }

// static inline int mc_adaptor_adc_config   (void *mcu) 
// {
//     return (*(struct mc_adaptor_i **)mcu)->adc_config(mcu);
// }

// static inline int mc_adaptor_adc_curr_read (void *mcu) 
// {
//     return (*(struct mc_adaptor_i **)mcu)->adc_read(mcu);
// }



#endif 