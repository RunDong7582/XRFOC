#ifndef __MC_ADAPTOR_H
#define __MC_ADAPTOR_H

#ifdef __cplusplus
 extern "C" {
#endif

/**
 *  @file:      XRFOC_Lib/include/mc_adaptor.h
 *  @brief:     Multi platform support.
 *  @author:    RunDong7582
 *  @date  :    2025 3/21 16:47 -> 2025 4/1 16:26
 *  @version:   XRFOC v0.2
 */

#include <stdint.h>

typedef int (*mc_adaptor_clk_init_fn_t)         (void *mcu, uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq);         
typedef int (*mc_adaptor_power_init_fn_t)       (void *mcu);
typedef int (*mc_adaptor_power_on_off_fn_t)     (void *mcu, int state);

typedef int (*mc_adaptor_pwm_init_fn_t)         (void *mcu, uint16_t arr, uint16_t psc);
typedef int (*mc_adaptor_pwm_set_duty_fn_t)     (void *mcu, uint8_t channel, float duty);

typedef int (*mc_adaptor_adc_init_fn_t)         (void *mcu);
typedef int (*mc_adaptor_adc_read_fn_t)         (void *mcu, uint8_t channel, float *value);

typedef int (*mc_adaptor_timer_init_fn_t)       (void *mcu, uint16_t arr, uint16_t psc);
typedef int (*mc_adaptor_timer_start_fn_t)      (void *mcu);

typedef int (*mc_adaptor_uart_init_fn_t)        (void *mcu, uint32_t baudrate);

struct mc_adaptor_i {
    /* CLK Tree operation */
    mc_adaptor_clk_init_fn_t        clk_init;
    /* power operation */
    mc_adaptor_power_init_fn_t      power_init;
    mc_adaptor_power_on_off_fn_t    power_on_off;
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
static inline int mc_clk_init (void *mcu, uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq) {
    return ((struct mc_adaptor_i *)mcu)->clk_init(mcu, plln, pllm, pllp, pllq);
}
static inline int mc_power_init (void *mcu) {
    return ((struct mc_adaptor_i *)mcu)->power_init(mcu);
}

static inline int mc_power_on_off (void *mcu, int state) {
    return ((struct mc_adaptor_i *)mcu)->power_on_off(mcu, state);
}

static inline int mc_pwm_init (void *mcu, uint16_t arr, uint16_t psc) {
    return ((struct mc_adaptor_i *)mcu)->pwm_init(mcu, arr, psc);
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

static inline int mc_timer_init (void *mcu, uint16_t arr, uint16_t psc ) {
    return ((struct mc_adaptor_i *)mcu)->timer_init(mcu, arr, psc);
}

static inline int mc_timer_start (void *mcu) {
    return ((struct mc_adaptor_i *)mcu)->timer_start(mcu);
}

static inline int mc_uart_init (void *mcu, uint32_t baudrate) {
    return ((struct mc_adaptor_i *)mcu)->uart_init(mcu, baudrate);
}

#endif 
