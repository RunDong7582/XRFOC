#ifndef __MC_ADAPTOR_H
#define __MC_ADAPTOR_H

#ifdef __cplusplus
 extern "C" {
#endif

/**
 *  @file:      XRFOC_Lib/include/mc_adaptor.h
 *  @brief:     Multi platform support.
 *  @author:    RunDong7582
 *  @date  :    2025.5.19 18:30
 *  @version:   XRFOC release version
 */

#include <stdint.h>

typedef int (*mc_adaptor_clk_init_fn_t)         (void *mcu, uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq);         
typedef int (*mc_adaptor_uart_init_fn_t)        (void *mcu, uint32_t baudrate);
typedef int (*mc_adaptor_power_init_fn_t)       (void *mcu);
typedef int (*mc_adaptor_pwm_init_fn_t)         (void *mcu, uint16_t arr, uint16_t psc);
typedef int (*mc_adaptor_timer_init_fn_t)       (void *mcu, uint16_t arr, uint16_t psc);


struct mc_adaptor_i {
    /* CLK Tree operation */
    mc_adaptor_clk_init_fn_t        clk_init;
    /* Uart operation */
    mc_adaptor_uart_init_fn_t       uart_init;
    /* power operation */
    mc_adaptor_power_init_fn_t      power_init;
    /* PWM  operation */
    mc_adaptor_pwm_init_fn_t        pwm_init;  
    // /* TIMER operation */
    // mc_adaptor_timer_init_fn_t      timer_init;
};

/* 内联函数封装调用 */
static inline int mc_clk_init   (void *mcu, uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq) {
    return ((struct mc_adaptor_i *)mcu)->clk_init(mcu, plln, pllm, pllp, pllq);
}

static inline int mc_uart_init  (void *mcu, uint32_t baudrate) {
    return ((struct mc_adaptor_i *)mcu)->uart_init(mcu, baudrate);
}

static inline int mc_power_init (void *mcu) {
    return ((struct mc_adaptor_i *)mcu)->power_init(mcu);
}

static inline int mc_pwm_init   (void *mcu, uint16_t arr, uint16_t psc) {
    return ((struct mc_adaptor_i *)mcu)->pwm_init(mcu, arr, psc);
}

// static inline int mc_timer_init (void *mcu, uint16_t arr, uint16_t psc ) {
//     return ((struct mc_adaptor_i *)mcu)->timer_init(mcu, arr, psc);
// }


#endif 
