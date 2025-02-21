/**
 *  @filename:: mc_stm32f030r8t6_conf.c
 *  @details:   this file is configured for corresponding chip
 *  @author:    XR
 *  @date  :    2025 2/19
 *  @version:   XRFOC 1.0
 */


#include "mc_hal_pwm.h"

static void STM32_PWM_Init() {
    // 初始化TIM1
}

static void STM32_SetDuty(uint8_t phase, float duty) {
    // 更新CCR
}

PWM_Interface stm32_pwm = {
    .init = STM32_PWM_Init,
    .set_duty = STM32_SetDuty
};