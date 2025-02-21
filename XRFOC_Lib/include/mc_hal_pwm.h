#ifndef __MC_HAL_PWM_H
#define __MC_HAL_PWM_H

#include "mc_common.h"

struct mc_hal_pwm_i{
    void (*mc_hal_pwm_init)(void);
    void (*mc_hal_pwm_set_duty)(uint8_t phase, float duty);
} PWM_Interface;

typedef void (*mc_hal_interface_fn) (uint8_t phase, float duty);

#endif // MC_HAL_PWM_H