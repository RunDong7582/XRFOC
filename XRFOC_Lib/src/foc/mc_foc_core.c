#include "mc_foc_core.h"

#ifdef STM32_PLATFORM
#include "stm32_pwm.c"
#elif defined(ESP32_PLATFORM)
#include "esp32_pwm.c"
#endif


Motor motor;
PWM_Interface *pwm = &stm32_pwm;
ADC_Interface *adc = &stm32_adc;
Motor_Init(&motor, pwm, adc);

// foc.c
void FOC_Update(Motor *motor) {
    Clarke_Transform(ia, ib, &i_alpha, &i_beta);
    Park_Transform(i_alpha, i_beta, theta, &id, &iq);
    PID_Update(&motor->pid_id, id, target_id);
    PID_Update(&motor->pid_iq, iq, target_iq);
    SVM_Generate(vd, vq, theta, &duty_a, &duty_b);
    motor->pwm->set_duty(PHASE_A, duty_a);
    // ...
}