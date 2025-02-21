#include "mc_motor.h"
#include "mc_foc_core.h"

void Motor_Init(Motor *motor, PWM_Interface *pwm, ADC_Interface *adc) {
    motor->pwm = pwm;
    motor->adc = adc;
    motor->pwm->init();
    // 其他初始化
}

void Motor_Run(Motor *motor, float target_speed) {
    FOC_Update(motor);
}