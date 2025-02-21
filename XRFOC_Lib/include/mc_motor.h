#ifndef __MC_MOTOR_H
#define __MC_MOTOR_H

typedef struct {
    PWM_Interface *pwm;
    ADC_Interface *adc;
    PID_Controller pid_id;
    PID_Controller pid_iq;
    float est_angle;
    float est_speed;
} Motor;

void Motor_Init(Motor *motor, PWM_Interface *pwm, ADC_Interface *adc);
void Motor_Run(Motor *motor, float target_speed);

#endif 