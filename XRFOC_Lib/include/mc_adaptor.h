#ifndef __MC_ADAPTOR_H
#define __MC_ADAPTOR_H

/* MCU Periphal & GPIO */ 

extern float voltage_power_supply;

#ifdef STM32
#define M0_PWMA_TIM   TIM1
#define M0_PWMA_CH    TIM_CHANNEL_1

#define CURRENT_A_ADC_CH   ADC_CHANNEL_0
#define CURRENT_B_ADC_CH   ADC_CHANNEL_1
#endif 

#ifdef ESP32

#define M1_PWM_A 26
#define M1_PWM_B 27 
#define M1_PWM_C 14

#define PWM_Freq 30000
#define PWM_Arr_bit 8

#endif

// void mc_pwm_init(void);
// void set_pwm_duty (TIM_HandleTypeDef *htim, uint32_t channel, float duty);
// float read_adc_voltage (ADC_HandleTypeDef *hadc, uint32_t channel);

#endif 