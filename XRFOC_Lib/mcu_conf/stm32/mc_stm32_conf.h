#ifndef __MC_STM32_CONF_H
#define __MC_STM32_CONF_H

#include "../../include/mc_adaptor.h"

/* MCU_STM32 Periphal Struct */ 
typedef struct {
    
    GPIO_InitTypeDef    gpio_config;
    TIM_HandleTypeDef   htim;
    ADC_HandleTypeDef   hadc;
    UART_HandleTypeDef  huart;
            uint32_t    pwm_freq;
            uint8_t     pwm_resolution;
} mc_stm32_hw_t;

struct mc_adaptor_stm32_periphal {
    struct mc_adaptor_i *adaptor;
};

#endif 