/**
 *  @file:      XRFOC_Lib/mcu_conf/stm32/mc_stm32_conf.c
 *  @brief:     The Periphal Configuration for STM32 chip series.
 *  @author:    RunDong7582
 *  @date  :    2025 2/19 -> 2025 3/21 16:11
 *  @version:   XRFOC v0.1
 */

#include "mc_stm32_conf.h"

/* Button Initialize */
static int stm32_button_init (void *mcu) {
    mc_stm32_hw_t *hw = (mc_stm32_hw_t *)mcu;
    GPIO_InitTypeDef gpio_config = {0};    
    hw->gpio_config.Pin = Motor_Enable_Pin;
    hw->gpio_config.Mode = GPIO_MODE_OUTPUT_PP;
    hw->gpio_config.Pull = GPIO_NOPULL;
    hw->gpio_config.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(Motor_Enable_GPIO_Port, &gpio_config);
    return 0;
}

static int stm32_button_on_off (void *mcu, int state) {
    mc_stm32_hw_t *hw = (mc_stm32_hw_t *)mcu;
    HAL_GPIO_WritePin(Motor_Enable_GPIO_Port, Motor_Enable_Pin, state);
    return 0;
}
/* PWM Initialize */
static int stm32_pwm_init (void *mcu, uint32_t freq, uint8_t resolution) {
    mc_stm32_hw_t *hw = (mc_stm32_hw_t *)mcu;
    hw->pwm_freq = freq;
    hw->pwm_resolution = resolution;

    /* Set TIM as PWM(N)*/
    TIM_OC_InitTypeDef oc_config = {0};
    hw->htim.Instance = TIM1;
    hw->htim.Init.Prescaler = SystemCoreClock / (freq * (1 << resolution)) - 1;
    hw->htim.Init.CounterMode = TIM_COUNTERMODE_UP;
    hw->htim.Init.Period = (1 << resolution) - 1;
    HAL_TIM_PWM_Init(&hw->htim);

    oc_config.OCMode = TIM_OCMODE_PWM1;
    oc_config.Pulse  = 0; // 0% duty , but in theory should be 50% duty.
    HAL_TIM_PWM_ConfigChannel(&hw->htim, &oc_config, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&hw->htim, TIM_CHANNEL_1);
}

static int stm32_pwm_set_duty(void *mcu, uint8_t channel, float duty) {
    mc_stm32_hw_t *hw = (mc_stm32_hw_t *)mcu;
    uint32_t pulse = (uint32_t)(duty * (1 << hw->pwm_resolution));
    __HAL_TIM_SET_COMPARE(&hw->htim, channel, pulse);
    return 0;
}

static int stm32_adc_init (void *mcu) {
    mc_stm32_hw_t *hw = (mc_stm32_hw_t *)mcu;
    ADC_ChannelConfTypeDef adc_config = {0};
    hw->hadc.Instance = ADC1;
    hw->hadc.Init.Resolution = ADC_RESOLUTION_12B;
    HAL_ADC_Init(&hw->hadc);

    adc_config.Channel = ADC_CHANNEL_0;
    adc_config.Rank = 1;
    HAL_ADC_ConfigChannel(&hw->hadc, &adc_config);
    return 0;
}

static int stm32_adc_read (void *mcu, uint8_t channel, float *value) {
    mc_stm32_hw_t *hw = (mc_stm32_hw_t *)mcu;
    HAL_ADC_Start(&hw->hadc);
    if (HAL_ADC_PollForConversion(&hw->hadc, 100) == HAL_OK) {
        *value = HAL_ADC_GetValue(&hw->hadc);
        return 0;
    }
    return -1;
}

static int stm32_timer_init (void *mcu, uint32_t freq) {
    mc_stm32_hw_t *hw = (mc_stm32_hw_t *)mcu;
    hw->htim.Instance = TIM6;
    hw->htim.Init.Prescaler = SystemCoreClock / freq - 1; // 不一定是对的 ？
    hw->htim.Init.CounterMode = TIM_COUNTERMODE_UP;
    hw->htim.Init.Period = 1;
    HAL_TIM_Base_Init(&hw->htim);
    HAL_TIM_Base_Start_IT(&hw->htim);
    return 0;
}

static int stm32_timer_start_it (void *mcu) {
    mc_stm32_hw_t *hw = (mc_stm32_hw_t *)mcu;
    HAL_TIM_Base_Start_IT(&hw->htim);
    return 0;
}

struct mc_adaptor_i stm32_adaptor = {
    .button_init = NULL,
    .button_on_off = stm32_button_on_off,
    .pwm_init = stm32_pwm_init,
    .pwm_set_duty = stm32_pwm_set_duty,
    .adc_init = stm32_adc_init,
    .adc_read = stm32_adc_read,
    .timer_init = stm32_timer_init,
    .timer_start = stm32_timer_start_it
};