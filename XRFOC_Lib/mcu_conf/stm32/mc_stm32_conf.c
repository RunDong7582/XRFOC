/**
 *  @file:      XRFOC_Lib/mcu_conf/stm32/mc_stm32_conf.c
 *  @brief:     The Periphal Configuration for STM32 chip series.
 *  @author:    RunDong7582
 *  @date  :    2025 3/21 16:11 -> 2025 4/1 16:28
 *  @version:   XRFOC v0.2
 */

 #include "mc_stm32_conf.h"

 /* Button Initialize */
 static int mt_clk_init_stm32( struct mc_adaptor_stm32_hw *mcu, uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq);
 static int mt_power_init_stm32( struct mc_adaptor_stm32_hw *mcu );
 static int mt_power_on_off_stm32( struct mc_adaptor_stm32_hw *mcu, int state );
 static int mt_pwm_init_stm32( struct mc_adaptor_stm32_hw *mcu, uint16_t arr, uint16_t psc );
 static int mt_pwm_set_duty_stm32( struct mc_adaptor_stm32_hw *mcu, uint8_t channel, float duty);
 static int mt_adc_init_stm32( struct mc_adaptor_stm32_hw *mcu);
 static int mt_adc_read_stm32( struct mc_adaptor_stm32_hw *mcu, uint8_t channel, float *value);
 static int mt_timer_init_stm32( struct mc_adaptor_stm32_hw *mcu, uint16_t arr, uint16_t psc);
 static int mt_timer_start_stm32( struct mc_adaptor_stm32_hw *mcu);
 
 static struct mc_adaptor_i adaptor_interface = {
     .clk_init       = (mc_adaptor_clk_init_fn_t)mt_clk_init_stm32,
     .power_init     = (mc_adaptor_power_init_fn_t)mt_power_init_stm32,
     .power_on_off   = (mc_adaptor_power_on_off_fn_t)mt_power_on_off_stm32,
     .pwm_init       = (mc_adaptor_pwm_init_fn_t)mt_pwm_init_stm32,
     .pwm_set_duty   = (mc_adaptor_pwm_set_duty_fn_t)mt_pwm_set_duty_stm32,
     .adc_init       = (mc_adaptor_adc_init_fn_t)mt_adc_init_stm32,
     .adc_read       = (mc_adaptor_adc_read_fn_t)mt_adc_read_stm32,
     .timer_init     = (mc_adaptor_timer_init_fn_t)mt_timer_init_stm32,
     .timer_start    = (mc_adaptor_timer_start_fn_t)mt_timer_start_stm32 
 };
 
 static int mt_clk_init_stm32 ( struct mc_adaptor_stm32_hw *mcu, uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq)
 {
    
 }
 
 static int mt_power_init_stm32 (struct mc_adaptor_stm32_hw *mcu ) {
     return 0;
 }
 
 static int mt_power_on_off_stm32 ( struct mc_adaptor_stm32_hw *mcu, int state ) {
     return 0;
 }
 /* PWM Initialize */
 static int mt_pwm_init_stm32 ( struct mc_adaptor_stm32_hw *mcu, uint16_t arr, uint16_t psc ) {
     return 0;
 }
 
 static int mt_pwm_set_duty_stm32 ( struct mc_adaptor_stm32_hw *mcu, uint8_t channel, float duty ) {
     return 0;
 }
 
 static int mt_adc_init_stm32 (struct mc_adaptor_stm32_hw *mcu) {
     return 0;
 }
 
 static int mt_adc_read_stm32 (struct mc_adaptor_stm32_hw *mcu, uint8_t channel, float *value) {
 }
 
 static int mt_timer_init_stm32 (struct mc_adaptor_stm32_hw *mcu, uint16_t arr, uint16_t psc) {
     return 0;
 }
 
 static int mt_timer_start_stm32 (struct mc_adaptor_stm32_hw *mcu) {
     return 0;
 }
 
 int mc_adaptor_stm32_hw_init (struct mc_adaptor_stm32_hw *mcu)
 {
     mcu->adaptor = &adaptor_interface;
     return 0;
 }
 
 