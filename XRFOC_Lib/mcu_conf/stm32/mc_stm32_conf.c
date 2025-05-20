/**
 *  @file:      XRFOC_Lib_on_stm32/mcu_conf/stm32/mc_stm32_conf.c
 *  @brief:     The Periphal Configuration for STM32 chip series.
 *  @author:    RunDong7582
 *  @date  :    2025.5.19 18:39
 *  @version:   XRFOC release version 
 */

 #include "mc_stm32_conf.h"
 
static int mt_clk_init_stm32( struct mc_adaptor_stm32_hw *mcu, uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq);
static int mt_power_init_stm32( struct mc_adaptor_stm32_hw *mcu );
static int mt_pwm_init_stm32( struct mc_adaptor_stm32_hw *mcu, uint16_t arr, uint16_t psc );
static int mt_uart_init_stm32 (struct mc_adaptor_stm32_hw *mcu , uint32_t baudrate );

static struct mc_adaptor_i adaptor_interface = {
     .clk_init       = (mc_adaptor_clk_init_fn_t)mt_clk_init_stm32,
     .uart_init      = (mc_adaptor_uart_init_fn_t)mt_uart_init_stm32
     .power_init     = (mc_adaptor_power_init_fn_t)mt_power_init_stm32,
     .pwm_init       = (mc_adaptor_pwm_init_fn_t)mt_pwm_init_stm32,
 };

int mc_adaptor_stm32_hw_init (struct mc_adaptor_stm32_hw *mcu)
{
    mcu->adaptor = &adaptor_interface;
    return 0;
}

static int mt_clk_init_stm32 ( struct mc_adaptor_stm32_hw *mcu, uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq) {
    return 0;   
}

static int mt_uart_init_stm32 (struct mc_adaptor_stm32_hw *mcu , uint32_t baudrate ) {
    return 0;
}

static int mt_power_init_stm32 (struct mc_adaptor_stm32_hw *mcu ) {
    return 0;
}

/* PWM Initialize */
static int mt_pwm_init_stm32 ( struct mc_adaptor_stm32_hw *mcu, uint16_t arr, uint16_t psc ) {
    return 0;
}
 
 
 
 