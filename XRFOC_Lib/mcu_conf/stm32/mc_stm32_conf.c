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
     HAL_StatusTypeDef ret = HAL_OK;
     RCC_ClkInitTypeDef rcc_clk_init_handle;
     RCC_OscInitTypeDef rcc_osc_init_handle;
     
     __HAL_RCC_PWR_CLK_ENABLE();                                         /* 使能PWR时钟 */
     
     /* 下面这个设置用来设置调压器输出电压级别，以便在器件未以最大频率工作时使性能与功耗实现平衡 */
 
     __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);      /* VOS = 1, Scale1, 1.2V内核电压,FLASH访问可以得到最高性能 */
 
     /* 使能HSE，并选择HSE作为PLL时钟源，配置PLL1，开启USB时钟 */
     rcc_osc_init_handle.OscillatorType = RCC_OSCILLATORTYPE_HSE;        /* 时钟源为HSE */
     rcc_osc_init_handle.HSEState = RCC_HSE_ON;                          /* 打开HSE */
     rcc_osc_init_handle.PLL.PLLState = RCC_PLL_ON;                      /* 打开PLL */
     rcc_osc_init_handle.PLL.PLLSource = RCC_PLLSOURCE_HSE;              /* PLL时钟源选择HSE */
     rcc_osc_init_handle.PLL.PLLN = plln;
     rcc_osc_init_handle.PLL.PLLM = pllm;
     rcc_osc_init_handle.PLL.PLLP = pllp;
     rcc_osc_init_handle.PLL.PLLQ = pllq;
 
     ret=HAL_RCC_OscConfig(&rcc_osc_init_handle);                        /*初始化RCC*/
     if(ret != HAL_OK)
     {
         return 1;                                                       /* 时钟初始化失败，可以在这里加入自己的处理 */
     }
 
     /* 选中PLL作为系统时钟源并且配置HCLK,PCLK1和PCLK2*/
     rcc_clk_init_handle.ClockType = (RCC_CLOCKTYPE_SYSCLK \
                                     | RCC_CLOCKTYPE_HCLK \
                                     | RCC_CLOCKTYPE_PCLK1 \
                                     | RCC_CLOCKTYPE_PCLK2);
 
     rcc_clk_init_handle.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;         /* 设置系统时钟时钟源为PLL */
     rcc_clk_init_handle.AHBCLKDivider = RCC_SYSCLK_DIV1;                /* AHB分频系数为1 */
     rcc_clk_init_handle.APB1CLKDivider = RCC_HCLK_DIV4;                 /* APB1分频系数为4 */
     rcc_clk_init_handle.APB2CLKDivider = RCC_HCLK_DIV2;                 /* APB2分频系数为2 */
 
     ret = HAL_RCC_ClockConfig(&rcc_clk_init_handle, FLASH_LATENCY_5);   /* 同时设置FLASH延时周期为5WS，也就是6个CPU周期 */
     if(ret != HAL_OK)
     {
         return 1;                                                       /* 时钟初始化失败 */
     }
     
     /* STM32F405x/407x/415x/417x Z版本的器件支持预取功能 */
     if (HAL_GetREVID() == 0x1001)
     {
         __HAL_FLASH_PREFETCH_BUFFER_ENABLE();                           /* 使能flash预取 */
     }
     return 0;
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
 
 