/**
 *  @file:      XRFOC_Lib/mcu_conf/stm32/mc_stm32_conf.c
 *  @brief:     The Periphal Configuration for STM32 chip series.
 *  @author:    RunDong7582
 *  @date  :    2026 3/19 18:00
 *  @version:   XRFOC v1.0
 */

 #include "mc_stm32_conf.h"
 
extern TIM_HandleTypeDef timx_handler;        
extern TIM_HandleTypeDef  g_atimx_handle;
extern TIM_OC_InitTypeDef g_atimx_oc_chy_handle;  
extern UART_HandleTypeDef g_uart1_handle; 
static int mt_clk_init_stm32( struct mc_adaptor_stm32_hw *mcu, uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq);
static int mt_power_init_stm32( struct mc_adaptor_stm32_hw *mcu );
static int mt_pwm_init_stm32( struct mc_adaptor_stm32_hw *mcu, uint16_t arr, uint16_t psc );
//  static int mt_timer_init_stm32( struct mc_adaptor_stm32_hw *mcu, uint16_t arr, uint16_t psc);
static int mt_uart_init_stm32 (struct mc_adaptor_stm32_hw *mcu , uint32_t baudrate );

static struct mc_adaptor_i adaptor_interface = {
     .clk_init       = (mc_adaptor_clk_init_fn_t)mt_clk_init_stm32,
    .uart_init      = (mc_adaptor_uart_init_fn_t)mt_uart_init_stm32,
     .power_init     = (mc_adaptor_power_init_fn_t)mt_power_init_stm32,
     .pwm_init       = (mc_adaptor_pwm_init_fn_t)mt_pwm_init_stm32,
    //  .timer_init     = (mc_adaptor_timer_init_fn_t)mt_timer_init_stm32,
 };

 int mc_adaptor_stm32_hw_init (struct mc_adaptor_stm32_hw *mcu)
 {
     mcu->adaptor = &adaptor_interface;
     return 0;
 }

 static int mt_clk_init_stm32 ( struct mc_adaptor_stm32_hw *mcu, uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq) {
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

 static int mt_uart_init_stm32 (struct mc_adaptor_stm32_hw *mcu , uint32_t baudrate ) {
    HAL_StatusTypeDef ret = HAL_OK;
    g_uart1_handle.Instance = USART1;                           /* USART1 */
    g_uart1_handle.Init.BaudRate = baudrate;                    /* 波特率 */
    g_uart1_handle.Init.WordLength = UART_WORDLENGTH_8B;        /* 字长为8位数据格式 */
    g_uart1_handle.Init.StopBits = UART_STOPBITS_1;             /* 一个停止位 */
    g_uart1_handle.Init.Parity = UART_PARITY_NONE;              /* 无奇偶校验位 */
    g_uart1_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;        /* 无硬件流控 */
    g_uart1_handle.Init.Mode = UART_MODE_TX_RX;                 /* 收发模式 */
    ret = HAL_UART_Init(&g_uart1_handle);                       /* HAL_UART_Init()会使能UART1 */
    if(ret != HAL_OK)
    {
        return 1;                                               /* 串口初始化失败 */
    }
    return 0;
 }

 static int mt_power_init_stm32 (struct mc_adaptor_stm32_hw *mcu ) {
    GPIO_InitTypeDef gpio_init_struct;
    
    SHUTDOWN_PIN_GPIO_CLK_ENABLE();
    SHUTDOWN2_PIN_GPIO_CLK_ENABLE();
  
    gpio_init_struct.Pin = SHUTDOWN_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SHUTDOWN_PIN_GPIO, &gpio_init_struct);    
    
    gpio_init_struct.Pin = SHUTDOWN2_PIN;
    HAL_GPIO_Init(SHUTDOWN2_PIN_GPIO, &gpio_init_struct);  
    return 0;
 }

 /* PWM Initialize */
 static int mt_pwm_init_stm32 ( struct mc_adaptor_stm32_hw *mcu, uint16_t arr, uint16_t psc ) {
    HAL_StatusTypeDef ret = HAL_OK;
    __HAL_RCC_TIM1_CLK_ENABLE();        /* TIMX 时钟使能 */
    g_atimx_handle.Instance = TIM1;                    /* 定时器x */
    g_atimx_handle.Init.Prescaler = psc;                        /* 定时器分频 */
    g_atimx_handle.Init.CounterMode = TIM_COUNTERMODE_UP;       /* 向上计数模式 */
    g_atimx_handle.Init.Period = arr;                           /* 自动重装载值 */
    g_atimx_handle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;   /* 分频因子 */
    g_atimx_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; /*使能TIMx_ARR进行缓冲*/
    g_atimx_handle.Init.RepetitionCounter = 0;                  /* 开始时不计数*/
    ret = HAL_TIM_PWM_Init(&g_atimx_handle);                          /* 初始化PWM */
    if(ret != HAL_OK)
    {
        return 1;                                               /* TIM1初始化失败 */
    }

    g_atimx_oc_chy_handle.OCMode = TIM_OCMODE_PWM1;             /* 模式选择PWM1 */
    g_atimx_oc_chy_handle.Pulse = 0;
    g_atimx_oc_chy_handle.OCPolarity = TIM_OCPOLARITY_HIGH;     /* 输出比较极性为低 */
    g_atimx_oc_chy_handle.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    g_atimx_oc_chy_handle.OCFastMode = TIM_OCFAST_DISABLE;
    g_atimx_oc_chy_handle.OCIdleState = TIM_OCIDLESTATE_RESET;
    g_atimx_oc_chy_handle.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&g_atimx_handle, &g_atimx_oc_chy_handle, ATIM_TIMX_PWM_CH1); /* 配置TIMx通道y */
    HAL_TIM_PWM_ConfigChannel(&g_atimx_handle, &g_atimx_oc_chy_handle, ATIM_TIMX_PWM_CH2); /* 配置TIMx通道y */   
    HAL_TIM_PWM_ConfigChannel(&g_atimx_handle, &g_atimx_oc_chy_handle, ATIM_TIMX_PWM_CH3); /* 配置TIMx通道y */

    /* 配置触发源 */
//    HAL_TIMEx_ConfigCommutationEvent(&g_atimx_handle, TIM_TS_ITR0, TIM_COMMUTATION_SOFTWARE);  /* 内部触发配置(TIM1->ITR0->TIM5) */ 
    
    /* 开启定时器通道1输出PWM */
    HAL_TIM_PWM_Start(&g_atimx_handle,TIM_CHANNEL_1);

    /* 开启定时器通道2输出PWM */
    HAL_TIM_PWM_Start(&g_atimx_handle,TIM_CHANNEL_2);

    /* 开启定时器通道3输出PWM */
    HAL_TIM_PWM_Start(&g_atimx_handle,TIM_CHANNEL_3);
    
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 2, 2);
    // HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    return 0;
 }
 
//  static int mt_timer_init_stm32 (struct mc_adaptor_stm32_hw *mcu, uint16_t arr, uint16_t psc) {

//     timx_handler.Instance = TIM6;                      /* 通用定时器X */
//     timx_handler.Init.Prescaler = psc;                          /* 设置预分频器  */
//     timx_handler.Init.CounterMode = TIM_COUNTERMODE_UP;         /* 向上计数器 */
//     timx_handler.Init.Period = arr;                             /* 自动装载值 */
//     timx_handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;   /* 时钟分频因子 */
//     HAL_TIM_Base_Init(&timx_handler);

//     HAL_TIM_OnePulse_Init(&timx_handler, TIM_OPMODE_SINGLE);
//     HAL_TIM_Base_Start_IT(&timx_handler);                       /* 使能通用定时器x和及其更新中断：TIM_IT_UPDATE */
//     __HAL_TIM_ENABLE_IT(&timx_handler,TIM_IT_UPDATE);           /* 清除更新中断标志位 */
//     return 0;
//  }
 
 
 