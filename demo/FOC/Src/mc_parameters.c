
/**
  ******************************************************************************
  * @file    mc_parameters.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides definitions of HW parameters specific to the
  *          configuration of the subsystem.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "parameters_conversion.h"
#include "r3_f0xx_pwm_curr_fdbk.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

#define FREQ_RATIO 1                /* Dummy value for single drive */
#define FREQ_RELATION HIGHEST_FREQ  /* Dummy value for single drive */
extern  PWMC_R3_1_Handle_t PWM_Handle_M1;

/**
  * @brief  Current sensor parameters Single Drive - three shunt, STM32F0X
  */
const R3_1_Params_t R3_1_Params =
{
/* Current reading A/D Conversions initialization -----------------------------*/
  .b_ISamplingTime =  LL_ADC_SAMPLINGTIME_13CYCLES_5,

/* PWM generation parameters --------------------------------------------------*/
  .hDeadTime = DEAD_TIME_COUNTS,
  .RepetitionCounter = REP_COUNTER,
  .hTafter = TW_AFTER,
  .hTbefore = TW_BEFORE_R3_1,
  .TIMx = TIM1,

/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs= (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,

  .ADCConfig = {
                 1<< MC_ADC_CHANNEL_1| 1<<MC_ADC_CHANNEL_0,
                 1<< MC_ADC_CHANNEL_2| 1<<MC_ADC_CHANNEL_0,
                 1<< MC_ADC_CHANNEL_2| 1<<MC_ADC_CHANNEL_0,
                 1<< MC_ADC_CHANNEL_2| 1<<MC_ADC_CHANNEL_1,
                 1<< MC_ADC_CHANNEL_2| 1<<MC_ADC_CHANNEL_1,
                 1<< MC_ADC_CHANNEL_1| 1<<MC_ADC_CHANNEL_0,
  },
  .ADCScandir = {
   LL_ADC_REG_SEQ_SCAN_DIR_BACKWARD>>ADC_CFGR1_SCANDIR_Pos,
   LL_ADC_REG_SEQ_SCAN_DIR_BACKWARD>>ADC_CFGR1_SCANDIR_Pos,
   LL_ADC_REG_SEQ_SCAN_DIR_FORWARD>>ADC_CFGR1_SCANDIR_Pos,
   LL_ADC_REG_SEQ_SCAN_DIR_FORWARD>>ADC_CFGR1_SCANDIR_Pos,
   LL_ADC_REG_SEQ_SCAN_DIR_BACKWARD>>ADC_CFGR1_SCANDIR_Pos,
   LL_ADC_REG_SEQ_SCAN_DIR_FORWARD>>ADC_CFGR1_SCANDIR_Pos,
  },
  .ADCDataReg1 = {
               &PWM_Handle_M1.ADC1_DMA_converted[0],
               &PWM_Handle_M1.ADC1_DMA_converted[0],
               &PWM_Handle_M1.ADC1_DMA_converted[1],
               &PWM_Handle_M1.ADC1_DMA_converted[1],
               &PWM_Handle_M1.ADC1_DMA_converted[0],
               &PWM_Handle_M1.ADC1_DMA_converted[1],
  },

  .ADCDataReg2 = {
               &PWM_Handle_M1.ADC1_DMA_converted[1],
               &PWM_Handle_M1.ADC1_DMA_converted[1],
               &PWM_Handle_M1.ADC1_DMA_converted[0],
               &PWM_Handle_M1.ADC1_DMA_converted[0],
               &PWM_Handle_M1.ADC1_DMA_converted[1],
               &PWM_Handle_M1.ADC1_DMA_converted[0],
  },
};

/* USER CODE BEGIN Additional parameters */

/* USER CODE END Additional parameters */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
