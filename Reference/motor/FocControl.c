#include "FocControl.h"
#include "FocConfig.h"
#include "FocObserver.h"
#include "FocComponentsSvpwm.h"
#include "arm_math.h" 
#include "PIController.h"
#include <string.h>
#include "tim.h"
#include "ADC.h"
#include "FocSample.h"
#include "FocStartUp.h"
#include "focDebug.h"
PIController_t gIDLoop;   //d轴电流环
PIController_t gIQLoop;   //q轴电流环
PIController_t gSpeedLoop;//速度环
ComponentsSvpwm_t gSvpwm; //svpwm结构体
FocSample_t gAdcSample;   //电流采样结构体
ifStartup_t gIfStartup;   //if启动结构体
ObserverSmc_t gObSmc;     //观测器结构体
control_t gFocMachine;    //foc状态机 


void CurrentLoopTest(void)
{
		static float time;
		time += FOC_PWM_PERIOD;
		if(time < 5.0f)
		{
			 gIfStartup.iq = 0.05f;
		}
		else if(time >= 5.0f && time < 10.0f)
		{
			gIfStartup.iq = 0.5f;
		}
		else
		{
			time = 0;
		}
}
/*******************************************************************************
 * @brief   foc主函数
 * @param   
 * @return  
 ******************************************************************************/
void FocMain(void)
{
	 BspInit();
	gFocMachine.sw = 1;
	gFocMachine.target = 0.5f;
	 for(;;)
		{
			DoControl(&gFocMachine);
			VofaGraphSend(2);
		}
}
/*******************************************************************************
 * @brief   外设初始化
 * @param   
 * @return  
 ******************************************************************************/
void BspInit(void)
{
	  TIM1->CCR4 = 4100;
		CurrentSampleInit(&gAdcSample);
}
/*******************************************************************************
 * @brief   对各个模块进行初始化
 * @param   
 * @return  
 ******************************************************************************/
void ControlInit(void)
{
	memset(&gIDLoop,0,sizeof(gIDLoop));
	memset(&gIQLoop,0,sizeof(gIQLoop));
	memset(&gSvpwm ,0,sizeof(gSvpwm));
	memset(&gIfStartup ,0,sizeof(gIfStartup));
	memset(&gObSmc ,0,sizeof(gObSmc));
	//D轴电流初始化
	gIDLoop.kp = Q15(CURRENT_ID_KP / R_BASE);
	gIDLoop.ki = Q15(CURRENT_ID_KI * FOC_PWM_PERIOD / R_BASE);
	gIDLoop.maxValue = Q15(CURRENT_ID_OUT_MAX / U_BASE);
	gIDLoop.minValue = Q15(CURRENT_ID_OUT_MIN / U_BASE);
	//Q轴电流初始化
	gIQLoop.kp = Q15(CURRENT_IQ_KP / R_BASE);
	gIQLoop.ki = Q15(CURRENT_ID_KI * FOC_PWM_PERIOD / R_BASE);
	gIQLoop.maxValue = Q15(CURRENT_IQ_OUT_MAX / U_BASE);
	gIQLoop.minValue = Q15(CURRENT_IQ_OUT_MIN / U_BASE);
	//观测器初始化
	ObserverSmcInit(&gObSmc);
	//svpwm初始化
	gSvpwm.vBus = Q15(12.0f / U_BASE);
	
	#if MOTOR_CONTORL_MODE == MOTOR_CONTORL_VOLTAGE_OPEN_LOOP //电压开环控制 id控制
	gIDLoop.maxValue = 24 * 0.5;
	gIDLoop.ki = 14 * 0.0005f;
	gIDLoop.kp = 0.010f * 0.0005f;
	gIDLoop.kc = 0;
	gSvpwm.vBus = 24.0;
  #endif 
}
/*******************************************************************************
 * @brief   ADC中断，foc执行周期
 * @param   
 * @return  
 ******************************************************************************/
void ADC_IRQHandler()
{
	
	if(__HAL_ADC_GET_FLAG(&hadc2,ADC_FLAG_JEOC) == SET)
	{
		__HAL_ADC_CLEAR_FLAG(&hadc2,ADC_FLAG_JEOC);
		TIM1->CCR4 = 4160;
		//采样电流 
		CurrentSampleStep(&gAdcSample,gSvpwm.sin_cos);
		//计算电流环
		gIDLoop.target = 0;
		gIDLoop.real = gAdcSample.iD;
//速度闭环模式
#if  MOTOR_CONTORL_SPEED_LOOP == MOTOR_CONTORL_MODE
		
#endif
//		gSpeedLoop.target = gFocMachine.target;
//		gSpeedLoop.real   =  gObSmc.oemga;
//		PIControllerStep(&gSpeedLoop);

		gIQLoop.target = gIfStartup.iq;
		gIQLoop.real = gAdcSample.iQ;		
		
		PIControllerStep(&gIQLoop);
		PIControllerStep(&gIDLoop);
		//估算角度
		ObserverSmcStep(&gObSmc,gSvpwm.vAlpha,gSvpwm.vBeta,gAdcSample.iAlpha,gAdcSample.iBeta);
		//if启动
		IfStartupStep(&gIfStartup,gObSmc.theta,&gSvpwm.sin_cos[0]);
		//gSvpwm.vBus = Q16(24.0f / U_BASE);
		gSvpwm.vd = gIDLoop.out;
		gSvpwm.vq = gIQLoop.out;
		SvpwmStep(&gSvpwm);
		//AdcSampleTest();
		//svpwm时间赋值给定时器
		TIM1->CCR1 = gSvpwm.ta;
		TIM1->CCR2 = gSvpwm.tb;
		TIM1->CCR3 = gSvpwm.tc;
		float graph[3];
		graph[0] = gSvpwm.ta;
		graph[1] = gAdcSample.aCurrent;

////		graph[0] = gObSmc.alphaEMF;
////		graph[1] = gObSmc.betaEMF;
//		//graph[1] = gSvpwm.ta ;
//		graph[0] = Q16ToF(gAdcSample.aCurrent) * CURRENT_BASE;
//		graph[1] = gSvpwm.ta;
//		graph[2] = 0;
  		GraphDataCopy(graph,3);

		
	}
}
/*******************************************************************************
 * @brief   PWM开始
 * @param   
 * @return  
 ******************************************************************************/
void PwmStart(void)
{
	TIM1->CNT = 0;
	TIM1->CR1 |= 0x01; 
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	
	TIM1->CCR4 = 4199 - 120;
	//__HAL_TIM_ENABLE_IT
	__HAL_TIM_ENABLE(&htim1);
	HAL_TIM_Base_Start(&htim1);	
}
/*******************************************************************************
 * @brief   PWM停止
 * @param   
 * @return  
 ******************************************************************************/
void PwmStop(void)
{
	TIM1->CR1 &=~0x01; 
	__HAL_TIM_DISABLE(&htim1);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
	HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);;	
}
/*******************************************************************************
 * @brief   电机控制状态机
 * @param   pHandle控制实例
 * @return  
 ******************************************************************************/
void DoControl(control_t *pHandle)
{

	switch(pHandle->machineState)
	{
#if MOTOR_CONTORL_MODE == MOTOR_CONTORL_VOLTAGE_OPEN_LOOP //电压开环控制 id控制
#define OLTAGE_OPEN_LOOP_OMEGA             (10.0f * 6.28f)   //开环时速度 
#define OLTAGE_OPEN_LOOP_ID                (0.3A)   //开环时D轴电流
		case FOC_STATE_IDLE:
			if(pHandle->sw)
			{
				ControlInit();
				pHandle->machineState = FOC_STATE_ALIGN;
				pHandle->openVoltageTheta = 0;
				PwmStart();
			}
			break;
		
		case FOC_STATE_ALIGN:	//对其角度0
		for(float i = 0.001f;i < 0.3f;i += 0.001f)
		{
			 gIDLoop.target = i;
		}
		pHandle->machineState = FOC_STATE_RUN;
			break;
		
		case FOC_STATE_RUN://对其位置后开环拖动电机运行
			gSvpwm.vd = 2.0f;
		  gSvpwm.vq = 0.0f;
		  pHandle->openVoltageTheta += OLTAGE_OPEN_LOOP_OMEGA * FOC_PWM_PERIOD;
			if(pHandle->openVoltageTheta > 6.28f)
			{
				 pHandle->openVoltageTheta -= 6.28f;
			}
			gSvpwm.sin_cos[0] = arm_sin_f32(pHandle->openVoltageTheta);
			gSvpwm.sin_cos[1] = arm_sin_f32(pHandle->openVoltageTheta);
			break;
		case FOC_STATE_STOP:
			break;
#elif MOTOR_CONTORL_MODE == MOTOR_CONTORL_CURRENT_LOOP //电流环模式
		case FOC_STATE_IDLE:
			if(pHandle->sw)
			{
				ControlInit();
				pHandle->machineState = FOC_STATE_RUN;
				pHandle->openVoltageTheta = 0;
				PwmStart();
			}
			break;
		
		case FOC_STATE_ALIGN:	//对其角度0
		for(float i = 0.001f;i < 0.3f;i += 0.001f)
		{
			 gIQLoop.target = i;
			
		}
		pHandle->machineState = FOC_STATE_RUN;
		break;	
		case FOC_STATE_RUN://直接进入闭环
			//gIQLoop.target = gIfStartup.iq;
			break;
		case FOC_STATE_STOP:
			ControlInit();
			break;
#endif
	}
}
