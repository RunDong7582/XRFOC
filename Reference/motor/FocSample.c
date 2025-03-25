#include "FocSample.h"
#include "FocConfig.h"
#include "FocDataBase.h"
#include "adc.h"
#include "tim.h"
#include "arm_math.h"
#include "focDebug.h"
/*******************************************************************************
 * @brief   计算ADC三个通道偏移值
 * @param   
 * @return  偏移值
 ******************************************************************************/
void CurrentSampleInit(FocSample_t * pHandle)
{
	__HAL_ADC_DISABLE_IT(&hadc2, ADC_IT_JEOC);
	HAL_TIM_Base_Start(&htim1);
	HAL_ADCEx_InjectedStart(&hadc2);
  
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	pHandle->aCurrentOffset = 0;
	pHandle->bCurrentOffset = 0;
	pHandle->cCurrentOffset = 0;
	for(int i = 0;i < 1024;i++)
	{
		HAL_Delay(1);
		pHandle->aCurrentOffset += ADC2->JDR1;
		pHandle->bCurrentOffset += ADC2->JDR2;
		pHandle->cCurrentOffset += ADC2->JDR3;
	}
	pHandle->aCurrentOffset /= 1024;
	pHandle->bCurrentOffset /= 1024;
	pHandle->cCurrentOffset /= 1024;
	
	TIM1->CR1 &=~0x01; 
	__HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOC);
}
/*******************************************************************************
 * @brief   计算ADC三个电流值
 * @param   
 * @return  
 ******************************************************************************/
/*******************************************************************************
 * @brief   计算ADC三个电流值
 * @param   
 * @return  
 ******************************************************************************/
void CurrentSampleStepf(FocSample_f * pHandle,float sin_cos[2])
{
		static float theta = 0;
		{
			theta += 0.0001f;
			if(theta > 6.28)
			{
				theta -=6.28f;
			}
		}
		#define CURRENT_CONV_RATE    (0.00244140625f)
		int a,b,c;
//		a = ADC2->JDR1 - pHandle->aCurrentOffset;
//	  b = ADC2->JDR2 - pHandle->bCurrentOffset;
//	  c = ADC2->JDR3 - pHandle->cCurrentOffset;
		pHandle->aCurrent = a * CURRENT_CONV_RATE;
		pHandle->bCurrent = b * CURRENT_CONV_RATE;
		pHandle->cCurrent = c * CURRENT_CONV_RATE;
		
		pHandle->iAlpha = (pHandle->aCurrent - 0.5f * ( pHandle->bCurrent + pHandle->cCurrent)) * 0.666666687F;
	  pHandle->iBeta =  0.577350529f * ( pHandle->bCurrent - pHandle->cCurrent);

		pHandle->iD = sin_cos[0] * pHandle->iBeta + pHandle->iAlpha * sin_cos[1];
	  pHandle->iQ = sin_cos[1] * pHandle->iBeta - pHandle->iAlpha * sin_cos[0];
	  
}
void CurrentSampleStep(FocSample_t * pHandle,iq_t sin_cos[2])
{

		static float theta;
		float theta2,theta3;
		pHandle->aCurrent = (iq_t)(ADC2->JDR1 - pHandle->aCurrentOffset);//pHandle->aCurrentOffset);
	  pHandle->bCurrent = (iq_t)(ADC2->JDR2 - pHandle->bCurrentOffset);//pHandle->bCurrentOffset);
	  pHandle->cCurrent = (iq_t)(ADC2->JDR3 - pHandle->cCurrentOffset);//pHandle->cCurrentOffset);
		pHandle->aCurrent = pHandle->aCurrent << 3;
		pHandle->bCurrent = pHandle->bCurrent << 3;
		pHandle->cCurrent = pHandle->cCurrent << 3;
	
		pHandle->iAlpha = Q15Mpy((pHandle->aCurrent - Q15Mpy(Q15(0.5f) , ( pHandle->bCurrent + pHandle->cCurrent))) , Q15(0.666666687F));
	  pHandle->iBeta =  Q15Mpy(Q15(0.577350529f) , ( pHandle->bCurrent - pHandle->cCurrent));

		pHandle->iD =  Q15Mpy(sin_cos[0] , pHandle->iBeta) +  Q15Mpy(pHandle->iAlpha , sin_cos[1]);
	  pHandle->iQ = Q15Mpy(sin_cos[1] , pHandle->iBeta) - Q15Mpy(pHandle->iAlpha , sin_cos[0]);
}
FocSample_f fAdc;
FocSample_t qAdc;
void AdcSampleTest(void)
{
//	static  float theta;
//	float theta2;
//	iq_t a,b,c;
//	fAdc.aCurrentOffset = 2048;
//	fAdc.bCurrentOffset = 2048;
//	fAdc.cCurrentOffset = 2048;
//	qAdc.aCurrentOffset = 2048;
//	qAdc.bCurrentOffset = 2048;
//	qAdc.cCurrentOffset = 2048;
//	if(theta >= M_2PI)
//	{
//		theta -= M_2PI;
//	}
//	theta += 0.0002f;
//	#define ADC_MAX  400
//	a = arm_sin_f32(theta) * ADC_MAX;
//	theta2 = theta - (M_2PI / 3.0f);
//	if(theta2 < 0)
//	{
//		theta2 += M_2PI;
//	}
//	b = arm_sin_f32(theta2) * ADC_MAX;
//	theta2 = theta - (M_2PI * 2.0f / 3.0f);
//	if(theta2 < 0)
//	{
//		theta2 += M_2PI;
//	}
//	c = arm_sin_f32(theta2) * ADC_MAX;
//	
//	iq_t qtheta = Q16(theta / M_2PI);
//	iq_t qsin_cos[2];
//	
//	qsin_cos[0] = arm_sin_f32(theta) * Q16MAX;
//	qsin_cos[1] = arm_cos_f32(theta) * Q16MAX;
//	
//	float ftheta = Q16(theta / M_2PI);
//	float fsin_cos[2];
//	fsin_cos[0] = arm_sin_f32(theta);
//	fsin_cos[1] = arm_cos_f32(theta);
//	
//	qAdc.aCurrent = a;
//	qAdc.bCurrent = b;
//	qAdc.cCurrent = c;
//	fAdc.aCurrent = a;
//	fAdc.bCurrent = b;
//	fAdc.cCurrent = c;	
//	CurrentSampleStep(&qAdc,qsin_cos);	
//	CurrentSampleStepf(&fAdc,fsin_cos);
// 		float graph[3];
//		graph[0] = Q16ToF(qAdc.aCurrent) * CURRENT_BASE;
//		graph[1] = fAdc.aCurrent;
//		graph[2] = 0;
//		GraphDataCopy(graph,3); 
}