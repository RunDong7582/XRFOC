#include "FocComponentsSvpwm.h"
#include "arm_math.h" 
#include "focdebug.h" 

ComponentsSvpwm_t ComponentsSvpwm;
float theta = 20.0f;
float debugBuff[1024];
int   buffIdx = 0;
void DebugBuffRead(float a)
{
	debugBuff[buffIdx] = a;
	buffIdx = (buffIdx + 1) % (sizeof(debugBuff) / sizeof(float));
}
/**
 * @brief   测试wvpwm是否正常
 * @param   
 * @return  
 */
void SvpwmTest(void)
{
	
		theta += 3.0f;
		if(theta >= 360.0f)
		{
			theta -= 360.0f;
		}
		float t = theta / 360.0f * 6.28f;
		ComponentsSvpwm.sin_cos[0] = arm_sin_f32(t);
		ComponentsSvpwm.sin_cos[1] = arm_cos_f32(t);
		ComponentsSvpwm.vBus = 12.0f;
		ComponentsSvpwm.vq = 6.0f;
		ComponentsSvpwm.vd = 0.0f * 0.5f;
		
		SvpwmStep(&ComponentsSvpwm);
		//GraphDataCopy(&ComponentsSvpwm.ta,1);
}
/*******************************************************************************
 * @brief   计算三个pwm比较器值
 * @param   svpwm结构体
 * @return  三个pwm周期数
 ******************************************************************************/
void SvpwmStep(ComponentsSvpwm_t *pHandle)
{

	 //转换为alpha beta轴
	  pHandle->vAlpha = Q15Mpy(pHandle->sin_cos[1] , pHandle->vd) - Q15Mpy(pHandle->sin_cos[0] , pHandle->vq);
	  pHandle->vBeta  = Q15Mpy(pHandle->sin_cos[0] , pHandle->vd) + Q15Mpy(pHandle->sin_cos[1] , pHandle->vq);
#if SVPWM_COMPUTATIONAL_METHOD  == SVPWM_COMPUTATIONAL_SECTOR
		int N = 0;
		float a,b,c;
	 //这里做了等幅值变换
	 //计算扇区
		a = pHandle->vBeta;
		c = -SQRT3_2 * pHandle->vAlpha - 0.5f * pHandle->vBeta;
		b = SQRT3_2 * pHandle->vAlpha - 0.5f * pHandle->vBeta;
		if(a > 0)
		{
			N += 1;
		}
		if(b > 0)
		{
			N += 2;
		}
		if(c > 0)
		{
			N += 4;
		}
		
		//计算各个扇区时间
	//	pHandle->tAlpha =  pHandle->vAlpha / pHandle->vBus;
	//	pHandle->tBeta   = (1.0f / SQRT3) * pHandle->vBeta / pHandle->vBus;		
		pHandle->tAlpha = 1.5f * pHandle->vAlpha / pHandle->vBus;
		pHandle->tBeta   = SQRT3_2 * pHandle->vBeta / pHandle->vBus;
		switch(N)
		{
			case 0:
			pHandle->ta = 0.5f;
			pHandle->tb = 0.5f;
			pHandle->tc = 0.5f;
			break;
			//第一扇区N = 3
			case 3:
			pHandle->tx = pHandle->tAlpha - pHandle->tBeta;
			pHandle->ty = pHandle->tBeta * 2.0f;
			pHandle->ta = (1.0f - pHandle->tx - pHandle->ty) * 0.25f;
			pHandle->tb = (1.0f + pHandle->tx - pHandle->ty) * 0.25f;
			pHandle->tc = (1.0f + pHandle->tx + pHandle->ty) * 0.25f;
			pHandle->sector = 1;
			break;
			//第二扇区 N = 1
			case 1:
			pHandle->tx = -pHandle->tAlpha + pHandle->tBeta;
			pHandle->ty = pHandle->tAlpha + pHandle->tBeta;
			pHandle->ta = (1.0f + pHandle->tx - pHandle->ty) * 0.25f;
			pHandle->tb = (1.0f - pHandle->tx - pHandle->ty) * 0.25f;
			pHandle->tc = (1.0f + pHandle->tx + pHandle->ty) * 0.25f;
			pHandle->sector = 2;
			break;
			//第三扇区 N = 5
			case 5:
			pHandle->tx = 2.0f * pHandle->tBeta;
			pHandle->ty = -pHandle->tAlpha - pHandle->tBeta;
			pHandle->ta = (1.0f + pHandle->tx + pHandle->ty) * 0.25f;
			pHandle->tb = (1.0f - pHandle->tx - pHandle->ty) * 0.25f;
			pHandle->tc = (1.0f + pHandle->tx - pHandle->ty) * 0.25f;
			pHandle->sector = 3;
			break;			
			//第四扇区 N = 4
			case 4:
			pHandle->tx = -2.0f * pHandle->tBeta;
			pHandle->ty = -pHandle->tAlpha + pHandle->tBeta;
			pHandle->ta = (1.0f + pHandle->tx + pHandle->ty) * 0.25f;
			pHandle->tb = (1.0f + pHandle->tx - pHandle->ty) * 0.25f;
			pHandle->tc = (1.0f - pHandle->tx - pHandle->ty) * 0.25f;
			pHandle->sector = 4;
			break;	
			//第五扇区 N = 6
			case 6:
			pHandle->tx = -pHandle->tAlpha - pHandle->tBeta;
			pHandle->ty = pHandle->tAlpha - pHandle->tBeta;
			pHandle->ta = (1.0f + pHandle->tx - pHandle->ty) * 0.25f;
			pHandle->tb = (1.0f + pHandle->tx + pHandle->ty) * 0.25f;
			pHandle->tc = (1.0f - pHandle->tx - pHandle->ty) * 0.25f;
			pHandle->sector = 4;
			break;		
			//第六扇区 N = 2
			case 2:
			pHandle->tx = pHandle->tAlpha + pHandle->tBeta;
			pHandle->ty = -2.0f * pHandle->tBeta;
			pHandle->ta = (1.0f - pHandle->tx - pHandle->ty) * 0.25f;
			pHandle->tb = (1.0f + pHandle->tx + pHandle->ty) * 0.25f;
			pHandle->tc = (1.0f + pHandle->tx - pHandle->ty) * 0.25f;
			pHandle->sector = 6;
			break;						
		}

#else
 //三次谐波叠加
  iq_t Min;
  iq_t sum1;
  iq_t sum2;
  iq_t y;
  Min = Q15Mpy(Q15(-0.5F) , pHandle->vAlpha);
	sum1 = Q15Mpy(Q15(0.866025388F) , pHandle->vBeta);
  sum2 = Min + sum1;
  sum1 = Min - sum1;
  if (pHandle->vAlpha < sum2) {
    Min = pHandle->vAlpha;
  } else {
    Min = sum2;
  }

  if (pHandle->vAlpha > sum2) {
    y = pHandle->vAlpha;
  } else {
    y = sum2;
  }

  if (Min >= sum1) {
    Min = sum1;
  }

  if (y <= sum1) {
    y = sum1;
  }
	
  Min = Q15Mpy((Min + y) , Q15(-0.5F));
  pHandle->ta = Min + pHandle->vAlpha;
  pHandle->tb = Min + sum2;
  pHandle->tc = Min + sum1;

  pHandle->ta = -Q15Div(pHandle->ta ,pHandle->vBus) + Q15(0.5f) ;
  pHandle->tb = -Q15Div(pHandle->tb ,pHandle->vBus) + Q15(0.5f) ;
  pHandle->tc = -Q15Div(pHandle->tc ,pHandle->vBus) + Q15(0.5f) ;
	pHandle->ta = pHandle->ta * PWM_PERIOD >> 15;
	pHandle->tb = pHandle->tb * PWM_PERIOD >> 15;
	pHandle->tc = pHandle->tc * PWM_PERIOD >> 15;
#endif
	DebugBuffRead(pHandle->ta);
//		pHandle->ta = PWM_PERIOD * pHandle->ta;
//		pHandle->tb = PWM_PERIOD * pHandle->tb;
//		pHandle->tc = PWM_PERIOD * pHandle->tc;	
}