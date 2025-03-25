#include "FocObserver.h"
#include "FocDataBase.h"
#include "arm_math.h" 
#include "math.h" 
#include <string.h>

short CORDIC_atan2(short x, short y)
{
    unsigned short a, i;
    short hAngle;
    long wXi, wYi, wXold;
    a = (unsigned short)((x > 0) ? x : -x);
    a = (unsigned short)((y > 0) ? a + y : a - y);
    for (i = 0; i < 16; i++)
    {
        if ((a & (0x8000 >> i)) != 0)
        {
            if (i == 0)
            {
                x = (short)(x >> 1);
                y = (short)(y >> 1);
            }
            else
            {
                x = (short)(x << (i - 1));
                y = (short)(y << (i - 1));
            }
            break;
        }
    }

    /*Determining quadrant*/
    if (x < 0)
    {
        if (y < 0)
        {
            /*Quadrant III, add 90 degrees so as to move to quadrant IV*/
            hAngle = 16384;
            wXi = -(y >> 1);
            wYi = x >> 1;
        }
        else
        {
            /*Quadrant II, subtract 90 degrees so as to move to quadrant I*/
            hAngle = -16384;
            wXi = y >> 1;
            wYi = -(x >> 1);
        }
    }
    else
    {
        /* Quadrant I or IV*/
        hAngle = 0;
        wXi = x >> 1;
        wYi = y >> 1;
    }
    wXold = wXi;

    /*begin the successive approximation process*/
    /*iteration0*/
    if (wYi < 0)
    {
        /*vector is in Quadrant IV*/
        hAngle += 8192;
        wXi = wXi - wYi;
        wYi = wXold + wYi;
    }
    else
    {
        /*vector is in Quadrant I*/
        hAngle -= 8192;
        wXi = wXi + wYi;
        wYi = -wXold + wYi;
    }
    wXold = wXi;

    /*iteration1*/
    if (wYi < 0)
    {
        /*vector is in Quadrant IV*/
        hAngle += 4836;
        wXi = wXi - (wYi >> 1);
        wYi = (wXold >> 1) + wYi;
    }
    else
    {
        /*vector is in Quadrant I*/
        hAngle -= 4836;
        wXi = wXi + (wYi >> 1);
        wYi = -(wXold >> 1) + wYi;
    }
    wXold = wXi;

    /*iteration2*/
    if (wYi < 0)
    {
        /*vector is in Quadrant IV*/
        hAngle += 2555;
        wXi = wXi - (wYi >> 2);
        wYi = (wXold >> 2) + wYi;
    }
    else
    {
        /*vector is in Quadrant I*/
        hAngle -= 2555;
        wXi = wXi + (wYi >> 2);
        wYi = -(wXold >> 2) + wYi;
    }
    wXold = wXi;

    /*iteration3*/
    if (wYi < 0)
    {
        /*vector is in Quadrant IV*/
        hAngle += 1297;
        wXi = wXi - (wYi >> 3);
        wYi = (wXold >> 3) + wYi;
    }
    else
    {
        /*vector is in Quadrant I*/
        hAngle -= 1297;
        wXi = wXi + (wYi >> 3);
        wYi = -(wXold >> 3) + wYi;
    }
    wXold = wXi;

    /*iteration4*/
    if (wYi < 0)
    {
        /*vector is in Quadrant IV*/
        hAngle += 651;
        wXi = wXi - (wYi >> 4);
        wYi = (wXold >> 4) + wYi;
    }
    else
    {
        /*vector is in Quadrant I*/
        hAngle -= 651;
        wXi = wXi + (wYi >> 4);
        wYi = -(wXold >> 4) + wYi;
    }
    wXold = wXi;

    /*iteration5*/
    if (wYi < 0)
    {
        /*vector is in Quadrant IV*/
        hAngle += 326;
        wXi = wXi - (wYi >> 5);
        wYi = (wXold >> 5) + wYi;
    }
    else
    {
        /*vector is in Quadrant I*/
        hAngle -= 326;
        wXi = wXi + (wYi >> 5);
        wYi = -(wXold >> 5) + wYi;
    }
    wXold = wXi;

    /*iteration6*/
    if (wYi < 0)
    {
        /*vector is in Quadrant IV*/
        hAngle += 163;
        wXi = wXi - (wYi >> 6);
        wYi = (wXold >> 6) + wYi;
    }
    else
    {
        /*vector is in Quadrant I*/
        hAngle -= 163;
        wXi = wXi + (wYi >> 6);
        wYi = -(wXold >> 6) + wYi;
    }
    wXold = wXi;

    /*iteration7*/
    if (wYi < 0)
    {
        /*vector is in Quadrant IV*/
        hAngle += 81;
        wXi = wXi - (wYi >> 7);
        wYi = (wXold >> 7) + wYi;
    }
    else
    {
        /*vector is in Quadrant I*/
        hAngle -= 81;
        wXi = wXi + (wYi >> 7);
        wYi = -(wXold >> 7) + wYi;
    }

    /*iteration8*/
    if (wYi < 0)
    {
        /*vector is in Quadrant IV*/
        hAngle += 41;
        wXi = wXi - (wYi >> 8);
        wYi = (wXold >> 8) + wYi;
    }
    else
    {
        /*vector is in Quadrant I*/
        hAngle -= 41;
        wXi = wXi + (wYi >> 8);
        wYi = -(wXold >> 8) + wYi;
    }

    /*iteration9*/
    if (wYi < 0)
    {
        /*vector is in Quadrant IV*/
        hAngle += 20;
        wXi = wXi - (wYi >> 9);
        wYi = (wXold >> 9) + wYi;
    }
    else
    {
        /*vector is in Quadrant I*/
        hAngle -= 20;
        wXi = wXi + (wYi >> 9);
        wYi = -(wXold >> 9) + wYi;
    }
    return -hAngle;
}

/*******************************************************************************
 * @brief    滑膜传感器初始化
 * @param    
 * @return   
 ******************************************************************************/
void ObserverSmcInit(ObserverSmc_t* pHandle)
{
	memset(pHandle,0,sizeof(ObserverSmc_t));
	pHandle->A = Q15(FOC_PWM_PERIOD * R_BASE / MOTOR_PHASE_INDUCTANCE);
	pHandle->B =  Q15(1.0f - MOTOR_PHASE_RESISTANCE /  MOTOR_PHASE_INDUCTANCE * FOC_PWM_PERIOD);
	pHandle->pllKi = OB_SMC_PLL_KI;
	pHandle->pllKp = OB_SMC_PLL_KP;
	pHandle->h = Q15(OB_SMC_GAIN);
	pHandle->lpfK = Q15(OB_SMC_LPF_K);
	pHandle->pi.ki = Q15(OB_SMC_PLL_KI / OMEGA_BASE * U_BASE * FOC_PWM_PERIOD); ;
	pHandle->pi.kp = Q15(OB_SMC_PLL_KP / OMEGA_BASE * U_BASE);
	pHandle->pi.maxValue = Q15(200);
	pHandle->pi.minValue = -Q15(200);
}
/*******************************************************************************
 * @brief    限幅函数
 * @param    
 * @return   
 ******************************************************************************/
#define Sat_MAX   (Q15MAX / 2)
#define ObSat(value) {if(value > Sat_MAX){value = Sat_MAX;}else if(value < -Sat_MAX){value = -Sat_MAX;}}
/*******************************************************************************
 * @brief    一阶低通滤波
 * @param    
 * @return   
 ******************************************************************************/
#define ObLpf(oldValue,newValue,a) (oldValue + Q15Mpy(a , (newValue - oldValue)))
/*******************************************************************************
 * @brief    通过滑膜+PLL锁相环方式估算电角度
 * @param    pHandle实例  vAlpha vBeta 电压值   realAlphaCurrent realBetaCurrent电流值
 * @return   oemga当前电角速度  theta当前电角度 sin_cos sincos值
 ******************************************************************************/
void ObserverSmcStep(ObserverSmc_t* pHandle,iq_t vAlpha,iq_t vBeta,iq_t realAlphaCurrent,iq_t realBetaCurrent)
{
		//计算观测电流
		pHandle->alphaCurreantHat = (Q15Mpy((vAlpha - pHandle->alphaZ) , pHandle->A) + Q15Mpy(pHandle->alphaCurreantHat ,pHandle->B));
	  pHandle->betaCurreantHat  = (Q15Mpy((vBeta  - pHandle->betaZ) ,  pHandle->A) + Q15Mpy(pHandle->betaCurreantHat  ,pHandle->B));
		//计算差值B
		//B = Ts * Rs / Ls
		//A = Ts / Ls
		pHandle->alphaErr = -realAlphaCurrent + pHandle->alphaCurreantHat;
	  pHandle->betaErr = -realBetaCurrent + pHandle->betaCurreantHat;
	 //对差值进行增益
		ObSat(pHandle->alphaErr);
		pHandle->alphaZ = Q15Mpy(pHandle->h , pHandle->alphaErr);
		ObSat(pHandle->betaErr);
		pHandle->betaZ  = Q15Mpy(pHandle->h , pHandle->betaErr);
	 //控制量就是反电动势就是alphaZ对alphaZ进行滤波后得到反电动势
		pHandle->alphaEMF = ObLpf(pHandle->alphaEMF,pHandle->alphaZ,pHandle->lpfK);
	  pHandle->betaEMF = ObLpf(pHandle->betaEMF,pHandle->betaZ,pHandle->lpfK);
//		//用PLL方法求解角度
//	  //先求解 角度差
//		//这里算的是标幺值，乘上电压值基准才是角度差的标幺
//		//KP = L * 带宽 
//	  //KI = R * 带宽 * pwm周期
//		//穿进去的差值是 角度差 * 2PI / u_base
//		pHandle->pi.target = -Q15Mpy(pHandle->sin_cos[0] , pHandle->betaEMF) - Q15Mpy(pHandle->sin_cos[1] , pHandle->alphaEMF);
//		pHandle->pi.real   = 0;
//	  //计算一下PI
//	  PIControllerStep(&pHandle->pi);
//		//PI输出为角速度值
//		pHandle->oemga = pHandle->pi.out;
//		//对速度进行积分得到角度
//		pHandle->theta += pHandle->oemga / SPEED_DIV_FOC_RATE;
//		//pHandle->theta = atan2f(-pHandle->alphaEMF,pHandle->betaEMF);
//		if(pHandle->theta >= Q15MAX)
//		{
//			pHandle->theta -= Q16MAX;
//		}
//		if(pHandle->theta < -Q15MAX)
//		{
//			pHandle->theta += Q16MAX;
//		}
//		 Q16SinCos(pHandle->theta,&pHandle->sin_cos[0]);
			short x = (short)(pHandle->alphaEMF);
			short y = (short)(pHandle->betaEMF);
			pHandle->theta = -(iq_t)(CORDIC_atan2(y,x) + 0);
			if(pHandle->theta > 32768)
			{
				pHandle->theta -= 65536;
			}
}

///*******************************************************************************
// * @brief    滑膜传感器初始化
// * @param    
// * @return   
// ******************************************************************************/
//void ObserverSmcInit(ObserverSmc_t* pHandle)
//{
//	memset(pHandle,0,sizeof(ObserverSmc_t));
//	pHandle->A = 1.0f /  MOTOR_PHASE_INDUCTANCE * FOC_PWM_PERIOD;
//	pHandle->B =  MOTOR_PHASE_RESISTANCE /  MOTOR_PHASE_INDUCTANCE * FOC_PWM_PERIOD;
//	pHandle->pllKi = OB_SMC_PLL_KI;
//	pHandle->pllKp = OB_SMC_PLL_KP;
//	pHandle->h = OB_SMC_GAIN;
//	pHandle->lpfK = OB_SMC_LPF_K;
//	pHandle->pi.ki = OB_SMC_PLL_KI;
//	pHandle->pi.kp = OB_SMC_PLL_KP;
//	pHandle->pi.maxValue = 10E6;
//	pHandle->pi.minValue = -10E6;
//}
///*******************************************************************************
// * @brief    限幅函数
// * @param    
// * @return   
// ******************************************************************************/
//#define ObSat(value) {if(value > 1){value = 1;}else if(value < -1){value = -1;}}
///*******************************************************************************
// * @brief    一阶低通滤波
// * @param    
// * @return   
// ******************************************************************************/
//#define ObLpf(oldValue,newValue,a) (oldValue + a * (newValue - oldValue))
///*******************************************************************************
// * @brief    通过滑膜+PLL锁相环方式估算电角度
// * @param    pHandle实例  vAlpha vBeta 电压值   realAlphaCurrent realBetaCurrent电流值
// * @return   oemga当前电角速度  theta当前电角度 sin_cos sincos值
// ******************************************************************************/
//void ObserverSmcStep(ObserverSmc_t* pHandle,float vAlpha,float vBeta,float realAlphaCurrent,float realBetaCurrent)
//{
//	  vAlpha = vAlpha / (32767.0f * U_BASE);
//	  vBeta = vAlpha / (32767.0f * U_BASE);
//	  realAlphaCurrent = realAlphaCurrent / (32767.0f * CURRENT_BASE);
//	  realBetaCurrent = realBetaCurrent / (32767.0f * CURRENT_BASE);
//		//计算观测电流
//		pHandle->alphaCurreantHat += ((vAlpha - pHandle->alphaZ) * pHandle->A - pHandle->alphaCurreantHat * pHandle->B);
//	  pHandle->betaCurreantHat += ((vBeta  - pHandle->betaZ)  * pHandle->A - pHandle->betaCurreantHat * pHandle->B);
//		//计算差值
//		pHandle->alphaErr = -realAlphaCurrent + pHandle->alphaCurreantHat;
//	  pHandle->betaErr = -realBetaCurrent + pHandle->betaCurreantHat;
//	 //对差值进行增益
//		ObSat(pHandle->alphaErr);
//		pHandle->alphaZ = pHandle->h * pHandle->alphaErr;
//		ObSat(pHandle->betaErr);
//		pHandle->betaZ  = pHandle->h * pHandle->betaErr;
//	 //控制量就是反电动势就是alphaZ对alphaZ进行滤波后得到反电动势
//		pHandle->alphaEMF = ObLpf(pHandle->alphaEMF,pHandle->alphaZ,pHandle->lpfK);
//	  pHandle->betaEMF = ObLpf(pHandle->betaEMF,pHandle->betaZ,pHandle->lpfK);
//		//用PLL方法求解角度
//	  //先求解 角度差
//		pHandle->pi.target = -pHandle->sin_cos[0] * pHandle->betaEMF - pHandle->sin_cos[1] * pHandle->alphaEMF;
//		pHandle->pi.real   = 0;
//	  //计算一下PI
//	  PIControllerStep(&pHandle->pi);
//		//PI输出为角速度值
//		pHandle->oemga = pHandle->pi.out;
//		//对速度进行积分得到角度
//		pHandle->theta += pHandle->oemga * FOC_PWM_PERIOD;
//		//pHandle->theta = atan2f(-pHandle->alphaEMF,pHandle->betaEMF);
//		if(pHandle->theta >= 6.2831853072f)
//		{
//			pHandle->theta -= 6.2831853072f;
//		}
//		if(pHandle->theta < 0)
//		{
//			pHandle->theta += 6.2831853072f;
//		}
//		pHandle->sin_cos[0] = arm_sin_f32(pHandle->theta);
//		pHandle->sin_cos[1] = arm_cos_f32(pHandle->theta);
//		
//		pHandle->theta  = pHandle->theta / 6.28f * 32768.0f;
//}