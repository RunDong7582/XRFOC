#ifndef _FocCommon_H
#define _FocCommon_H
#include "FocMath.h"
/*******************************************************************************
 * @brief    一阶低通滤波器
 * @param    oldValue 需要求的值  newValue新值  k 滤波系数
 * @return   
 ******************************************************************************/
#define LPFStep(oldValue,newValue,k)   {oldValue = (oldValue +  k * (newValue - oldValue));}

typedef struct
{
	 iq_t out;
	 iq_t target;
	 iq_t inc;
}ramp_t;
#endif