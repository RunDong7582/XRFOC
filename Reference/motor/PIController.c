#include "PIController.h"
#include "FocConfig.h"
/**
 * @brief   pi计算
 * @param   target目标值  real实际值
 * @return  out输出值
 */
void PIControllerStep(PIController_t *pi)
{
		iq_t integ,out;
		pi->err = pi->target - pi->real;
	  integ = pi->integral;
		integ += Q15Mpy(pi->ki , pi->err); 
		out = integ + Q15Mpy(pi->kp , pi->err);
		//防止积分饱和，限幅处理
		if(out > pi->maxValue)
		{
			out = pi->maxValue;
		}
		else if(out < pi->minValue )	
		{
			out = pi->minValue;
		}
		else
		{
			
		}
		pi->integral = integ;
		pi->out = out;
 }