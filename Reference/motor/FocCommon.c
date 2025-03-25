#include "FocCommon.h"

/*******************************************************************************
 * @brief    爬坡函数
 * @param    target 目标值  inc爬坡值 
 * @return   out输出值
 ******************************************************************************/
void RampStep(ramp_t *pHandle)
{
	 if(pHandle->target < pHandle->out)
	 {
		  pHandle->out += pHandle->inc;
			if(pHandle->out >= pHandle->target)		
			{
					pHandle->out = pHandle->target;
			}
	 }
	 else if(pHandle->target > pHandle->out)
	 {
		  pHandle->out -= pHandle->inc;
			if(pHandle->out <= pHandle->target)		
			{
					pHandle->out = pHandle->target;
			}
	 }
	 else
	 {
		 pHandle->out = pHandle->target;
	 }
}