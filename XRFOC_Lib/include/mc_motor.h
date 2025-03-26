#ifndef __MC_MOTOR_H
#define __MC_MOTOR_H

/**
 *  @file:      XRFOC_Lib/include/mc_motor.h
 *  @brief:     Motor model abstract.
 *  @author:    RunDong7582
 *  @date  :    2025/3/25 17:26
 *  @version:   XRFOC v0.1
 */

#include "../common/mc_common.h"

typedef struct {
	
	int pp;				// polar pairs
	int dir;			// direction of the motor rotating
	#ifdef ESP32
	int pwmA;
	int pwmB;
	int pwmC;
		#ifdef demo_V4_support
		int enable;			
		#endif 
	#endif

} Motor_params;

#endif 