#ifndef __MC_MOTOR_H
#define __MC_MOTOR_H

/**
 *  @file:      XRFOC_Lib/include/mc_motor.h
 *  @brief:     Motor model abstract.
 *  @author:    RunDong7582
 *  @date  :    2025 3/25 17:26 -> 2025 4/1 16:26
 *  @version:   XRFOC v0.2
 */

#include "../common/mc_common.h"
#include <stdint.h>

typedef struct {
	uint8_t pp;				// polar pairs
	uint8_t dir;			// direction of the motor rotating
	
} Motor_params;

void xr_motor_power_init ( Motor_params *mt );

#endif 
