#ifndef __MC_PID_H
#define __MC_PID_H

/**
 *  @file       XRFOC_Lib/include/mc_pid.h
 *  @brief      Pid algorithm implemented.
 *  @author:    RunDong7582
 *  @date       2025 3/20 15:59 -> 2025 4/1 16:26
 *  @version    XRFOC v0.2
 */

/*--------inc--------------*/ 
#include "../common/mc_common.h"
#include <stdint.h>

/*--------variable---------*/
// volatile uint32_t timer_counter = 0;

/*--------pid struct------*/
typedef struct {

    float P;
    float I;
    float D;
    float ramp;
    float limit;
    float error_prev;
    float output_prev;
    float integral_prev;
    unsigned long timestamp_prev;

} PIDController;

float pid_operator ( PIDController* pid, float error );

#endif