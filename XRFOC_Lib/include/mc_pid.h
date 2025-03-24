#ifndef __MC_PID_H
#define __MC_PID_H

/**
 *  @file       XRFOC_Lib/include/mc_pid.h
 *  @brief      Pid algorithm implemented.
 *  @author:    RunDong7582
 *  @date       2025 3/20 15:59
 *  @version    XRFOC v0.1
 */

/*--------inc--------------*/ 
#include "../common/mc_common.h"

/*--------variable---------*/
volatile uint32_t timer_counter = 0;

/*--------pid struct------*/
typedef struct {

    float P;
    float I;
    float D;
    float output_ramp;
    float limit;
    float error_prev;
    float output_prev;
    float integral_prev;
    uint32_t timestamp_prev;

} PIDController;

/*--------func declare-----*/
void  pid_init    ( PIDController* pid, float P, float I, float D, float ramp, float limit);
float pid_process ( PIDController* pid, float error ); 

#endif