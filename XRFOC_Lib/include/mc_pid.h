#ifndef __MC_PID_H
#define __MC_PID_H

/**
 *  @file       XRFOC_Lib/include/mc_pid.h
 *  @brief      Pid algorithm implemented.
 *  @author:    RunDong7582
 *  @date       2025.5.19 18:32
 *  @version    XRFOC release version 
 */

/*--------inc--------------*/ 
#include "../common/mc_common.h"

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
    // unsigned long timestamp_prev;
    uint32_t timestamp_prev;
} PIDController;

float pid_operator ( PIDController* pid, float error );

#endif