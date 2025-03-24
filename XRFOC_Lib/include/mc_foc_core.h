#ifndef __MC_FOC_CORE_H
#define __MC_FOC_CORE_H

/**
 *  @file:      XRFOC_Lib/include/mc_adaptor.h
 *  @brief:     Main FOC's call Func application layer.
 *  @author:    RunDong7582
 *  @date  :    2025 3/21 14:10
 *  @version:   XRFOC v0.1
 */

/* ------- inc ------------ */
#include "../common/mc_common.h"
#include "../include/mc_pid.h"
#include "../include/mc_lowpassfilter.h"
#include <math.h>

/* ------ define ---------- */
#define     PI    3.14159265359f
#define     PWM_Freq       30000  // 30kHz
#define     PWM_Arr_bit        8  //  8bit
/* ----- variable --------- */
volatile uint32_t timer_counter = 0;

/* ----- FOC struct ------- */

/* ----- func declare ----- */
void XFOC_set_speed_pid     ( float P, float I, float D, float ramp, float limit );
void XFOC_set_angle_pid     ( float P, float I, float D, float ramp, float limit );
void XFOC_set_current_q_pid ( float P, float I, float D, float ramp );
void XFOC_set_current_d_pid ( float P, float I, float D, float ramp ); 

float XFOC_speed_pid_tune   ( float error );
float XFOC_angle_pid_tune   ( float error );
float _normalizeAngle       ( float angle ); 

#endif 