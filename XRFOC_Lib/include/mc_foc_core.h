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
#define     timer_freq      1000  // 1khz
#define     PWM_Freq       30000  // 30kHz
#define     PWM_Arr_bit        8  //  8bit

#define HIGH 1
#define LOW  0

/* ----- variable --------- */
volatile uint32_t timer_counter = 0;

/* ----- FOC struct ------- */
typedef struct {
    float I_d;
    float I_q;
    float I_alpha;
    float I_beta;
    float Ud;
    float Uq;
} xfoc_vol_cur_t;

/* ----- pid interface func ----- */
void XFOC_set_speed_pid     ( float P, float I, float D, float ramp, float limit );
void XFOC_set_angle_pid     ( float P, float I, float D, float ramp, float limit );
void XFOC_set_current_q_pid ( float P, float I, float D, float ramp );
void XFOC_set_current_d_pid ( float P, float I, float D, float ramp ); 

float XFOC_speed_pid_tune   ( float error );
float XFOC_angle_pid_tune   ( float error );

/* ----- Math solution for fast floaing point calculation ----- */
float _normalizeAngle       ( float angle ); 
float _sqrtApprox           ( float number ); 
float _atan2                ( float y, float x );

/* ----- SVPWM Func --------*/
void mc_set_torque ( float Uq, float Ud, float angle_el );
/* ----- foc state ----- */
void xfoc_disable   ( void );
void xfoc_enable    ( void ); 
void xfoc_vbus_set  ( float power_supply ); 

/* ------ Current Update ------- */
xfoc_vol_cur_t xfoc_Iq_Id_calc (float current_a, float current_b, float angle_el);

#endif 