#ifndef __MC_FOC_CORE_H
#define __MC_FOC_CORE_H

/**
 *  @file:      XRFOC_Lib/include/mc_adaptor.h
 *  @brief:     Main FOC's call Func application layer.
 *  @author:    RunDong7582
 *  @date  :    2025 3/21 14:10
 *  @version:   XRFOC v0.1
 */

#include "../include/mc_pid.h"
#include "../include/mc_lowpass_filter.h"
#include "../include/mc_smo_pll.h"
#include <math.h>
#include <stdint.h>

#define     TIMER_FREQ       1000  //  1kHz
#define     PWM_FREQ        30000  // 30kHz
#define     PWM_ARR_BIT        8   //  8bit
#define     PWM_Period       255   //  0 ~ 256

#define     HIGH    1
#define     LOW     0

volatile uint32_t timer_counter = 0;

typedef struct {

    float I_d;
    float I_q;
    float I_alpha;
    float I_beta;
    float Ud;
    float Uq;

} xrfoc_cur_vol_s;

typedef struct {

    float I_a;
    float I_b;
    float I_c;

} xrfoc_origin_t;

/* -------------------- pid interface func -------------------- */
void xrfoc_set_speed_pid     ( float P, float I, float D, float ramp, float limit );
void xrfoc_set_angle_pid     ( float P, float I, float D, float ramp, float limit );
void xrfoc_set_current_q_pid ( float P, float I, float D, float ramp );
void xrfoc_set_current_d_pid ( float P, float I, float D, float ramp ); 

float xrfoc_speed_pid_tune   ( float error );
float xrfoc_angle_pid_tune   ( float error );

/* ----- Math solution for fast floaing point calculation ----- */
float _normalizeAngle       ( float angle ); 
float _sqrtApprox           ( float number ); 
float _atan2                ( float y, float x );

/* ---------------------- SVPWM Func -------------------------- */
void mc_setpwm      ( float Ua, float Ub, float Uc );
void mc_set_torque  ( float Uq, float Ud, float angle_el );

/* ----------------------- foc state -------------------------- */
int  xfoc_vf_start   ( int dir );
void xrfoc_disable   ( void );
void xrfoc_enable    ( void ); 
void xrfoc_vbus_set  ( float power_supply ); 
void xrfoc_run       ( void );

/* ---------------------- Current Update ---------------------- */
xrfoc_cur_vol_s xrfoc_Iq_Id_calc (float current_a, float current_b, float angle_el);
float xrfoc_current_q ( void );
float xrfoc_current_d ( void );
void  xrfoc_getphase_current ( void );
/* -------------------- SMO Control Mode ----------------------*/
void  xrfoc_smo_current_set_torque  ( float input );
void  xrfoc_smo_current_set_speed   ( float input );
void  xrfoc_smo_voltage_set_torque  ( float input );

/* ---------------------- Serial input ------------------------ */
void First_Target ( float tar );

#endif 