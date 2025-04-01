#ifndef __MC_FOC_CORE_H
#define __MC_FOC_CORE_H

/**
 *  @file:      XRFOC_Lib/include/mc_adaptor.h
 *  @brief:     Main FOC's call Func application layer.
 *  @author:    RunDong7582
 *  @date  :    2025 3/21 14:10 -> 2025 4/1 16:26
 *  @version:   XRFOC v0.2
 */

#include "../include/mc_pid.h"
#include "../include/mc_lowpass_filter.h"
#include "../include/mc_smo_pll.h"
#include "../include/mc_motor.h"
#include <math.h>

struct Cur_vol_s {
    float I_d;
    float I_q;
    float I_alpha;
    float I_beta;
    float Ud;
    float Uq;
};

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

/* ---------------------- SVPWM Func -------------------------- */
void mc_setpwm      ( float Ua, float Ub, float Uc );
void mc_set_torque  ( float Uq, float Ud, float angle_el );

/* ----------------------- foc state -------------------------- */
int  xfoc_vf_start   ( int num, int dir );
void xrfoc_disable   ( void );
void xrfoc_enable    ( void ); 
void xrfoc_vbus_set  ( float power_supply ); 
void xrfoc_run       ( void );

/* ---------------------- Current Update ---------------------- */
struct Cur_vol_s cal_Iq_Id (float current_a, float current_b, float angle_el);
float xrfoc_current_q_lpf ( void );
float xrfoc_current_d_lpf ( void );
void  xrfoc_getphase_current ( void );

/* -------------------- SMO Control Mode ----------------------*/
void  xrfoc_smo_current_set_torque  ( float input );
void  xrfoc_smo_current_set_speed   ( float input );
void  xrfoc_smo_voltage_set_torque  ( float input );

/* ---------------------- Serial input ------------------------ */
void First_Target ( float tar );

#endif 