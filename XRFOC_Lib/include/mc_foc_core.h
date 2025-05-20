#ifndef __MC_FOC_CORE_H
#define __MC_FOC_CORE_H

/**
 *  @file:      XRFOC_Lib/include/mc_adaptor.h
 *  @brief:     Main FOC's call Func application layer.
 *  @author:    RunDong7582
 *  @date  :    2025.5.19 18:30
 *  @version:   XRFOC release version 
 */

#include "../XRFOC_Lib/include/mc_pid.h"
#include "../XRFOC_Lib/include/mc_lowpass_filter.h"
#include "../XRFOC_Lib/include/mc_smo_pll.h"
#include <math.h>

typedef enum {
    VOLTAGE_Torque = 0,
    CURRENT_Torque,
    SPEED_Torque,
} Control_Obj;

typedef enum {
    _V_F_STAGE = 0,     // vf开环启动阶段
    _ALIGN_STAGE,       // 电角度对齐阶段
    _CLOSE_LOOP_STAGE   // 观测器闭环阶段
} FOC_STAGE;

struct Cur_vol_s {
    float I_d;
    float I_q;
    float I_alpha;
    float I_beta;
    float Ud;
    float Uq;
};

typedef struct {
    float eAngle;
    float openloopVel;
    float angle_thresh; 
    float Uq_boost;
    float angle_error;
    uint16_t cntangle;
    FOC_STAGE schedule;
} XRFOC_Console;

typedef struct {
    uint8_t num;
    uint8_t loop;
    uint8_t power;
    uint8_t pp;
    uint8_t dir;
    uint8_t svpwm_flag;
    uint8_t run_flag;
    uint8_t lock_flag;
} Motor_Console;

typedef struct {
    float current_u;
	float current_v;
	float current_w;
    float Target_flt;
    float Target_set;
    Control_Obj TYPE;
    uint8_t cntt;
} Motor_Console_Data;

void xrfoc_module_struct_clear  ( void );
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
// float _atan2                ( float y, float x );
// float _sin                  ( float x );
// float _cos                  ( float x );

/* ---------------------- SVPWM Func -------------------------- */
void mc_setpwm      ( float Ua, float Ub, float Uc );
void mc_set_torque  ( float Uq, float Ud, float angle_el );

/* ----------------------- foc state -------------------------- */
int  xrfoc_vf_start   ( uint8_t num, uint8_t dir );
void xrfoc_vbus_set  ( float power_supply ); 
void xrfoc_run       ( void );
void xrfoc_pretarget (int _PP, int _DIR);
/* ---------------------- Current Update ---------------------- */
struct Cur_vol_s cal_Iq_Id (float current_a, float current_b, float angle_el);
float xrfoc_current_q_lpf ( void );
float xrfoc_current_d_lpf ( void );
void  xrfoc_getphase_current ( void );

/* -------------------- SMO Control Mode ----------------------*/
void  xrfoc_smo_current_set_torque  ( float Target , Control_Obj type );
void  xrfoc_smo_current_set_speed   ( float Target , Control_Obj type );
void  xrfoc_smo_voltage_set_torque  ( float Target , Control_Obj type );

/* ---------------------- Serial input ------------------------ */

#endif 
