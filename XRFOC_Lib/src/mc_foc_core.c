/**
 *  @file:      XRFOC_Lib/src/mc_foc_core.c
 *  @brief:     Main FOC's call Func application layer.
 *  @author:    RunDong7582
 *  @date  :    2025 3/21 14:06 
 *  @version:   XRFOC v0.1
 */

#include "../include/mc_foc_core.h"

#define _constrain(IMM, Min, Max) ((IMM)<(Min)?(Min) : ((IMM)>(Max)?(Max):(IMM)))

/* ------- extern declare ------- */
extern PIDController current_q_loop;
extern PIDController current_d_loop;
extern PIDController speed_loop;
extern PIDController position_loop;

extern LowPassFilter Speed_Flt;
extern LowPassFilter CurrentQ_Flt;
extern LowPassFilter CurrentD_Flt;

extern float voltage_power_supply; // 如果需要使用电压变量

/* ====== PID SET FUNC ======== */
/*        Speed pid             */
void XFOC_set_speed_pid     ( float P, float I, float D, float ramp, float limit ) 
{
    speed_loop.P = P;
    speed_loop.I = I;
    speed_loop.D = D;
    speed_loop.output_ramp = ramp;
    speed_loop.limit = limit;
}

/*        Angle pid             */
void XFOC_set_angle_pid     ( float P, float I, float D, float ramp, float limit ) 
{
    position_loop.P = P;
    position_loop.I = I;
    position_loop.D = D;
    position_loop.output_ramp = ramp;
    position_loop.limit = limit;
}

/*        Q-axis pid            */
void XFOC_set_current_q_pid ( float P, float I, float D, float ramp ) 
{
    current_q_loop.P = P;
    current_q_loop.I = I;
    current_q_loop.D = D;
    current_q_loop.output_ramp = ramp;
    current_q_loop.limit = voltage_power_supply / 2;
}

/*        D-axis pid            */
void XFOC_set_current_d_pid ( float P, float I, float D, float ramp )  
{
    current_d_loop.P = P;
    current_d_loop.I = I;
    current_d_loop.D = D;
    current_d_loop.output_ramp = ramp;
    current_d_loop.limit = 1.2;
}

/* ===== PID SET interface ==== */
/*        Speed pid             */

float XFOC_speed_pid_tune (float error)  
{
    return pid_process(&speed_loop, error); 
}

/*        Angle pid             */
float XFOC_angle_pid_tune(float error) 
{
    return pid_process(&position_loop, error);
}

/* ====== ANGLE - FUNC ======== */
float _normalizeAngle (float angle) 
{
    float a = fmod(angle, 2 * PI);  
    return a >= 0 ? a : (a + 2 * PI);
}



