/**
 *  @file:      XRFOC_Lib/src/mc_pid.c
 *  @brief:     Pid algorithm implemented.
 *  @author:    RunDong7582
 *  @date  :    2025 3/21 11:17
 *  @version:   XRFOC v0.1
 */

#include "../include/mc_pid.h"

#define _constrain(IMM, Min, Max) ((IMM)<(Min)?(Min) : ((IMM)>(Max)?(Max):(IMM)))

void pid_init ( PIDController* pid, float P, float I, float D, float ramp, float limit ) {

    pid->P = P;
    pid->I = I;
    pid->D = D;
    pid->output_ramp = ramp;
    pid->limit = limit;
    pid->error_prev = 0;
    pid->output_prev = 0;
    pid->integral_prev = 0;
    pid->timestamp_prev = timer_counter;

}

float pid_process ( PIDController* pid, float error )  
{
    // fetch current timestamp
    uint32_t timestamp = timer_counter;
    
    // calculate delta time
    uint32_t dt = timestamp - pid->timestamp_prev;
 
    // convert to second . 1 tick = 1ms. Ts: .s
    float Ts = (float) dt * 1e-3f;

    if ( Ts > 0.5f ) Ts = 1e-3f;
    
    /* Propotion branch */
    float proportional = pid->P * error;

    /* Intergration branch -> Tustin plot intergraion */ 
    /* 其实就是梯形近似法积分 */
    float intergral = pid->integral_prev + 0.5f * Ts * pid->I * ( error + pid->error_prev );
    
    intergral = _constrain( intergral, -pid->limit, pid->limit );
    /* Derivative branch */
    /* 其实就是用y的变化量近似y的微分 (一阶泰勒公式) */
    float derivative = pid->D * (error - pid->error_prev) / Ts;

    /* Accumulate */
    float Output = proportional + intergral + derivative;
    Output = _constrain(Output, -pid->limit, pid->limit);
    
    if ( pid->output_ramp > 0 ) {
        /* limit the rate of pid */
        float Output_rate = (Output - pid->output_prev) / Ts;
        if ( Output_rate > pid->output_ramp ) 
            Output = pid->output_prev + pid->output_ramp * Ts;
        else if ( Output_rate < -(pid->output_ramp) )
            Output = pid->output_prev - pid->output_ramp * Ts;
    }

    pid->integral_prev = intergral;
    pid->output_prev = Output;
    pid->error_prev = error;
    pid->timestamp_prev = timestamp;

    return Output;
}