/**
 *  @file:      XRFOC_Lib/src/mc_pid.c
 *  @brief:     Pid algorithm implemented.
 *  @author:    RunDong7582
 *  @date  :    2026 3/19 18:00
 *  @version:   XRFOC v1.0
 */

 #include <stdint.h>
 #include "../XRFOC_Lib/APP/app.h"
 #include "../XRFOC_Lib/include/mc_pid.h"
 #include "../XRFOC_Lib/include/mc_time.h"
 
 #define _constrain(IMM, Min, Max) ((IMM)<(Min)?(Min) : ((IMM)>(Max)?(Max):(IMM)))
 
 float pid_operator ( PIDController* pid, float error )  
 {
     // fetch current timestamp
     uint32_t timestamp = mc_time_us();

     float Ts = (timestamp - pid->timestamp_prev) * 1e-6f;
     if(Ts <= 0.0f || Ts > 0.01f) Ts = 1e-4f;
     
     /* Propotion branch */
     float proportional = pid->P * error;
 
    /* Intergration branch -> Tustin plot intergraion */ 
    /* 其实就是梯形近似法积分 */
    float intergral_candidate = pid->integral_prev + 0.5f * Ts * pid->I * ( error + pid->error_prev );

    intergral_candidate = _constrain( intergral_candidate, -pid->limit, pid->limit );
     /* Derivative branch */
     /* 其实就是用y的变化量近似y的微分 (一阶泰勒公式) */
     float derivative = pid->D * (error - pid->error_prev) / Ts;
 
     /* Accumulate */
    float output_unsat = proportional + intergral_candidate + derivative;
    float output = _constrain(output_unsat, -pid->limit, pid->limit);

    /* Anti-windup: 条件积分，输出饱和且误差继续推动饱和时冻结积分 */
    float intergral = intergral_candidate;
    if ((output >= pid->limit && error > 0.0f) || (output <= -pid->limit && error < 0.0f)) {
        intergral = pid->integral_prev;
        output = _constrain(proportional + intergral + derivative, -pid->limit, pid->limit);
    }
     
     if ( pid->ramp > 0 ) {
         /* limit the rate of pid */
         float output_rate = (output - pid->output_prev) / Ts;
         if ( output_rate > pid->ramp ) 
             output = pid->output_prev + pid->ramp * Ts;
         else if ( output_rate < -(pid->ramp) )
             output = pid->output_prev - pid->ramp * Ts;
     }
 
     pid->integral_prev = intergral;
     pid->output_prev = output;
     pid->error_prev = error;
     pid->timestamp_prev = timestamp;
 
     return output;
 }

 
 
 
 