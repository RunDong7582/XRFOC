/**
 *  @file:      XRFOC_Lib/src/mc_pid.c
 *  @brief:     Pid algorithm implemented.
 *  @author:    RunDong7582
 *  @date  :    2025.5.19 18:47
 *  @version:   XRFOC release version
 */
 #include <stdint.h>
 #include "../XRFOC_Lib/APP/app.h"
 #include "../XRFOC_Lib/include/mc_pid.h"

 #define _constrain(IMM, Min, Max) ((IMM)<(Min)?(Min) : ((IMM)>(Max)?(Max):(IMM)))
 
 float pid_operator ( PIDController* pid, float error )  
 {
     uint32_t timestamp = HAL_GetTick();

     float Ts = (timestamp - pid->timestamp_prev) * 1e-3f;
     if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
     
     /* Propotion branch */
     float proportional = pid->P * error;
 
     /* Intergration branch -> Tustin plot intergraion */ 
     float intergral = pid->integral_prev + 0.5f * Ts * pid->I * ( error + pid->error_prev );
     
     intergral = _constrain( intergral, -pid->limit, pid->limit );
     
     /* Derivative branch */
     float derivative = pid->D * (error - pid->error_prev) / Ts;
 
     /* Accumulate */
     float output = proportional + intergral + derivative;
     output = _constrain(output, -pid->limit, pid->limit);
     
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

 
 