/**
 *  @file:      XRFOC_Lib/src/mc_lowpassfilter.c
 *  @brief:     LowPassFilter first order implemented.
 *  @author:    RunDong7582
 *  @date  :    2025 3/21 11:06
 *  @version:   XRFOC v0.1
 */

#include "../include/mc_lowpassfilter.h"

void filter_Init (LowPassFilter* filter, float time_constant)
{
    filter->Tc = time_constant;
    filter->y_prev = 0;
    filter->timestamp_prev = timer_counter;
}

float filter_process (LowPassFilter* filter, float x)
{
    // fetch current timestamp
    uint32_t timestamp = timer_counter;

    // calculate delta time
    uint32_t dt_cnt = timestamp - filter->timestamp_prev;
    
    // convert to second . 1 tick = 1ms. dt: .s
    float dt = (float) dt_cnt * 1e-3f; 

    if ( dt > 0.3f ) {
        // reset filter
        filter->y_prev = x;
        filter->timestamp_prev = timestamp;
        return x;
    }

    // Dynamically adjust filter coefficient - alpha 
    float alpha = filter->Tc / ( dt + filter->Tc );
    
    // Update filter output
    float y = alpha * filter->y_prev + ( 1.0f - alpha ) * x;
    filter->y_prev = y;
    filter->timestamp_prev = timestamp;
   
    return y;
}
