/**
 *  @file:      XRFOC_Lib/src/mc_lowpassfilter.c
 *  @brief:     LowPassFilter first order implemented.
 *  @author:    RunDong7582
 *  @date  :    2025 3/21 11:06 -> 2025 4/1 16:30
 *  @version:   XRFOC v0.2
 */

 #include "../include/mc_lowpass_filter.h"

 float LowPassFilter_operator ( LowPassFilter* filter, float x )
 {
     // unsigned long timestamp = micros();
     unsigned long timestamp = 0;
 
     float dt = (timestamp - filter->timestamp_prev)*1e-6f;
 
     if (dt < 0.0f ) dt = 1e-3f;
     else if(dt > 0.3f) {
         filter->y_prev = x;
         filter->timestamp_prev = timestamp;
         return x;
     }
 
     float alpha = filter->Tc/(filter->Tc + dt);
     float y = alpha* filter->y_prev + (1.0f - alpha) * x;
     filter->y_prev = y;
     filter->timestamp_prev = timestamp;
     return y;
 }
 