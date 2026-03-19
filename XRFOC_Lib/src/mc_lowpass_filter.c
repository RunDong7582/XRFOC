/**
 *  @file:      XRFOC_Lib/src/mc_lowpassfilter.c
 *  @brief:     LowPassFilter first order implemented.
 *  @author:    RunDong7582
 *  @date  :    2026 3/19 18:00
 *  @version:   XRFOC v1.0
 */
 #include "../XRFOC_Lib/APP/app.h"
 #include "../XRFOC_Lib/include/mc_lowpass_filter.h"
 #include "../XRFOC_Lib/include/mc_time.h"

 float LowPassFilter_operator ( LowPassFilter* filter, float x )
 {
     uint32_t timestamp = mc_time_us();
 
     float dt = (timestamp - filter->timestamp_prev) * 1e-6f;
 
     if (dt <= 0.0f ) dt = 1e-4f;
     else if(dt > 0.05f) {
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

 
