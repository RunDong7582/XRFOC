/**
 *  @file:      XRFOC_Lib_on_stm32/src/mc_lowpassfilter.c
 *  @brief:     LowPassFilter first order implemented.
 *  @author:    RunDong7582
 *  @date  :    2025.5.19 18:46
 *  @version:   XRFOC release version
 */
 #include "../XRFOC_Lib/APP/app.h"
 #include "../XRFOC_Lib/include/mc_lowpass_filter.h"

 float LowPassFilter_operator ( LowPassFilter* filter, float x )
 {
     uint32_t timestamp = HAL_GetTick();
 
     float dt = (timestamp - filter->timestamp_prev) * 1e-3f;
 
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

 
