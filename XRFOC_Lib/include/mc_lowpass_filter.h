#ifndef __MC_filter_H
#define __MC_filter_H

/**
 *  @file       XRFOC_Lib/include/mc_lowpass_filter.h
 *  @brief      LowPassFilter first order implemented.
 *  @author:    RunDong7582
 *  @date       2025.5.19 18:32
 *  @version    XRFOC release version 
 */

typedef struct {

    float Tc;    // Wc = 1/(R*C) = 1/Tc | wc :cut-off freq
    float y_prev;
    unsigned long timestamp_prev;

} LowPassFilter;

float LowPassFilter_operator  ( LowPassFilter* filter, float x );

#endif