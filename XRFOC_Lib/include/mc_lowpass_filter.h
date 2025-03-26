#ifndef __MC_filter_H
#define __MC_filter_H

/**
 *  @file       XRFOC_Lib/include/mc_lowpass_filter.h
 *  @brief      LowPassFilter first order implemented.
 *  @author:    RunDong7582
 *  @date       2025 3/21 11:02 
 *  @version    XRFOC v0.1
 */

#include "../common/mc_common.h"

volatile uint32_t timer_counter = 0;
 
typedef struct {

    float Tc;    // Wc = 1/(R*C) = 1/Tc | wc :cut-off freq
    float y_prev;
    uint32_t timestamp_prev;

} LowPassFilter;

void  lowpass_filter_init    ( LowPassFilter* filter, float time_constant );
float lowpass_filter_process ( LowPassFilter* filter, float x );

#endif