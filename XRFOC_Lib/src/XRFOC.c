/**
 *  @file:      XRFOC.c
 *  @brief:     Motor run task
 *  @author:    RunDong7582
 *  @date  :    2025 3/21 14:32
 *  @version:   XRFOC v0.1
 */

#include <stdio.h>

/* ------------ inc ------------------- */
#include "../common/mc_common.h"
#include "../include/mc_pid.h"
#include "../include/mc_lowpassfilter.h"
#include "../include/mc_foc_core.h"
#include "../include/mc_motor.h"

/* ------ Global User Define --------- */
#define _sqrt(a)                  ( _sqrtApprox(a) )
#define _constrain(IMM, Min, Max) ((IMM)<(Min)?(Min) : ((IMM)>(Max)?(Max):(IMM)))

#define _PI                         3.14159265359f
#define _PI_2                       1.57079632679f
#define _PI_3                        1.0471975512f
#define _3PI_2                      4.71238898038f
#define _SQRT3_2                    0.86602540378f
#define _SQRT3                      1.73205080757f
#define _SQRT3_2                    0.86602540378f
#define _1_SQRT3                    0.57735026919f
#define _2_SQRT3                    1.15470053838f

/* ------ Global Event Enum ------------ */
enum {

    EVENT_NONE = 0,
    EVENT_FAULT,
    EVENT_OVER_CURRENT,
    EVENT_OVER_VOLTAGE,
    EVENT_OVER_SPEED,
    EVENT_OVER_TEMPERATURE,
    EVENT_UNDER_VOLTAGE,
    EVENT_UNDER_SPEED,
    EVENT_UNDER_TEMPERATURE,
    EVENT_OVER_POSITION,
    EVENT_UNDER_POSITION,
    EVENT_OVER_CURRENT_D,
    EVENT_OVER_CURRENT_Q,
    EVENT_OVER_SPEED_D,
    EVENT_OVER_SPEED_Q,
    EVENT_OVER_TEMPERATURE_D,
    EVENT_OVER_TEMPERATURE_Q,
    EVENT_OVER_VOLTAGE_D,
    EVENT_OVER_VOLTAGE_Q,
    EVENT_UNDER_VOLTAGE_D,
    EVENT_UNDER_VOLTAGE_Q,
    EVENT_UNDER_SPEED_D,
    EVENT_UNDER_SPEED_Q,
    EVENT_UNDER_TEMPERATURE_D,
    EVENT_UNDER_TEMPERATURE_Q,
    EVENT_OVER_POSITION_D,
    EVENT_OVER_POSITION_Q,
    EVENT_UNDER_POSITION_D,
    EVENT_UNDER_POSITION_Q,
    EVENT_MAX

} XRFOC_EVENT;


/* ------ Global User Variable --------- */
float voltage_power_supply = 12.0f;


/* ------ Global User Struct --------- */



/**
 *  @addtogroup : PIDController
 *  @brief      : Initialize the PID struct for three loop.
 *  
 */
/* CURRENT  LOOP */
PIDController current_q_loop;
PIDController current_d_loop;
/* SPEED    LOOP */
PIDController speed_loop;
/* POSITION LOOP */
PIDController position_loop;

/**
 *  @addtogroup : LowPassFilter
 *  @brief      : Initialize the LowPassFilter struct for speed & current loop.
 *  
 */
LowPassFilter Speed_Flt;
LowPassFilter CurrentQ_Flt;
LowPassFilter CurrentD_Flt;

int foc_module_init (void)
{
    /* lpf curr d-q initialize */
    filter_Init(&CurrentQ_Flt, 0.002f);     //电流环 Q轴滤波 Tc = 0.002s <=> 带宽500Hz   
    filter_Init(&CurrentD_Flt, 0.002f);     //电流环 D轴滤波 Tc = 0.002s <=> 带宽500Hz

    /* lpf speed d-q initialize */
    filter_Init(&Speed_Flt, 0.01f);         //速度环滤波 Tc = 0.01s <=> 带宽100Hz

    /* pid curr d-q initialize */
    pid_init(&current_q_loop,  1.2f, 0.0f, 0.0f, 100000.0f, voltage_power_supply / 2.0f);
    pid_init(&current_d_loop,  1.2f, 0.0f, 0.0f, 100000.0f, voltage_power_supply / 2.0f);
    
    /* pid speed initialize */
    pid_init(&speed_loop,      2.0f, 0.0f, 0.0f, 100000.0f, voltage_power_supply / 2.0f);
   
    /* pid position intialize */
    pid_init(&position_loop,   2.0f, 0.0f, 0.0f, 100000.0f, voltage_power_supply / 2.0f);

    return 1;
}

int main (void)
{
    if (xfoc_module_init())
    {
        printf("Foc module init success.\n");
    }
    else
    {
        printf("Foc module init failed.\n");
        return 2;
    }

    return 0;
}


