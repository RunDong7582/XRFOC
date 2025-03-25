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
#include "../include/mc_adaptor.h"

/* ------ Global User Define --------- */
#define _sqrt(a)                  ( _sqrtApprox(a) )
#define _constrain(IMM, Min, Max) ((IMM)<(Min)?(Min) : ((IMM)>(Max)?(Max):(IMM)))

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

typedef enum {
    STATE_PREALIGN,
    STATE_OPENLOOP,
    STATE_CLOSEDLOOP,
} FOCState;

/* ------ Extern Global struct  -------- */
extern struct mc_adaptor_i stm32_adaptor;

/* ------ Global User Variable --------- */
float voltage_power_supply = 12.0f;
float serial_target;
float filter_ouput;
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

int xfoc_module_init (void)
{
    /* lpf speed d-q initialize */
    filter_Init(&Speed_Flt, 0.01f);         //速度环滤波 Tc = 0.01s <=> 带宽100Hz

    /* lpf curr d-q initialize */
    filter_Init(&CurrentQ_Flt, 0.002f);     //电流环 Q轴滤波 Tc = 0.002s <=> 带宽500Hz   
    filter_Init(&CurrentD_Flt, 0.002f);     //电流环 D轴滤波 Tc = 0.002s <=> 带宽500Hz
    
    /* pid speed (w omega) initialize */
    pid_init(&speed_loop,      2.0f, 0.0f, 0.0f, 100000.0f, voltage_power_supply / 2.0f);
   
    /* pid position (angle) intialize */
    pid_init(&position_loop,   2.0f, 0.0f, 0.0f, 100000.0f, voltage_power_supply / 2.0f);

    /* pid curr d-q initialize */
    pid_init(&current_q_loop,  1.2f, 0.0f, 0.0f, 100000.0f, voltage_power_supply / 2.0f);
    pid_init(&current_d_loop,  1.2f, 0.0f, 0.0f, 100000.0f, voltage_power_supply / 2.0f);
    
    /* VBUS */
    xfoc_vbus_set(12.0f);
    /* User add encoder initialize */
    /* sensor_as5600_init(&sensor0); */
    /* sensor_as5600_init(&sensor1); */

    /* sensor protocol */
    /* s0_twowire_iic_init(&sensor0); */
    /* s1_twowire_iic_init(&sensor1); */

    
    /* Periphal config */
    mc_button_init(&stm32_adaptor);
    mc_pwm_init(&stm32_adaptor, PWM_Freq, PWM_Arr_bit);
    mc_adc_init(&stm32_adaptor);
    mc_timer_init(&stm32_adaptor, timer_freq);
    mc_timer_start(&stm32_adaptor);
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


/* 用于shell控制、解析上位机发送指令 */
/* refer to my project " Gesture recognition based on Raspiberry 4B & STM32 G4 "*/
/* strtok is very equal to this job */
void serialReceiveUserCommand ( void ) 
{

}

float serial_motor_target ( void ) 
{
    return serial_target;
}