/**
 *  @file:      XRFOC.c
 *  @brief:     Motor run task.
 *  @author:    RunDong7582
 *  @date  :    2025 4/1 13:31
 *  @version:   XRFOC v0.2
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../common/mc_common.h"
#include "../mcu_conf/stm32/mc_stm32_conf.h"
#include "../include/mc_foc_core.h"
#include "../include/mc_adaptor.h"

float voltage_power_supply;
float Target_flt;

Motor_params motor_bldc = {
    .pp =   7,
    .dir = -1,
};

SMO_PLL_Observer SMO = {
    .Rs = 8.25f,
    .Ls = 0.004256f,
    .h = 0.3f,
    .PLL_kp = 521.0f,
    .PLL_ki = 40000.0f,
    .VF_acc = 0.3f,
    .VF_max_vel = 210,
    .VF_uq_delta = 0.006f,
};

extern struct Cur_vol_s Cur_vol;

LowPassFilter CurrentQ_Flt =  { 0.002f, 0.0f, 0};   //M1电流环Q
LowPassFilter CurrentD_Flt = { 0.002f, 0.0f, 0};   //M1电流环D

//PID
PIDController speed_loop =    { .P = 2, .I = 0, .D = 0, .ramp = 100000, .limit = 0.0f, .error_prev = 0.0f, .output_prev = 0.0f, .integral_prev = 0.0f, .timestamp_prev = 0};
PIDController position_loop = { .P = 2, .I = 0, .D = 0, .ramp = 100000, .limit = 0.0f, .error_prev = 0.0f, .output_prev = 0.0f, .integral_prev = 0.0f, .timestamp_prev = 0};

PIDController current_q_loop = { .P = 1.2, .I = 0, .D = 0, .ramp = 100000, .limit = 0.0f, .error_prev = 0.0f, .output_prev = 0.0f, .integral_prev = 0.0f, .timestamp_prev = 0};
PIDController current_d_loop = { .P = 1.2, .I = 0, .D = 0, .ramp = 100000, .limit = 0.0f, .error_prev = 0.0f, .output_prev = 0.0f, .integral_prev = 0.0f, .timestamp_prev = 0};


/* ========================== STM32 Periphal =========================== */

void XRFOC_Init()
{
    xr_motor_power_init(&motor_bldc);
}


void xrfoc_module_struct_clear  ( void )
{
    memset(&SMO,         0, sizeof(SMO));
    memset(&Cur_vol,     0, sizeof(Cur_vol));
    memset(&motor_bldc,     0, sizeof(motor_bldc));
	memset(&CurrentQ_Flt,   0, sizeof(CurrentQ_Flt));
	memset(&CurrentD_Flt,   0, sizeof(CurrentD_Flt));
    memset(&speed_loop,     0, sizeof(speed_loop));
	memset(&position_loop,  0, sizeof(position_loop));
	memset(&current_q_loop,    0, sizeof(current_q_loop));
    memset(&current_d_loop,    0, sizeof(current_d_loop));
}

int bldc_init ( struct mc_adaptor_stm32_hw *adaptor )
{
    if (mc_adaptor_stm32_hw_init(adaptor))
    {
        return 0;
    }
}

int main (void)
{
    struct mc_adaptor_stm32_hw adaptor;
    bldc_init(&adaptor);

    return 0;
}
