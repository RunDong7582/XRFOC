/**
 *  @file:      XRFOC_Lib/src/mc_foc_core.c
 *  @brief:     Main FOC's call Func application layer.
 *  @author:    RunDong7582
 *  @date  :    2025 3/21 14:06 
 *  @version:   XRFOC v0.1
 */

#include "../include/mc_foc_core.h"
#include "../include/mc_adaptor.h"

#define _constrain(IMM, Min, Max)   ((IMM)<(Min)?(Min) : ((IMM)>(Max)?(Max):(IMM)))
#define _sqrt(a)                    (_sqrtApprox(a))

/* ------- extern declare ------- */
extern PIDController current_q_loop;
extern PIDController current_d_loop;
extern PIDController speed_loop;
extern PIDController position_loop;

extern LowPassFilter Speed_Flt;
extern LowPassFilter CurrentQ_Flt;
extern LowPassFilter CurrentD_Flt;

/* ------- global variable ------- */
xfoc_vol_cur_t      xfoc_core;
xfoc_origin_t       xfoc_origin;

float voltage_power_supply; 
extern struct mc_adaptor_i stm32_adaptor;
extern float filter_ouput;
extern float serial_target;

/* ====== PID SET FUNC ======== */
/*        Speed pid             */
void XFOC_set_speed_pid     ( float P, float I, float D, float ramp, float limit ) 
{
    speed_loop.P = P;
    speed_loop.I = I;
    speed_loop.D = D;
    speed_loop.output_ramp = ramp;
    speed_loop.limit = limit;
}

/*        Angle pid             */
void XFOC_set_angle_pid     ( float P, float I, float D, float ramp, float limit ) 
{
    position_loop.P = P;
    position_loop.I = I;
    position_loop.D = D;
    position_loop.output_ramp = ramp;
    position_loop.limit = limit;
}

/*        Q-axis pid            */
void XFOC_set_current_q_pid ( float P, float I, float D, float ramp ) 
{
    current_q_loop.P = P;
    current_q_loop.I = I;
    current_q_loop.D = D;
    current_q_loop.output_ramp = ramp;
    current_q_loop.limit = voltage_power_supply / 2;
}

/*        D-axis pid            */
void XFOC_set_current_d_pid ( float P, float I, float D, float ramp )  
{
    current_d_loop.P = P;
    current_d_loop.I = I;
    current_d_loop.D = D;
    current_d_loop.output_ramp = ramp;
    current_d_loop.limit = 1.2;
}

/* ===== PID SET interface ==== */
/*        Speed pid             */

float XFOC_speed_pid_tune ( float error )  
{
    return pid_process(&speed_loop, error); 
}

/*        Angle pid             */
float XFOC_angle_pid_tune ( float error ) 
{
    return pid_process(&position_loop, error);
}

/* ====== Float Type return Process FUNC ======== */
float _normalizeAngle ( float angle ) 
{
    float a = fmod(angle, 2 * PI);  
    return a >= 0 ? a : (a + 2 * PI);
}

/* ==== Fast Inverse Square Root FUNC ==== */
float _sqrtApprox ( float number ) {  //low in fat
    long i;
    float y;
    // float x;
    // const float f = 1.5F; // better precision
  
    // x = number * 0.5F;
    y = number;
    i = *(long*)&y;
    i = 0x5f375a86 - (i >> 1);
    y = *(float*)&i;
    // y = y * ( f - ( x * y * y ) ); // better precision
    return number * y;
}

/* ==== Fast arctan ==== */
/* Talyor split for atan  */
float _atan2 ( float y, float x ) {
    // a := min (|x|, |y|) / max (|x|, |y|)
    float abs_y = fabsf(y);
    float abs_x = fabsf(x);
    // inject FLT_MIN in denominator to avoid division by zero
    float a = min(abs_x, abs_y) / (max(abs_x, abs_y));
    // s := a * a
    float s = a * a;
    // r := ((-0.0464964749 * s + 0.15931422) * s - 0.327622764) * s * a + a
    float r =
        ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
    // if |y| > |x| then r := 1.57079637 - r
    if (abs_y > abs_x) r = 1.57079637f - r;
    // if x < 0 then r := 3.14159274 - r
    if (x < 0.0f) r = 3.14159274f - r;
    // if y < 0 then r := -r
    if (y < 0.0f) r = -r;

    return r;
}

void mc_set_torque ( float Uq, float Ud, float angle_el ) {
    xfoc_core.Uq = Uq;
    xfoc_core.Ud = Ud;
    float Uout;
    if (Ud) {
        Uout = _sqrt(Ud * Ud + Uq * Uq) / voltage_power_supply;
        // angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));
        angle_el = _normalizeAngle(angle_el + _atan2(Uq, Ud));
        // the angle_el have to move the offset , dut to the Ud (!= 0)
        // to optimize atan , we can use fast atan2.
    } else {
        Uout = Uq / voltage_power_supply;
        angle_el = _normalizeAngle(angle_el + _PI_2);
    }
    int sector = floor(angle_el / _PI_3) + 1;
    float T1 = _SQRT3 * sin(sector * _PI_3 - angle_el) * Uout;
    float T2 = _SQRT3 * sin(angle_el - (sector - 1.0f) * _PI_3) * Uout;
    float T0 = 1 - T1 - T2; 

    float Ta, Tb, Tc;
    switch (sector) {
        case 1:
            Ta = T1 + T2 + T0 / 2;
            Tb = T2 + T0 / 2;
            Tc = T0 / 2;
            break;
        case 2:
            Ta = T1 + T0 / 2;
            Tb = T1 + T2 + T0 / 2;
            Tc = T0 / 2;
            break;
        case 3:
            Ta = T0 / 2;
            Tb = T1 + T2 + T0 / 2;
            Tc = T2 + T0 / 2;
            break;
        case 4:
            Ta = T0 / 2;
            Tb = T1 + T0 / 2;
            Tc = T1 + T2 + T0 / 2;
            break;
        case 5:
            Ta = T2 + T0 / 2;
            Tb = T0 / 2;
            Tc = T1 + T2 + T0 / 2;
            break;
        case 6:
            Ta = T1 + T2 + T0 / 2;
            Tb = T0 / 2;
            Tc = T1 + T0 / 2;
            break;
        default:
            Ta = 0;
            Tb = 0;
            Tc = 0;
    }

    float Ua = Ta * voltage_power_supply;
    float Ub = Tb * voltage_power_supply;
    float Uc = Tc * voltage_power_supply;

    mc_pwm_set_duty(&stm32_adaptor, PWM_A_CH, Ua);
    mc_pwm_set_duty(&stm32_adaptor, PWM_B_CH, Ub);
    mc_pwm_set_duty(&stm32_adaptor, PWM_C_CH, Uc);

}

void xfoc_enable ( void ) {
    mc_button_on_off(&stm32_adaptor, HIGH);
}

void xfoc_disable ( void ) {
    mc_button_on_off(&stm32_adaptor, LOW);
}

void xfoc_vbus_set ( float power_supply ) {
    voltage_power_supply = power_supply;
} 

xfoc_vol_cur_t xfoc_Iq_Id_calc (float current_a, float current_b, float angle_el) 
{
    xfoc_vol_cur_t xfoc_core_child;
    xfoc_core_child.I_alpha = current_a;
    xfoc_core_child.I_beta = _1_SQRT3 * current_a + _2_SQRT3 * current_b;
    float ct = cos(angle_el);
    float st = sin(angle_el);
    xfoc_core_child.I_d = xfoc_core_child.I_alpha * ct + xfoc_core_child.I_beta * st;
    xfoc_core_child.I_q = xfoc_core_child.I_beta * ct - xfoc_core_child.I_alpha * st;
    return xfoc_core_child;
}

/* current filtering */
/* as for the INA current sensing sensor, it is necessary to be filtered */
/* but if you use the adc on chip, it is best not to add filtering , which causes lags */
float xfoc_current_q ( void ) {
    float filtered_I_q = filter_process(&CurrentQ_Flt, xfoc_core.I_q);
    return filtered_I_q;
}

float xfoc_current_d ( void ) {
    float filtered_I_d = filter_process(&CurrentD_Flt, xfoc_core.I_d);
    return filtered_I_d;
}

/* ===== SMO Voltage Torque ===== */
void xfoc_smo_voltage_set_torque ( float input )
{
    filter_ouput = 0.002 * input + 0.998 * filter_ouput;
    mc_set_torque(filter_ouput, 0, SMO.est_theta);
}

/* ===== SMO Current Torque ===== */
void xfoc_smo_current_set_torque ( float input )
{
    filter_ouput = 0.002 * input + 0.998 * filter_ouput;
    mc_set_torque(pid_process(&current_q_loop, filter_ouput - xfoc_current_q()), 0, SMO.est_theta);
}

/* ===== SMO Speed Control ===== */
void xfoc_smo_current_set_speed ( float input )
{
    filter_ouput = 0.002 * input + 0.998 * filter_ouput;
    if ( filter_ouput < -5 || filter_ouput > 5)
    {
        mc_set_torque(pid_process(&current_q_loop, (XFOC_speed_pid_tune(filter_ouput - SMO.est_speed / 7) - xfoc_current_q())), 0, SMO.est_theta);
    }
    else
    {
        mc_set_torque(0, 0, 0);
    }
}

/* ===== Simple interface func ===== */
// need sensor !
// void xfoc_current_set_torque ( float input )
// {
//     mc_set_torque(pid_process(&current_q_loop, input - xfoc_current_q()), pid_process(&current_d_loop, -xfoc_current_d()), S1_electricalAngle());
// }

// void xfoc_voltage_set_torque ( float input )
// {
//     mc_set_torque(input, 0, S1_electricalAngle());
// }

// void xfoc_set_speed_angle ( float input )
// {
//     xfoc_voltage_set_torque(XFOC_speed_pid_tune(input - SMO.est_speed));
// }

int vf_start ( int num, int dir )
{
    if (num == 0) {
        SMO._VF_start(num , dir);
        if (SMO.VF_flag == 1) 
            return 1;
    }
    if (num == 1) {
        SMO._VF_start(num, dir);
        if (SMO.VF_flag == 1)
            return 1;
    }
    return 0;
}

void First_Target ( float tar ) {
    serial_target = tar;
}

void xfoc_getphase_current ( void ) {
    mc_adc_read(&stm32_adaptor, CURRENT_A_ADC_CH, &xfoc_origin.I_a);
    mc_adc_read(&stm32_adaptor, CURRENT_B_ADC_CH, &xfoc_origin.I_b);
    mc_adc_read(&stm32_adaptor, CURRENT_C_ADC_CH, &xfoc_origin.I_c);
}

void xrfoc_runloop ( void ) 
{
    xfoc_getphase_current();

    xfoc_core = xfoc_Iq_Id_calc(xfoc_origin.I_a, xfoc_origin.I_b, SMO.est_theta);

    xfoc_smo_close_loop(&xfoc_core);
}
