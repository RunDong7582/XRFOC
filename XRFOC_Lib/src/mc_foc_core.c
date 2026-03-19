/**
 *  @file:      XRFOC_Lib/src/mc_foc_core.c
 *  @brief:     Main FOC's call Func application layer.
 *  @author:    RunDong7582
 *  @date  :    2026 3/19 18:00
 *  @version:   XRFOC v1.0
 *  @license:   PolyForm Noncommercial License 1.0.0 (Non-Commercial Use Only)
 *              This software is provided for educational and evaluation purposes.
 *              Commercial use is strictly prohibited. See LICENSE for details.
 */

#include "../XRFOC_Lib/include/mc_adaptor.h"
#include "../XRFOC_Lib/include/mc_foc_core.h"

#include <string.h> 
#define _constrain(IMM, Min, Max)   ((IMM)<(Min)?(Min) : ((IMM)>(Max)?(Max):(IMM)))
#define _sqrt(a)                    (_sqrtApprox(a))
#define CURRENT_PID_SAFE_RATIO      (0.85f)
 

 
extern SMO_PLL_Observer SMO;
extern LowPassFilter CurrentQ_Flt;
extern LowPassFilter CurrentD_Flt;
extern PIDController speed_loop;
extern PIDController position_loop;
extern PIDController current_q_loop;
extern PIDController current_d_loop;
extern struct Cur_vol_s Cur_vol;
extern Motor_Console motor_console;
extern Motor_Console_Data motor_I_UVW;

static xrfoc_pwm_write_duty_fn_t s_pwm_write_fn = 0;

void xrfoc_register_pwm_write_fn ( xrfoc_pwm_write_duty_fn_t fn )
{
   s_pwm_write_fn = fn;
}

void xrfoc_module_struct_clear  ( void )
{
   memset(&SMO, 0, sizeof(SMO));
   memset(&Cur_vol, 0, sizeof(Cur_vol));
   memset(&speed_loop, 0, sizeof(speed_loop));
   memset(&CurrentQ_Flt, 0, sizeof(CurrentQ_Flt));
   memset(&CurrentD_Flt, 0, sizeof(CurrentD_Flt));
   memset(&position_loop, 0, sizeof(position_loop));
   memset(&current_q_loop, 0, sizeof(current_q_loop));
   memset(&current_d_loop, 0, sizeof(current_d_loop));
}

/* ===================== PID 设置函数====================== */

void xrfoc_set_speed_pid ( float P, float I, float D, float ramp, float limit ) 
{
    speed_loop.P = P;
    speed_loop.I = I;
    speed_loop.D = D;
    speed_loop.ramp = ramp;
    speed_loop.limit = limit;
}

void xrfoc_set_angle_pid ( float P, float I, float D, float ramp, float limit ) 
{
    position_loop.P = P;
    position_loop.I = I;
    position_loop.D = D;
    position_loop.ramp = ramp;
    position_loop.limit = limit;
}

void xrfoc_set_current_q_pid ( float P, float I, float D, float ramp )  
{
   float linear_limit = motor_console.power * 0.577f;
   float safe_limit = linear_limit * CURRENT_PID_SAFE_RATIO;
   float output_limit = _constrain(safe_limit, 1.2f, linear_limit);

    current_q_loop.P = P;
    current_q_loop.I = I;
    current_q_loop.D = D;
    current_q_loop.ramp = ramp;
	 current_q_loop.limit = output_limit;
//    current_q_loop.limit = motor_console.power;
}

void xrfoc_set_current_d_pid ( float P, float I, float D, float ramp ) 
{
   float linear_limit = motor_console.power * 0.577f;
   float safe_limit = linear_limit * CURRENT_PID_SAFE_RATIO;
   float output_limit = _constrain(safe_limit, 1.2f, linear_limit);

    current_d_loop.P = P;
    current_d_loop.I = I;
    current_d_loop.D = D;
    current_d_loop.ramp = ramp;
   current_d_loop.limit = output_limit;
}
 

float xrfoc_speed_pid_tune ( float error ) {
     return pid_operator (&speed_loop, error);
}

float xrfoc_angle_pid_tune ( float error ) {
     return pid_operator (&position_loop, error);
}
 
float _normalizeAngle ( float angle ) {
   float a = fmod(angle, 2 * _PI);  
   return a >= 0 ? a : (a + 2 * _PI);
}
 
void mc_setpwm ( float Ua, float Ub, float Uc ) {

	Ua = _constrain(Ua, 0.0f, motor_console.power);
	Ub = _constrain(Ub, 0.0f, motor_console.power);
	Uc = _constrain(Uc, 0.0f, motor_console.power);

	float dc_a = _constrain(Ua / motor_console.power, 0.0f, 1.0f);
	float dc_b = _constrain(Ub / motor_console.power, 0.0f, 1.0f);
	float dc_c = _constrain(Uc / motor_console.power, 0.0f, 1.0f);

   if (s_pwm_write_fn) {
      s_pwm_write_fn(dc_a, dc_b, dc_c);
   }
 //  __HAL_TIM_SET_COMPARE(&g_atimx_handle, TIM_CHANNEL_1, dc_a * MAX_PWM_DUTY);
 //  __HAL_TIM_SET_COMPARE(&g_atimx_handle, TIM_CHANNEL_2, dc_b * MAX_PWM_DUTY);
 //  __HAL_TIM_SET_COMPARE(&g_atimx_handle, TIM_CHANNEL_3, dc_c * MAX_PWM_DUTY);
}
 
float _sqrtApprox ( float number ) {  
   long i;
   float y;
   y = number;
   i = *(long*)&y;
   i = 0x5f375a86 - (i >> 1);
   y = *(float*)&i;
   return number * y;
}

float _atan2(float y, float x) {
   // a := min (|x|, |y|) / max (|x|, |y|)
   float abs_y = ABS(y);
   float abs_x = ABS(x);
   float a = min(abs_x, abs_y) / (max(abs_x, abs_y));
   float s = a * a;
   float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
   if (abs_y > abs_x) r = 1.57079637f - r;
   if (x < 0.0f) r = 3.14159274f - r;
   if (y < 0.0f) r = -r;

   return r;
}

// static inline float f1 ( float x ) {
//    float u = 1.3528548e-10f;
//    u = u * x + -2.4703144e-08f;
//    u = u * x + 2.7532926e-06f;
//    u = u * x + -0.00019840381f;
//    u = u * x + 0.0083333179f;
//    return u * x + -0.16666666f;
// }

// static inline float f2 ( float x ) {
//    float u = 1.7290616e-09f;
//    u = u * x + -2.7093486e-07f;
//    u = u * x + 2.4771643e-05f;
//    u = u * x + -0.0013887906f;
//    u = u * x + 0.041666519f;
//    return u * x + -0.49999991f;
// }

// float _sin ( float x ) {

//    int si = (int)(x * 0.31830988f);
//    x = x - (float)si * _PI;
//    if (si & 1) {
//      x = x > 0.0f ? x - _PI : x + _PI;
//    }
//    return x + x * x * x * f1(x * x);
// }
 
// float _cos(float x) {
//    // si = (int)(x / pi)
//    int si = (int)(x * 0.31830988f);
//    x = x - (float)si * _PI;
//    if (si & 1) {
//      x = x > 0.0f ? x - _PI : x + _PI;
//    }
//    return 1.0f + x * x * f2(x * x);
// }

void mc_set_torque ( float Uq, float Ud, float angle_el ) 
{
   Cur_vol.Uq = Uq;
   Cur_vol.Ud = Ud;
   float Uout;
   if (Ud) {
     Uout = _sqrt(Ud * Ud + Uq * Uq) / motor_console.power;
     angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));
   //   angle_el = _normalizeAngle(angle_el + _atan2(Uq, Ud));

   } else {
     Uout = Uq / motor_console.power;
     angle_el = _normalizeAngle(angle_el + _PI_2);
   }
   int sector = floor(angle_el / _PI_3) + 1;
   float T1 = _SQRT3 * sin(sector * _PI_3 - angle_el) * Uout;
   float T2 = _SQRT3 * sin(angle_el - (sector - 1.0f) * _PI_3) * Uout;
   float T0 = Tperiod - T1 - T2; // Tperiod = 1
 
   float sum = T1 + T2;
   if ( sum > Tperiod )
   {
     float k = Tperiod / sum;
     T1 *= k;
     T2 *= k;
   }

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
 
    float Ua = Ta * motor_console.power;
    float Ub = Tb * motor_console.power;
    float Uc = Tc * motor_console.power;
 
    mc_setpwm (Ua, Ub, Uc);
}
 
void xrfoc_vbus_set (float power_supply) {
   motor_console.power = power_supply;   
}

void xrfoc_startup_set_mode ( uint8_t mode )
{
   if (mode == XRFOC_STARTUP_IF) {
      SMO.startup_mode = XRFOC_STARTUP_IF;
   } else {
      SMO.startup_mode = XRFOC_STARTUP_VF;
   }
}
 
void xrfoc_pretarget (int _PP, int _DIR) {
   motor_console.pp  = _PP;
   motor_console.dir = _DIR;
   mc_set_torque (3, 0, _3PI_2); 
   delay_ms(1000);
   mc_set_torque (0, 0, _3PI_2);  
}

//=========================电流读取=========================
 
struct Cur_vol_s cal_Iq_Id(float current_a, float current_b, float angle_el) 
{
   struct Cur_vol_s r_Cur_vol;
   r_Cur_vol.I_alpha = current_a;
   r_Cur_vol.I_beta = _1_SQRT3 * current_a + _2_SQRT3 * current_b;
   float ct = cos(angle_el);
   float st = sin(angle_el);
   r_Cur_vol.I_d = r_Cur_vol.I_alpha * ct + r_Cur_vol.I_beta * st;
   r_Cur_vol.I_q = r_Cur_vol.I_beta * ct - r_Cur_vol.I_alpha * st;
   return r_Cur_vol;
}
 
float xrfoc_current_q_lpf (void) {
  float I_q_filt = LowPassFilter_operator(&CurrentQ_Flt, Cur_vol.I_q);
  return I_q_filt;
}

// float xrfoc_current_q_lpf (void) 
// {
// 	CurrentQ_Flt.timestamp_prev = HAL_GetTick();
// 	float I_q_filt = LowPassFilter_operator(&CurrentQ_Flt, Cur_vol.I_q);
// 	return I_q_filt;
// }

float xrfoc_current_d_lpf (void) {
   float I_d_filt = LowPassFilter_operator(&CurrentD_Flt, Cur_vol.I_d);
   return I_d_filt;
}
 
/* ============================ SMO FUNC ======================= */
void xrfoc_smo_voltage_set_torque ( float Target , Control_Obj type ) {
   if ( type == VOLTAGE_Torque ) {

      motor_I_UVW.Target_flt = 0.002f * Target + 0.998f * motor_I_UVW.Target_flt;
      
      mc_set_torque (motor_I_UVW.Target_flt, 0, SMO.Est_Theta);

   } else {

//      printf("Error: Voltage set torque, please check!\r\n");

   }
}
 
void xrfoc_smo_current_set_torque ( float Target , Control_Obj type ) { 
   if ( type == CURRENT_Torque ) {

      motor_I_UVW.Target_flt = 0.002f * Target + 0.998f * motor_I_UVW.Target_flt;

   float iq_meas = xrfoc_current_q_lpf();
   float iq_error = motor_I_UVW.Target_flt - iq_meas;
   float pid_out = pid_operator(&current_q_loop, iq_error);

      // --- 增加基于测量电流的限流保护 ---
      const float I_max = 10.0f; // 允许的最大相电流 (A)，根据你的电机/驱动设定
      const float SAFE_V_RATIO = 0.5f; // 安全系数，相对 Vbus 的比例（或用 motor_console.power/some_factor）
      float vbus = motor_console.power;
      float v_max_safe = vbus * SAFE_V_RATIO; // 简单安全电压上限
      // 把 pid_out 限幅在 [-v_max_safe, v_max_safe]
      if (pid_out > v_max_safe) pid_out = v_max_safe;
      if (pid_out < -v_max_safe) pid_out = -v_max_safe;

      // 如果测到电流超过 I_max，则进一步衰减 pid_out（简单策略）
        if (fabsf(iq_meas) > I_max) {
           float scale = I_max / (fabsf(iq_meas) + 1e-6f);
          pid_out *= scale; // 线性缩减输出
      }

      mc_set_torque (pid_out, 0, SMO.Est_Theta);
      
//      // 考虑id
//      //mc_set_torque (pid_operator(&current_q_loop, motor_I_UVW.Target_flt - xrfoc_current_q_lpf()), pid_operator(&current_d_loop, -(xrfoc_current_d_lpf())), SMO.Est_Theta);

   } else {
      
//      printf("Error: Current set torque, please check!\r\n");
   
   }
}
 
void xrfoc_smo_current_set_speed ( float Target , Control_Obj type) {
   if ( type == SPEED_Torque) {

      motor_I_UVW.Target_flt = 0.002f * Target + 0.998f * motor_I_UVW.Target_flt;

    if (motor_I_UVW.Target_flt < -5.0f || motor_I_UVW.Target_flt > 5.0f ) {
       float mech_speed_est = SMO.Est_speed / motor_console.pp;
       float speed_error = motor_I_UVW.Target_flt - mech_speed_est;

       /* 速度环输出定义为 Iq_ref */
       float iq_ref = xrfoc_speed_pid_tune(speed_error);

       /* 电流环误差 */
       float iq_meas = xrfoc_current_q_lpf();
       float iq_error = iq_ref - iq_meas;

       /* 电流环输出定义为 Uq_cmd */
       float uq_cmd = pid_operator(&current_q_loop, iq_error);
       uq_cmd = _constrain(uq_cmd, -current_q_loop.limit, current_q_loop.limit);

       mc_set_torque(uq_cmd, 0.0f, SMO.Est_Theta);

       // 当然也可以使用更准确的 ud != 0 的方式来实现，原代码中注释了这句，使用了上一种方式
       // mc_set_torque(pid_operator(&current_q_loop, (xrfoc_speed_pid_tune(motor_I_UVW.Target_flt - SMO.Est_speed / 2) - xrfoc_current_q_lpf())), pid_operator(&current_d_loop, -(xrfoc_current_d_lpf())), SMO.Est_Theta);
    } else {
       mc_set_torque(0.0f, 0.0f, SMO.Est_Theta);
    }

   } else {
      
//      printf("Error: Speed set torque, please check!\r\n");

   }
}
 
// ================简易接口函数================
int xrfoc_vf_start ( uint8_t num, uint8_t dir) {

   if (num == 0) { // 电机motor0
      // smo_VF_start( &SMO_M0, num, dir);
      // if (SMO_M0.VF_flag == 1)
      //   return 1;
//      printf("Warning: Please create corresponding struct as well!\r\n");
   }
   if (num == 1) { // 电机motor1 选用
     if (SMO.startup_mode == XRFOC_STARTUP_IF) {
        smo_IF_start(&SMO, num, dir);
     } else {
        smo_VF_start(&SMO, num, dir);
     }
     if (SMO.VF_flag == 1)
       return 1;
   }
   return 0;

}
