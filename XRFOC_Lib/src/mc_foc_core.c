/**
 *  @file:      XRFOC_Lib/src/mc_foc_core.c
 *  @brief:     Main FOC's call Func application layer.
 *  @author:    RunDong7582
 *  @date  :    2025 3/21 14:06 -> 2025 4/1 16:29
 *  @version:   XRFOC v0.2
 */

 #include "../include/mc_foc_core.h"
 #include "../include/mc_adaptor.h"
 
 #define _constrain(IMM, Min, Max)   ((IMM)<(Min)?(Min) : ((IMM)>(Max)?(Max):(IMM)))
 #define _sqrt(a)                    (_sqrtApprox(a))
 
 struct Cur_vol_s Cur_vol = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
 
 int cntt;
 extern float Target_flt;
 extern float voltage_power_supply;
 extern Motor_params motor_bldc;
 extern SMO_PLL_Observer SMO;
 extern struct Cul_vol_s Cur_vol;
 extern LowPassFilter CurrentQ_Flt;
 extern LowPassFilter CurrentD_Flt;
 extern PIDController speed_loop;
 extern PIDController position_loop;
 extern PIDController current_q_loop;
 extern PIDController current_d_loop;
 
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
    current_q_loop.P = P;
    current_q_loop.I = I;
    current_q_loop.D = D;
    current_q_loop.ramp = ramp;
    current_q_loop.limit = voltage_power_supply / 2;
}

void xrfoc_set_current_d_pid ( float P, float I, float D, float ramp ) 
{
    current_d_loop.P = P;
    current_d_loop.I = I;
    current_d_loop.D = D;
    current_d_loop.ramp = ramp;
    current_d_loop.limit = 1.2;
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

   Ua = _constrain(Ua, 0.0f, voltage_power_supply);
   Ub = _constrain(Ub, 0.0f, voltage_power_supply);
   Uc = _constrain(Uc, 0.0f, voltage_power_supply);
   
   float dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
   float dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
   float dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);
    
   // write into pwm channel 
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
 
void mc_set_torque ( float Uq, float Ud, float angle_el ) 
{
   Cur_vol.Uq = Uq;
   Cur_vol.Ud = Ud;
   float Uout;
   if (Ud) {
     Uout = _sqrt(Ud * Ud + Uq * Uq) / voltage_power_supply;
     angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));
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
 
    mc_setpwm (Ua, Ub, Uc);
}
 
void xrfoc_enable() {
   // digitalWrite(enable, HIGH);
}
 
void xrfoc_disable() {
   // digitalWrite(enable, LOW);
}
 
void xrfoc_vbus_set (float power_supply) {
   voltage_power_supply = power_supply;   
   VF_smo_init( &SMO );
}
 
//=========================电流读取=========================
 
//通过Ia,Ib,Ic计算Iq,Id(目前仅输出Iq)
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

float xrfoc_current_d_lpf (void) {
   float I_d_filt = LowPassFilter_operator(&CurrentD_Flt, Cur_vol.I_d);
   return I_d_filt;
}
 
/* ============================ SMO FUNC ======================= */
void xrfoc_smo_voltage_set_torque ( float Target ) {
   Target_flt = 0.002 * Target + 0.998 * Target_flt;
   mc_set_torque (Target_flt, 0, SMO.Est_Theta);
}
 
void xrfoc_smo_current_set_torque ( float Target ) {
   Target_flt = 0.002 * Target + 0.998 * Target_flt;
   mc_set_torque (pid_operator(&current_q_loop, Target_flt - xrfoc_current_q_lpf()), 0, SMO.Est_Theta);
}
 
void xrfoc_smo_current_set_speed (float Target) {
   Target_flt = 0.002 * Target + 0.998 * Target_flt;
   if (Target_flt < -5 || Target_flt > 5)
        mc_set_torque(pid_process(&current_q_loop, (xrfoc_speed_pid_tune(Target_flt - SMO.Est_speed / 7) - xrfoc_current_q_lpf())), 0, SMO.Est_Theta);
   else
        mc_set_torque(0, 0, 0);
}
 
// ================简易接口函数================
int xfoc_vf_start ( int num, int dir) {
   if (num == 1) {
     smo_VF_start( &SMO, num, dir);;
     if (SMO.VF_flag == 1)
       return 1;
   }
   return 0;
}
 
void xrfoc_run() {
    
   cntt++;
//    Cur_vol = cal_Iq_Id(CS_M1.current_a, CS_M1.current_b, SMO.Est_Theta);
   smo_closeloop(&SMO, &Cur_vol);
 
   if (cntt > 80) {
     cntt = 0;
   }
}
 