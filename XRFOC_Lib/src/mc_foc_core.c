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
 
 
 extern PIDController current_q_loop;
 extern PIDController current_d_loop;
 
 extern PIDController speed_loop;
 extern PIDController position_loop;
 
 extern LowPassFilter Speed_Flt;
 extern LowPassFilter CurrentQ_Flt;
 extern LowPassFilter CurrentD_Flt;
 
 extern xrfoc_cur_vol_s  Cur_Vol;
 extern xrfoc_origin_t  xfoc_orgin;
 
 extern SMO_PLL_Observer SMO;
 
 extern float voltage_power_supply; 
 extern struct mc_adaptor_i stm32_adaptor;
 extern float filter_ouput;
 extern float serial_target;
 
 int pt_cntt;
 
 /* ================ PID SET FUNC =================== */
 /*        Speed pid             */
 void xrfoc_set_speed_pid     ( float P, float I, float D, float ramp, float limit ) 
 {
     speed_loop.P = P;
     speed_loop.I = I;
     speed_loop.D = D;
     speed_loop.output_ramp = ramp;
     speed_loop.limit = limit;
 }
 
 /*        Angle pid             */
 void xrfoc_set_angle_pid     ( float P, float I, float D, float ramp, float limit ) 
 {
     position_loop.P = P;
     position_loop.I = I;
     position_loop.D = D;
     position_loop.output_ramp = ramp;
     position_loop.limit = limit;
 }
 
 /*        Q-axis pid            */
 void xrfoc_set_current_q_pid ( float P, float I, float D, float ramp ) 
 {
     current_q_loop.P = P;
     current_q_loop.I = I;
     current_q_loop.D = D;
     current_q_loop.output_ramp = ramp;
     current_q_loop.limit = voltage_power_supply / 2;
 }
 
 /*        D-axis pid            */
 void xrfoc_set_current_d_pid ( float P, float I, float D, float ramp )  
 {
     current_d_loop.P = P;
     current_d_loop.I = I;
     current_d_loop.D = D;
     current_d_loop.output_ramp = ramp;
     current_d_loop.limit = 1.2;
 }
 
 /* ================ PID SET interface =================== */
 /*        Speed pid             */
 
 float xrfoc_speed_pid_tune ( float error )  
 {
     return pid_process(&speed_loop, error); 
 }
 
 /*        Angle pid             */
 float xrfoc_angle_pid_tune ( float error ) 
 {
     return pid_process(&position_loop, error);
 }
 
 /* =============== Float Type return Process FUNC ================= */
 float _normalizeAngle ( float angle ) 
 {
     float a = fmod(angle, 2 * _PI);  
     return a >= 0 ? a : (a + 2 * _PI);
 }
 
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
 
 /* =============== Float Type return Process FUNC ================ */
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
 
 /* =============== Update the PWM config to Controller =========== */
 void mc_setpwm ( float Ua, float Ub, float Uc ) {
 
   Ua = _constrain(Ua, 0.0f, voltage_power_supply);
   Ub = _constrain(Ub, 0.0f, voltage_power_supply);
   Uc = _constrain(Uc, 0.0f, voltage_power_supply);
 
   float dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
   float dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
   float dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);
 
   ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, dc_a * 255);
   ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
   ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, dc_b * 255);
   ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);
   ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5, dc_c * 255);
   ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5);
 }
 /* ================= SVPWM To be optimized ======================= */
 void mc_set_torque ( float Uq, float Ud, float angle_el ) {
     Cur_Vol.Uq = Uq;
     Cur_Vol.Ud = Ud;
     float Uout;
     if (Ud) {
         Uout = _sqrt(Ud * Ud + Uq * Uq) / voltage_power_supply;
         angle_el = _normalizeAngle(angle_el + _atan2(Uq, Ud));
         // angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));
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
 
     #ifdef STM32
     mc_pwm_set_duty(&stm32_adaptor, PWM_A_CH, Ua);
     mc_pwm_set_duty(&stm32_adaptor, PWM_B_CH, Ub);
     mc_pwm_set_duty(&stm32_adaptor, PWM_C_CH, Uc); 
     #endif 
     #ifdef ESP32 
     mc_setpwm(Ua,Ub,Uc);
     #endif
 }
 
 void xrfoc_enable ( void ) {
     mc_button_on_off(&stm32_adaptor, HIGH);
 }
 
 void xrfoc_disable ( void ) {
     mc_button_on_off(&stm32_adaptor, LOW);
 }
 
 void xrfoc_vbus_set ( float power_supply ) {
     voltage_power_supply = power_supply;
 } 
 
 xrfoc_cur_vol_s xrfoc_Iq_Id_calc (float current_a, float current_b, float angle_el) 
 {
     xrfoc_cur_vol_s Cur_Vol_child;
     Cur_Vol_child.I_alpha = current_a;
     Cur_Vol_child.I_beta = _1_SQRT3 * current_a + _2_SQRT3 * current_b;
     float ct = cos(angle_el);
     float st = sin(angle_el);
     Cur_Vol_child.I_d = Cur_Vol_child.I_alpha * ct + Cur_Vol_child.I_beta * st;
     Cur_Vol_child.I_q = Cur_Vol_child.I_beta * ct - Cur_Vol_child.I_alpha * st;
     return Cur_Vol_child;
 }
 
 /* current filtering */
 /* as for the INA current sensing sensor, it is necessary to be filtered */
 /* but if you use the adc on chip, it is best not to add filtering , which causes lags */
 float xrfoc_current_q ( void ) {
     float filtered_I_q = lowpass_filter_process(&CurrentQ_Flt, Cur_Vol.I_q);
     return filtered_I_q;
 }
 
 float xrfoc_current_d ( void ) {
     float filtered_I_d = lowpass_filter_process(&CurrentD_Flt, Cur_Vol.I_d);
     return filtered_I_d;
 }
 
 /* ==================== SMO Voltage Torque ====================== */
 void xrfoc_smo_voltage_set_torque ( float input ) {
     filter_ouput = 0.002 * input + 0.998 * filter_ouput;
     mc_set_torque(filter_ouput, 0, SMO.Est_Theta);
 }
 
 /* ==================== SMO Current Torque ====================== */
 void xrfoc_smo_current_set_torque ( float input ) {
     filter_ouput = 0.002 * input + 0.998 * filter_ouput;
     mc_set_torque(pid_process(&current_q_loop, filter_ouput - xrfoc_current_q()), 0, SMO.Est_Theta);
 }
 
 /* ==================== SMO Speed Control ======================= */
 void xrfoc_smo_current_set_speed ( float input ) {
     filter_ouput = 0.002 * input + 0.998 * filter_ouput;
     if ( filter_ouput < -5 || filter_ouput > 5)
         mc_set_torque(pid_process(&current_q_loop, (xrfoc_speed_pid_tune(filter_ouput - SMO.Est_speed / 7) - xrfoc_current_q())), 0, SMO.Est_Theta);
     else
         mc_set_torque(0, 0, 0);
 }
 
 /* ===== Simple interface func ===== */
 // need sensor ! User could add by yourself
 // void xrfoc_current_set_torque ( float input )
 // {
 //     mc_set_torque(pid_process(&current_q_loop, input - xrfoc_current_q()), pid_process(&current_d_loop, -xrfoc_current_d()), S1_electricalAngle());
 // }
 
 // void xrfoc_voltage_set_torque ( float input )
 // {
 //     mc_set_torque(input, 0, S1_electricalAngle());
 // }
 
 // void xrfoc_set_speed_angle ( float input )
 // {
 //     xrfoc_voltage_set_torque(xrfoc_speed_pid_tune(input - SMO.Est_speed));
 // }
 
 // .....
 
 int xfoc_vf_start ( int dir )
 {
     smo_VF_start(dir);
     if (SMO.VF_flag == 1)
         return 1;
 
     return 0;
 }
 
 void First_Target ( float tar ) {
     serial_target = tar;
 }
 
 void xrfoc_getphase_current ( void ) {
     mc_adc_read(&stm32_adaptor, CURRENT_A_ADC_CH, &xrfoc_origin.I_a);
     mc_adc_read(&stm32_adaptor, CURRENT_B_ADC_CH, &xrfoc_origin.I_b);
     mc_adc_read(&stm32_adaptor, CURRENT_C_ADC_CH, &xrfoc_origin.I_c);
 }
 
 void xrfoc_run ( void ) 
 {
     xrfoc_getphase_current();
     Cur_Vol = xrfoc_Iq_Id_calc(xrfoc_origin.I_a, xrfoc_origin.I_b, SMO.Est_Theta);
     smo_closeloop(&Cur_Vol);
     if ( pt_cntt++ >20 )
     {
         pt_cntt = 0;
         // print something.
     }
 }
 