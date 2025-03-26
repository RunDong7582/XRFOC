/**
 *  @file:      XRFOC_Lib/src/mc_smo_pll.c
 *  @brief:     Sliding Mode Observer & Phase Locked Loop algorithm
 *  @author:    RunDong7582
 *  @date  :    2025/3/26/10:08
 *  @version:   XRFOC v0.1
 */

 #include "../include/mc_smo_pll.h"
 #include "../include/mc_foc_core.h"
 
 float   target_speed_openloop = 0.0f;
 float   shaft_angle;
 uint32_t open_loop_timestamp = 0;
 uint32_t now_us     = 0;
 uint32_t init_time  = 0;
 
 
 void VF_smo_init ( SMO_PLL_Observer *smo )
 {
     smo->VF_flag = 0;
     smo->VF_acc  = 0;
     smo->VF_max_vel = 0;
     smo->VF_uq_delta = 0;
 }
 
 void smo_para_update ( SMO_PLL_Observer *smo, SMO_params self )
 {
     smo->Rs = self.Rs;
     smo->Ls = self.Ls;
     smo->h  = self.h;
     smo->PLL_kp = self.PLL_kp;
     smo->PLL_ki = self.PLL_ki;
     smo->VF_acc = self.VF_acc;
     smo->VF_max_vel = self.VF_max_vel;
     smo->VF_uq_delta = self.VF_uq_delta;
 }
 
 float smo_sat_plane ( float err, float limits )
 {
     if (err > limits) 
         return 1;
     else if (err < -limits) 
         return -1;
     else
         return err / limits;
 }
 
 
 void smo_position_estimate ( SMO_PLL_Observer *smo )
 {
     // Observing the current
     smo->Est_Ialpha += smo->Ts * ((-smo->Rs / smo->Ls) *smo->Est_Ialpha + 1 / smo->Ls * (smo->Ualpha - smo->Ealpha));
     smo->Est_Ibeta  += smo->Ts * ((-smo->Rs / smo->Ls) *smo->Est_Ibeta  + 1 / smo->Ls * (smo->Ubeta - smo->Ebeta));
 
     // Current error -> build the sliding mode plane
     smo->Ialpha_Err = smo->Est_Ialpha - smo->Ialpha;
     smo->Ibeta_Err  = smo->Est_Ibeta  - smo->Ibeta;
 
     smo->Ealpha = smo->h * smo_sat_plane(smo->Ialpha_Err, 0.5f);
     smo->Ebeta  = smo->h * smo_sat_plane(smo->Ibeta_Err , 0.5f);
 
     smo->Ealpha_flt = 0.1 * smo->Ealpha_flt + 0.9 * smo->Ealpha;
     smo->Ebeta_flt  = 0.1 * smo->Ebeta_flt  + 0.9 * smo->Ebeta;
 
     // Fetch angle & speed through pll
     if (smo->_dir = 0)
         smo_pll_parse_eangle( &smo );
     else {
         /* cause bemf = -B*dw/dt*A , dir < 0, bemf * -1 */
         smo->Ealpha_flt *= -1;
         smo->Ebeta_flt  *= -1;
         smo_pll_parse_eangle( &smo );
     }
 }
 
 void smo_pll_parse_eangle ( SMO_PLL_Observer *smo )
 {
     float alpha = smo->Ealpha_flt;
     float beta  = smo->Ebeta_flt;
 
     smo->Theta_err = -1 * alpha * cos(smo->Est_Theta) + beta * sin(smo->Est_Theta) * -1;
     /* pi controller to fetch e_angle */
     /* position */
     smo->i_err += smo->Ts * smo->PLL_ki * smo->Theta_err;
     smo->Est_speed = smo->PLL_kp * smo->Theta_err + smo->i_err;
 
     /* speed filter */
     smo->Est_speed_F = smo->Est_speed_F * 0.9 + smo->Est_speed * 0.1;
 
     smo->Est_Theta += smo->Ts * smo->Est_speed; /* intergral w(omega) to fetch e_angle */
     /* limit the Amplitude of e_angle */
     smo->Est_Theta = _normalizeAngle (smo->Est_Theta);
 }
 
 void smo_closeloop ( SMO_PLL_Observer *smo, xrfoc_cur_vol_s Cur_Vol ) 
 {
     // record the running time
     smo->now_time = timer_counter;
     smo->Ts = (smo->now_time - smo->last_time) * 1e-6f;
     smo->last_time = smo->now_time;
     if ( smo->Ts < 0 || smo->Ts > 5e-3f) smo->Ts = 1e-3f;
 
     // update the variable
     smo->Ialpha = Cur_Vol.I_alpha;
     smo->Ibeta  = Cur_Vol.I_beta;
 
     smo->Ualpha = -sin(smo->Est_Theta) * Cur_Vol.Uq;
     smo->Ubeta  =  cos(smo->Est_Theta) * Cur_Vol.Uq;
 
     smo_position_estimate(smo);
 }
 
 
 void smo_VF_start ( SMO_PLL_Observer *smo , int dir )
 {
     smo->_dir = dir;
     if ( smo->VF_flag == 0 ) {
         init_time = timer_counter;
         now_us = timer_counter;
         smo->VF_flag = 2;
     }
 
     if ( (now_us - init_time) * 1e-3f >= 3000 ) {
             smo->VF_flag = 1;
     }
 
     if ( smo->VF_flag == 2) {
         now_us = timer_counter;
         float Ts = (now_us - open_loop_timestamp) * 1e-3f;
         /* process abnormal condition */
         if ( Ts > 0.01f) Ts = 1e-3f;
 
         /* calculate the angle to achieve target speed, intergral with the angle */
         if ( target_speed_openloop < smo->VF_max_vel ) {
                 target_speed_openloop = target_speed_openloop + smo->VF_acc; //0.3 is 加速度
         } else {
                 target_speed_openloop = smo->VF_max_vel;
         }
         shaft_angle = _normalizeAngle (shaft_angle + dir * target_speed_openloop * Ts );
 
         mc_set_torque (2.0f + target_speed_openloop * smo->VF_uq_delta, 0, shaft_angle);
         open_loop_timestamp = now_us;
     }
 }