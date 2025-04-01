/**
 *  @file:      XRFOC_Lib/src/mc_smo_pll.c
 *  @brief:     Sliding Mode Observer & Phase Locked Loop algorithm
 *  @author:    RunDong7582
 *  @date  :    2025 3/26 10:08 -> 2025 4/1 16:31
 *  @version:   XRFOC v0.2
 */

 #include "../include/mc_smo_pll.h"
 #include "../include/mc_foc_core.h"
  
/* 滑模VF强拖标志位清零*/
void VF_smo_init ( SMO_PLL_Observer *smo )
{
    smo->VF_flag = 0;
    smo->VF_acc  = 0;
    smo->VF_max_vel = 0;
    smo->VF_uq_delta = 0;
}

/**********************************************************
 **
* 滑模闭环
*/
void smo_closeloop ( SMO_PLL_Observer *smo, struct Cur_vol_s *Cur_vol ) {
//   smo->now_time = micros();
    smo->now_time = 0;
    smo->Ts = (smo->now_time - smo->last_time) * 1e-6f;
    smo->last_time = smo->now_time;
    if (smo->Ts < 0 || smo->Ts > 5e-3f) smo->Ts = 1e-3f;

    //更新变量
    smo->Ialpha = Cur_vol->I_alpha;
    smo->Ibeta = Cur_vol->I_beta;

    // Ualpha =Cur_vol.Ud*cos(smo->Est_Theta)-sin(smo->Est_Theta)*Cur_vol.Uq;
    // Ubeta = Cur_vol.Ud*sin(smo->Est_Theta)+cos(smo->Est_Theta)*Cur_vol.Uq;

    smo->Ualpha = -sin(smo->Est_Theta) * Cur_vol->Uq;
    smo->Ubeta = cos(smo->Est_Theta) * Cur_vol->Uq;

    smo_position_estimate(smo);
}

/**********************************************************
 **
* 滑模符号函数(饱和函数)
*/
float smo_sat_plane ( float err, float limits ) {
if (err > limits ) return 1;
else if (err < -limits) return -1;
else
    return err / limits;
}

/**********************************************************
 **
* 滑模观测部分-计算反电动势-
*/
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
    if (smo->_dir >= 0)
        smo_pll_calc( smo , smo->Ealpha_flt, smo->Ebeta_flt );
    else {
        /* cause bemf = -B*dw/dt*A , dir < 0, bemf * -1 */
        smo_pll_calc( smo , -smo->Ealpha_flt, -smo->Ebeta_flt );
    }
}

/**********************************************************
 **
* 锁相环部分
# 解析反电动势中的角度
*/
void smo_pll_calc ( SMO_PLL_Observer *smo, float alpha, float beta ) {

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
/**********************************************************
 **
* 初始启动部分
* VF启动
# num:区分电机0和电机1
# dir启动的方向
*/
float Target_Vel_openloop = 0;
uint32_t open_loop_timestamp = 0;
uint32_t now_us = 0;
uint32_t init_time = 0;
float shaft_angle;

void smo_VF_start ( SMO_PLL_Observer *smo , int num, int dir ) {
smo->_dir = dir;
if ( smo->VF_flag == 0 ) {
    //   init_time = micros();
        init_time = 0;
    //   now_us = micros();
        now_us = 0;
    smo->VF_flag = 2;
}

if ( (now_us - init_time) * 1e-3f >= 3000 ) {
        smo->VF_flag = 1;
}

if ( smo->VF_flag == 2) {
    // now_us = micros();
    now_us = 0;
    float Ts = (now_us - open_loop_timestamp) * 1e-3f;
    /* process abnormal condition */
    if (Ts <= 0 || Ts > 0.01f)
    Ts = 1e-3f;

    /* calculate the angle to achieve target speed, intergral with the angle */
    if ( Target_Vel_openloop < smo->VF_max_vel ) {
            Target_Vel_openloop = Target_Vel_openloop + smo->VF_acc; //0.3 is 加速度
    } else {
            Target_Vel_openloop = smo->VF_max_vel;
    }
    shaft_angle = _normalizeAngle (shaft_angle + dir * Target_Vel_openloop * Ts );
    
    if (num == 1) {
        mc_set_torque (2.0f + Target_Vel_openloop * smo->VF_uq_delta, 0, shaft_angle);
    }
    open_loop_timestamp = now_us;
}
}