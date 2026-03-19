/**
 *  @file:      XRFOC_Lib/src/mc_smo_pll.c
 *  @brief:     Sliding Mode Observer & Phase Locked Loop algorithm
 *  @author:    RunDong7582
 *  @date  :    2026 3/19 18:00
 *  @version:   XRFOC v1.0
 */

 #include "../XRFOC_Lib/include/mc_smo_pll.h"
 #include "../XRFOC_Lib/include/mc_foc_core.h"
 #include "../XRFOC_Lib/include/mc_time.h"

 extern struct Cur_vol_s Cur_vol;
 extern Motor_Console motor_console;
 extern XRFOC_Console hfoc;
 extern PIDController current_q_loop;

static inline float _constrain_f32(float value, float min_v, float max_v)
{
    return (value < min_v) ? min_v : ((value > max_v) ? max_v : value);
}

static inline float xrfoc_startup_dir_sign(uint8_t dir)
{
    return (dir == CCW) ? 1.0f : -1.0f;
}

static inline void xrfoc_startup_set_dir(SMO_PLL_Observer *smo, uint8_t dir)
{
    if (dir == CCW) {
        smo->_dir = 1;
    } else if (dir == CW) {
        smo->_dir = -1;
    } else {
        smo->_dir = 0;
    }
}

static inline void xrfoc_observer_gain_schedule(SMO_PLL_Observer *smo)
{
    if (smo->observer_gain_schedule_en == 0U) {
        return;
    }

    float w_abs = fabsf(smo->Est_speed_F);
    float w1 = (smo->sched_w1 > 0.0f) ? smo->sched_w1 : 80.0f;
    float w2 = (smo->sched_w2 > w1) ? smo->sched_w2 : (w1 + 120.0f);

    if (w_abs < w1) {
        if (smo->h_low > 0.0f) smo->h = smo->h_low;
        if (smo->pll_kp_low > 0.0f) smo->PLL_kp = smo->pll_kp_low;
        if (smo->pll_ki_low > 0.0f) smo->PLL_ki = smo->pll_ki_low;
    } else if (w_abs < w2) {
        if (smo->h_mid > 0.0f) smo->h = smo->h_mid;
        if (smo->pll_kp_mid > 0.0f) smo->PLL_kp = smo->pll_kp_mid;
        if (smo->pll_ki_mid > 0.0f) smo->PLL_ki = smo->pll_ki_mid;
    } else {
        if (smo->h_high > 0.0f) smo->h = smo->h_high;
        if (smo->pll_kp_high > 0.0f) smo->PLL_kp = smo->pll_kp_high;
        if (smo->pll_ki_high > 0.0f) smo->PLL_ki = smo->pll_ki_high;
    }
}
/**********************************************************
 **
* 滑模闭环
*/
void smo_closeloop ( SMO_PLL_Observer *smo ) {
    
//   smo->now_time = micros();
    smo->now_time = mc_time_us();
    smo->Ts = (smo->now_time - smo->last_time) * 1e-6f;
    smo->last_time = smo->now_time;
    if (smo->Ts <= 0.0f || smo->Ts > 5e-3f) smo->Ts = 1e-4f;

    //更新变量
    smo->Ialpha = Cur_vol.I_alpha;
    smo->Ibeta  = Cur_vol.I_beta;

    smo->Ualpha = -sin(smo->Est_Theta) * Cur_vol.Uq;
    smo->Ubeta = cos(smo->Est_Theta)   * Cur_vol.Uq;

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

    smo->Ealpha_flt = 0.1f * smo->Ealpha_flt + 0.9f * smo->Ealpha;
    smo->Ebeta_flt  = 0.1f * smo->Ebeta_flt  + 0.9f * smo->Ebeta;

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
    smo->Est_speed_F = smo->Est_speed_F * 0.9f + smo->Est_speed * 0.1f;

    /* observer gains schedule by speed */
    xrfoc_observer_gain_schedule(smo);

    /* intergral w(omega) to fetch e_angle */
    smo->Est_Theta += smo->Ts * smo->Est_speed; 

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

void smo_VF_start ( SMO_PLL_Observer *smo , uint8_t num, uint8_t dir ) {
    xrfoc_startup_set_dir(smo, dir);
	
    if ( smo->VF_flag == 0 ) {
        init_time = mc_time_us();
        now_us = mc_time_us();
        open_loop_timestamp = now_us;
        Target_Vel_openloop = 0.0f;
        shaft_angle = hfoc.eAngle;
        smo->VF_flag = 2;
    }

    if ( smo->VF_flag == 2) {

		now_us = mc_time_us();
        float Ts = (now_us - open_loop_timestamp) * 1e-6f; 
        if (Ts <= 0 || Ts > 0.01f)
        Ts = 1e-4f;

        if ( Target_Vel_openloop < smo->VF_max_vel ) {
                Target_Vel_openloop = Target_Vel_openloop + smo->VF_acc; //0.3 is 加速度
        } else {
                Target_Vel_openloop = smo->VF_max_vel;
        }
        shaft_angle = _normalizeAngle (shaft_angle + xrfoc_startup_dir_sign(dir) * Target_Vel_openloop * Ts );
        
        hfoc.eAngle = shaft_angle;              
        hfoc.openloopVel = Target_Vel_openloop; 

        if (num == 1 && (hfoc.schedule == _V_F_STAGE || hfoc.schedule == _ALIGN_STAGE)) {
            mc_set_torque (hfoc.Uq_boost + Target_Vel_openloop * smo->VF_uq_delta, 0, shaft_angle);
        }

        open_loop_timestamp = now_us;
    }
}

void smo_IF_start ( SMO_PLL_Observer *smo , uint8_t num, uint8_t dir )
{
    xrfoc_startup_set_dir(smo, dir);

    if (smo->VF_flag == 0) {
        init_time = mc_time_us();
        now_us = mc_time_us();
        open_loop_timestamp = now_us;
        Target_Vel_openloop = 0.0f;
        shaft_angle = hfoc.eAngle;
        smo->VF_flag = 2;
    }

    if (smo->VF_flag == 2) {
        now_us = mc_time_us();
        float Ts = (now_us - open_loop_timestamp) * 1e-6f;
        if (Ts <= 0.0f || Ts > 0.01f) {
            Ts = 1e-4f;
        }

        if (Target_Vel_openloop < smo->VF_max_vel) {
            Target_Vel_openloop += smo->VF_acc;
        } else {
            Target_Vel_openloop = smo->VF_max_vel;
        }

        shaft_angle = _normalizeAngle(shaft_angle + xrfoc_startup_dir_sign(dir) * Target_Vel_openloop * Ts);
        hfoc.eAngle = shaft_angle;
        hfoc.openloopVel = Target_Vel_openloop;

        if (num == 1 && (hfoc.schedule == _V_F_STAGE || hfoc.schedule == _ALIGN_STAGE)) {
            float iq_ref = fabsf(smo->IF_current_ref) * xrfoc_startup_dir_sign(dir);
            float iq_meas = xrfoc_current_q_lpf();
            float uq_cmd = pid_operator(&current_q_loop, iq_ref - iq_meas);

            float uq_limit_hi = (smo->IF_uq_max > 0.0f) ? smo->IF_uq_max : current_q_loop.limit;
            float uq_limit_lo = (smo->IF_uq_min > 0.0f) ? smo->IF_uq_min : 0.8f;
            uq_limit_hi = _constrain_f32(uq_limit_hi, uq_limit_lo, current_q_loop.limit);

            uq_cmd = _constrain_f32(uq_cmd, -uq_limit_hi, uq_limit_hi);
            if (fabsf(uq_cmd) < uq_limit_lo) {
                uq_cmd = (uq_cmd >= 0.0f) ? uq_limit_lo : -uq_limit_lo;
            }

            mc_set_torque(uq_cmd, 0.0f, shaft_angle);
        }

        open_loop_timestamp = now_us;
    }
}
