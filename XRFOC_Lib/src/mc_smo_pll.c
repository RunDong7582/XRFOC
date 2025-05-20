/**
 *  @file:      XRFOC_Lib/src/mc_smo_pll.c
 *  @brief:     Sliding Mode Observer & Phase Locked Loop algorithm
 *  @author:    RunDong7582
 *  @date  :    2025.5.19 18:49
 *  @version:   XRFOC release version
 */

#include "../XRFOC_Lib/include/mc_smo_pll.h"
#include "../XRFOC_Lib/include/mc_foc_core.h"

 extern struct Cur_vol_s Cur_vol;
 extern Motor_Console motor_console;
 extern XRFOC_Console hfoc;

void smo_closeloop ( SMO_PLL_Observer *smo ) {
    

    smo->now_time = HAL_GetTick();
    smo->Ts = (smo->now_time - smo->last_time) * 1e-3f;
    smo->last_time = smo->now_time;
    if (smo->Ts < 0 || smo->Ts > 5e-3f) smo->Ts = 1e-3f;

    smo->Ialpha = Cur_vol.I_alpha;
    smo->Ibeta  = Cur_vol.I_beta;

    smo->Ualpha = -sin(smo->Est_Theta) * Cur_vol.Uq;
    smo->Ubeta = cos(smo->Est_Theta)   * Cur_vol.Uq;

    smo_position_estimate(smo);
}

float smo_sat_plane ( float err, float limits ) {
    if (err > limits ) return 1;
    else if (err < -limits) return -1;
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

void smo_pll_calc ( SMO_PLL_Observer *smo, float alpha, float beta ) {

    smo->Theta_err = -1 * alpha * cos(smo->Est_Theta) + beta * sin(smo->Est_Theta) * -1;
    /* pi controller to fetch e_angle */
    /* position */
    smo->i_err += smo->Ts * smo->PLL_ki * smo->Theta_err;
    smo->Est_speed = smo->PLL_kp * smo->Theta_err + smo->i_err;

    /* speed filter */
    smo->Est_speed_F = smo->Est_speed_F * 0.9f + smo->Est_speed * 0.1f;

    /* intergral w(omega) to fetch e_angle */
    smo->Est_Theta += smo->Ts * smo->Est_speed; 

    /* limit the Amplitude of e_angle */
    smo->Est_Theta = _normalizeAngle (smo->Est_Theta);

}

float Target_Vel_openloop = 0;
uint32_t open_loop_timestamp = 0;
uint32_t now_us = 0;
uint32_t init_time = 0;
float shaft_angle;

void smo_VF_start ( SMO_PLL_Observer *smo , uint8_t num, uint8_t dir ) {

	if (dir == CCW) 
	{
		smo->_dir = 1;
	} 
	else if (dir == CW) 
	{
		smo->_dir = -1;
	} 
	else 
	{
		smo->_dir = 0;
	}
	
    if ( smo->VF_flag == 0 ) {
	//  init_time = micros();
		init_time = HAL_GetTick();
	//  now_us = micros();
		now_us = HAL_GetTick();
        smo->VF_flag = 2;
    }

    if ( (now_us - init_time) >= 3000 ) {
        smo->VF_flag = 1;
    }

    if ( smo->VF_flag == 2) {
        // now_us = micros();
        now_us = HAL_GetTick();
        float Ts = (now_us - open_loop_timestamp) * 1e-3f; //ms to s，若为us，*1e-6f
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
        
        hfoc.eAngle = shaft_angle;              //给定电角度
        hfoc.openloopVel = Target_Vel_openloop; //给定自增电角速度

        if (num == 1 && hfoc.schedule == _V_F_STAGE || hfoc.schedule == _ALIGN_STAGE ) {
            mc_set_torque (hfoc.Uq_boost + Target_Vel_openloop * smo->VF_uq_delta, 0, shaft_angle);
        }

        open_loop_timestamp = now_us;
		delay_us(1000);
    }
}
