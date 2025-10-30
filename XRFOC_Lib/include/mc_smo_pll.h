#ifndef __MC_SMO_PLL_H
#define __MC_SMO_PLL_H

/**
 *  @file:      XRFOC_Lib/include/mc_smo_pll.h
 *  @brief:     Observer algorithm for sensorless FOC.
 *  @author:    RunDong7582
 *  @date  :    2025.5.19 18:33
 *  @version:   XRFOC release version 
 */
#include "mc_foc_core.h"
#include "mc_common.h"

struct SMO_params {

    float Rs;
    float Ls;
    float h;
    float PLL_kp;
    float PLL_ki;
    
    float VF_acc;
    float VF_max_vel;
    float VF_uq_delta;
};

typedef struct {

    float Rs;
    float Ls;
    float Ualpha;
    float Ubeta;
  
    float Ialpha;
    float Ibeta;
  
    float Est_Ialpha;
    float Est_Ibeta;
  
    float Ialpha_Err;
    float Ibeta_Err;
  
    float h;
    /* 滑模增益要足够大以使式中观测的电流收敛到实际电流，
    但这个值太大会导致观测的电流剧烈抖动，增加估计误差 */
  
    float Ealpha;
    float Ebeta;
  
    float Ealpha_flt;
    float Ebeta_flt;
  
    float Theta_err;
  
    float Est_Theta;
    float Theta_Err_last;
  
    float Est_speed;
    float Est_speed_F;
  
    float PLL_kp;
    float PLL_ki;
  
    float i_err;
    float last_i_err;
  
    int _dir;
    uint32_t now_time;
    uint32_t last_time;
    float Ts;
  
    uint8_t VF_flag;
    float VF_acc;
    float VF_max_vel;
    float VF_uq_delta;
  
} SMO_PLL_Observer;
  
void  smo_closeloop 				( SMO_PLL_Observer *smo );
void  smo_position_estimate         ( SMO_PLL_Observer *smo );
float smo_sat_plane  				( float err, float limits );
void  smo_pll_calc   				( SMO_PLL_Observer *smo, float alpha, float beta );
void  smo_VF_start 					( SMO_PLL_Observer *smo , uint8_t num, uint8_t dir );
#endif 
