#ifndef __MC_SMO_PLL_H
#define __MC_SMO_PLL_H

/**
 *  @file:      XRFOC_Lib/include/mc_smo_pll.h
 *  @brief:     Observer algorithm for sensorless FOC.
 *  @author:    RunDong7582
 *  @date  :    2026 3/19 18:00
 *  @version:   XRFOC v1.0
 */
#include "mc_foc_core.h"
#include "mc_common.h"

#define XRFOC_STARTUP_VF    (0U)
#define XRFOC_STARTUP_IF    (1U)

struct SMO_params {

    float Rs;
    float Ls;
    float h;
    float PLL_kp;
    float PLL_ki;
    
    float VF_acc;
    float VF_max_vel;
    float VF_uq_delta;

    float IF_current_ref;
    float IF_uq_min;
    float IF_uq_max;
    uint8_t startup_mode;
    uint8_t startup_fallback_en;
    uint8_t fallback_mode;
    uint16_t align_timeout_count;

    uint8_t observer_gain_schedule_en;
    float sched_w1;
    float sched_w2;
    float h_low;
    float h_mid;
    float h_high;
    float pll_kp_low;
    float pll_kp_mid;
    float pll_kp_high;
    float pll_ki_low;
    float pll_ki_mid;
    float pll_ki_high;
};

typedef struct {

    float Rs;
    float Ls;
    //Uα Uβ
    float Ualpha;
    float Ubeta;
  
    //α和β的实际电流
    float Ialpha;
    float Ibeta;
  
    //α和β的估算电流
    float Est_Ialpha;
    float Est_Ibeta;
  
    //α和β的电流误差
    float Ialpha_Err;
    float Ibeta_Err;
  
    //滑模增益
    float h;
  
    //估算的反电动势
    float Ealpha;
    float Ebeta;
  
    //低通滤波后的估算的反电动势
    float Ealpha_flt;
    float Ebeta_flt;
  
    //传入锁相环的角度误差
    float Theta_err;
  
    //锁相环估算的角度
    float Est_Theta;
    float Theta_Err_last;
  
    //锁相环估算的速度
    float Est_speed;
    float Est_speed_F;
  
    //锁相环的PI值
    float PLL_kp;
    float PLL_ki;
  
    float i_err;
    float last_i_err;
  
    //方向
    int _dir;
    //时间变量
    uint32_t now_time;
    uint32_t last_time;
    float Ts;
  
    uint8_t VF_flag;
    float VF_acc;
    float VF_max_vel;
    float VF_uq_delta;

    float IF_current_ref;
    float IF_uq_min;
    float IF_uq_max;
    uint8_t startup_mode;
    uint8_t startup_fallback_en;
    uint8_t fallback_mode;
    uint16_t align_timeout_count;

    uint8_t observer_gain_schedule_en;
    float sched_w1;
    float sched_w2;
    float h_low;
    float h_mid;
    float h_high;
    float pll_kp_low;
    float pll_kp_mid;
    float pll_kp_high;
    float pll_ki_low;
    float pll_ki_mid;
    float pll_ki_high;
  
} SMO_PLL_Observer;
  
void  smo_closeloop 				( SMO_PLL_Observer *smo );
void  smo_position_estimate         ( SMO_PLL_Observer *smo );
float smo_sat_plane  				( float err, float limits );
void  smo_pll_calc   				( SMO_PLL_Observer *smo, float alpha, float beta );
//void  smo_VF_start   				( SMO_PLL_Observer *smo , int num, int dir ); 
void smo_VF_start 					( SMO_PLL_Observer *smo , uint8_t num, uint8_t dir );
void smo_IF_start 					( SMO_PLL_Observer *smo , uint8_t num, uint8_t dir );
#endif 
