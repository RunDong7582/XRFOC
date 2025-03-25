#ifndef _FOCDATABASE_H
#define _FOCDATABASE_H
#include "FocConfig.h"
#define SQRT3                  (float)1.73205080757f
#define SQRT3_2                (float)(SQRT3 * 0.5f)
#define SPEED_LOOP_TIME        (float)(0.001f)
#define F_BASE                 (float)(1.0f / SPEED_LOOP_TIME)
#define U_BASE                 (float)(24.0)
#define CURRENT_BASE           (float)(10.0f)
#define R_BASE                 (float)(U_BASE / CURRENT_BASE)
#define OMEGA_BASE             (float)(M_2PI / SPEED_LOOP_TIME)
#define OMEGA_ACC_BASE         (float)(OMEGA_BASE / SPEED_LOOP_TIME)
#define L_BASE                 (float)(U_BASE / (OMEGA_BASE * CURRENT_BASE))
#define T_BASE                 (float)(M_2PI * L_BASE / R_BASE)
#define THETA_BASE             (float)(M_2PI)
#define SPEED_DIV_FOC_RATE     (float)(SPEED_LOOP_TIME / FOC_PWM_PERIOD)

#endif
