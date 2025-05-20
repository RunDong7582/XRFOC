#ifndef __XRFOC_APP_H
#define __XRFOC_APP_H

/**
 *  @file:      XRFOC_Lib/APP/app.h
 *  @brief:     XRFOC Application Layer.
 *  @author:    RunDong7582
 *  @date  :    2025.5.19 14:01
 *  @version:   XRFOC release version
 */

#include "../XRFOC_Lib/include/mc_foc_core.h"
#include "../XRFOC_Lib/include/mc_adaptor.h"
#include "../XRFOC_Lib/mcu_conf/stm32/mc_stm32_conf.h"
#include "../XRFOC_Lib/common/mc_common.h"

void step_up(void);
void step_down(void);

void xrfoc_run (void);
void xrfoc_target_torque_set ( Control_Obj type );

void xrfoc_current_offset_sample (void);
void xrfoc_current_update (void);

void APP_StateMachine_Handler ( XRFOC_Console *self );

#endif

