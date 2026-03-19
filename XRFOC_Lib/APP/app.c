/**
 *  @file:      XRFOC_Lib_on_stm32/APP/app.c
 *  @brief:     XRFOC Application Layer.
 *  @author:    RunDong7582
 *  @date  :    2026 3/19 18:00
 *  @version:   XRFOC v1.0
 */

#include "app.h"

extern float Target_flt;
extern SMO_PLL_Observer SMO;
extern LowPassFilter CurrentQ_Flt;
extern LowPassFilter CurrentD_Flt;
extern PIDController speed_loop;
extern PIDController position_loop;
extern PIDController current_q_loop;
extern PIDController current_d_loop;
extern struct Cur_vol_s Cur_vol;
extern Motor_Console motor_console;
extern Motor_Console_Data motor_I_UVW;

extern XRFOC_Console hfoc;

extern uint16_t adc_amp_offset[3][ADC_AMP_OFFSET_TIMES+1];
extern int16_t adc_amp[3];
extern float  adc_amp_bus;

extern ADC_HandleTypeDef hadc1;

#define ALIGN_OK_COUNT_TARGET      (200)
#define SPEED_MATCH_THRESHOLD_RAD  (12.0f)

void APP_StateMachine_Handler ( XRFOC_Console *self )
{
    static FOC_STAGE last_stage = _V_F_STAGE;
    static uint16_t align_guard_cnt = 0;
    int bAngleClosedLoop = 0;               /* 表示可以切换至闭环标志位 */         

    if (self->schedule != last_stage)
    {
        align_guard_cnt = 0;
        last_stage = self->schedule;
    }

    switch (self->schedule)                 /* 状态机调度 */     
    {
        case _V_F_STAGE:                    /* 注意switch case 写法与平常不同，参考lks08 demo */
        {
            xrfoc_vf_start(motor_console.num, motor_console.dir);
            if ( SMO.VF_flag == 2 )  {
                    self->schedule = _ALIGN_STAGE;
            }
            break; 
        }
        case _ALIGN_STAGE:
        {
            float mech_speed_est = SMO.Est_speed / motor_console.pp;
            float speed_error = ABS(self->openloopVel - mech_speed_est);
            uint16_t align_timeout_cnt = (SMO.align_timeout_count > 0U) ? SMO.align_timeout_count : 1200U;
            bAngleClosedLoop = xrfoc_vf_start(motor_console.num, motor_console.dir);
            if ( bAngleClosedLoop == 0 && SMO.VF_flag == 2 ) {
                align_guard_cnt++;
                if ( (self->openloopVel >= (SMO.VF_max_vel - 1.0f)) && ( self->cntangle < ALIGN_OK_COUNT_TARGET) )
                {
                    self->angle_error = ABS( SMO.Est_Theta - self->eAngle ); /* 计算电角度误差 */
                    if ( self->angle_error > 1.0f ) 
                    {
                        self->angle_error = 2*_PI - self->angle_error; /* 归一化 */
                    }
                    if( (self->angle_error < (self->angle_thresh + err_rad)) && (speed_error < SPEED_MATCH_THRESHOLD_RAD) ) 
                                    self->cntangle++;
                    else if (self->cntangle > 0)
                                    self->cntangle--;
                }
                else 
                {
                    bAngleClosedLoop = 1;
                    self->cntangle = 0;
                }

                if (align_guard_cnt > align_timeout_cnt)
                {
                    if ((SMO.startup_mode == XRFOC_STARTUP_IF) && (SMO.startup_fallback_en != 0U))
                    {
                        SMO.startup_mode = (SMO.fallback_mode == XRFOC_STARTUP_IF) ? XRFOC_STARTUP_VF : SMO.fallback_mode;
                        SMO.VF_flag = 0;
                        self->cntangle = 0;
                        self->schedule = _V_F_STAGE;
                    }
                    else
                    {
                        bAngleClosedLoop = 1;
                        self->cntangle = 0;
                    }
                    align_guard_cnt = 0;
                }
            } 
            else if ( bAngleClosedLoop == 1 && SMO.VF_flag == 2 ) {
                self->schedule = _CLOSE_LOOP_STAGE;
                align_guard_cnt = 0;
            }
            break;
        }
        case _CLOSE_LOOP_STAGE:
            // xrfoc_smo_voltage_set_torque(motor_I_UVW.Target_set,motor_I_UVW.TYPE);          //电压力矩（无电流闭环）
            // xrfoc_smo_current_set_torque(motor_I_UVW.Target_set,motor_I_UVW.TYPE);       //电流力矩
            xrfoc_smo_current_set_speed(motor_I_UVW.Target_set,motor_I_UVW.TYPE);        //速度电流双闭环
        break;
    }
}

void xrfoc_current_offset_sample (void)
{
	uint8_t i=0;
	uint32_t avg[3] = {0,0,0};
			
	for(i=0; i<ADC_AMP_OFFSET_TIMES; i++)               /* 将采集的每个通道值累加 */
	{
		adc_amp_offset[0][i] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);//g_adc_val[2];         /* 得到还未开始运动时三相的基准电压 */
		adc_amp_offset[1][i] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);//g_adc_val[3];         
		adc_amp_offset[2][i] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);//g_adc_val[4];    
		
		avg[0] += adc_amp_offset[0][i];
		avg[1] += adc_amp_offset[1][i];
		avg[2] += adc_amp_offset[2][i];
	}

	for(uint8_t i=0; i<3; i++)
	{
		avg[i] /= ADC_AMP_OFFSET_TIMES;                         /* 取平均即软件滤波 */
		adc_amp_offset[i][ADC_AMP_OFFSET_TIMES] = avg[i];       /* 得到还未开始运动时的基准电压 */
	}

}

void xrfoc_current_update (void)
{
	for(uint8_t i=0; i<3; i++)
	{
		adc_val_m1[i] = g_adc_val[i+2];
		adc_amp[i] = adc_val_m1[i] - adc_amp_offset[i][ADC_AMP_OFFSET_TIMES];
        adc_amp_un[i] = adc_amp[i];
	}
	/*运算母线电流（母线电流为任意两个有开关动作的相电流之和）*/
	adc_amp_bus = (adc_amp_un[0] + adc_amp_un[1] + adc_amp_un[2])*ADC2CURT;

    motor_I_UVW.current_u = adc_amp_un[0]* ADC2CURT;    /*U*/
    motor_I_UVW.current_v = adc_amp_un[1]* ADC2CURT;    /*V*/
	motor_I_UVW.current_w = adc_amp_un[2]* ADC2CURT;    /*W*/
}

/**
 * @brief       运行观测器计算
 * @param       无
 * @retval      无
 */
void xrfoc_run ( void ) {

    /* LED0(红灯) 翻转 表示运行一次FOC&观测器解算 */
	// LED0_TOGGLE();
	
    //	xrfoc_current_offset_sample();
    /* current_u/v/w由ADC注入中断回调统一更新 */
    
    motor_I_UVW.cntt++;
    // hfoc.eAngle = (hfoc.schedule < _CLOSE_LOOP_STAGE) ? hfoc.eAngle:SMO.Est_Theta; 
    // Cur_vol = cal_Iq_Id(motor_I_UVW.current_u, motor_I_UVW.current_v, hfoc.eAngle);

    float park_angle = (hfoc.schedule < _CLOSE_LOOP_STAGE) ? hfoc.eAngle : SMO.Est_Theta;
    Cur_vol = cal_Iq_Id(motor_I_UVW.current_u, motor_I_UVW.current_v, park_angle);
    smo_closeloop(&SMO);
    
    if (motor_I_UVW.cntt > 42) {
        // print Estimated SMO electrical angle
        motor_I_UVW.cntt = 0;
    }
}

void xrfoc_target_torque_set ( Control_Obj type )
{
    switch (type) {
        case VOLTAGE_Torque:
            motor_I_UVW.Target_set = 8.0f;          /* advice : 8V */
            break;
        case CURRENT_Torque:
            motor_I_UVW.Target_set = 2.0f;          /* advice : 1A or 2A */
            break;
        case SPEED_Torque:
            motor_I_UVW.Target_set = 90.0f;          /* advice : 90 rad/s */
            break;
        default:
            motor_I_UVW.Target_set = 0.0f;
            printf("please input the torque type!\r\n");
            break;
    }
}

float voltage[3];
float current[3];
float current_lpf[3];
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if(hadc == &hadc1)
	{
        motor_console.svpwm_flag = 1;
		
		adc_val_m1[0] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
		adc_val_m1[1] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
		adc_val_m1[2] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
		
		for(uint8_t i=0; i<3; i++)
		{           
			adc_amp[i] = adc_val_m1[i] - adc_amp_offset[i][ADC_AMP_OFFSET_TIMES];
		}

		adc_amp_bus = (adc_amp[0] + adc_amp[1] + adc_amp[2])*ADC2CURT;

		current[0] = adc_amp[0]* ADC2CURT;
		current[1] = adc_amp[1]* ADC2CURT;
		current[2] = adc_amp[2]* ADC2CURT;
		
		FirstOrderRC_LPF(current_lpf[0],current[0],0.1f);
		FirstOrderRC_LPF(current_lpf[1],current[1],0.1f);
		FirstOrderRC_LPF(current_lpf[2],current[2],0.1f);
		
		motor_I_UVW.current_u = current_lpf[0];
		motor_I_UVW.current_v = current_lpf[1];
		motor_I_UVW.current_w = current_lpf[2];

        if (motor_console.run_flag == RUN)
        {
            xrfoc_run();
            APP_StateMachine_Handler(&hfoc);
        }

    }
}