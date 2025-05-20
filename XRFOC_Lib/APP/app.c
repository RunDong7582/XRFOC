/**
 *  @file:      XRFOC_Lib/APP/app.c
 *  @brief:     XRFOC Application Layer.
 *  @author:    RunDong7582
 *  @date  :    2025.5.19 14:01 
 *  @version:   XRFOC release version 
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

void APP_StateMachine_Handler ( XRFOC_Console *self )
{
    int bAngleClosedLoop = 0;               /* 表示可以切换至闭环标志位 */         
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
            bAngleClosedLoop = xrfoc_vf_start(motor_console.num, motor_console.dir);
            if ( bAngleClosedLoop == 0 && SMO.VF_flag == 2 ) {
                if ( (self->openloopVel == SMO.VF_max_vel) && ( self->cntangle < 200) ) /* 这个计数值上限100，需要测试 */
                {
                    self->angle_error = ABS( SMO.Est_Theta - self->eAngle ); /* 计算电角度误差 */
                    if ( self->angle_error > 1.0f ) 
                    {
                        self->angle_error = 2*_PI - self->angle_error; /* 归一化 */
                    }
                    if( self->angle_error < self->angle_thresh + err_rad ) 
                                    self->cntangle++;
                }
                else 
                {
                    bAngleClosedLoop = 1;
                    self->cntangle = 0;
                }
            } 
            else if ( bAngleClosedLoop == 1 && SMO.VF_flag == 2 ) {
                self->schedule = _CLOSE_LOOP_STAGE;
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
//	for(uint8_t i=0; i<3; i++)
//	{
//		adc_val_m1[i] = g_adc_val[i+2];
//		adc_amp[i] = adc_val_m1[i] - adc_amp_offset[i][ADC_AMP_OFFSET_TIMES];
//		if(adc_amp[i] < 0)  
//			adc_amp_un[i] = 0;                  /* 反电动势电压为悬空绕组直接清0 */
//		else if(adc_amp[i] >= 0)                /* 去除反电动势引起的负电流数据 */
//			adc_amp_un[i] = adc_amp[i];
//	}
//	/*运算母线电流（母线电流为任意两个有开关动作的相电流之和）*/
//	adc_amp_bus = (adc_amp_un[0] + adc_amp_un[1] + adc_amp_un[2])*ADC2CURT;

//    motor_I_UVW.current_u = adc_amp_un[0]* ADC2CURT;    /*U*/
//    motor_I_UVW.current_v = adc_amp_un[1]* ADC2CURT;    /*V*/
//	motor_I_UVW.current_w = adc_amp_un[2]* ADC2CURT;    /*W*/
}


/**
 * @brief       设置目标值++
 * @param       无
 * @retval      无
 */
void step_up(void)
{
    motor_console.run_flag = RUN;                       /* 开启运行 */
    start_motor1();                                     /* 开启运行 */
    
    if ( motor_console.dir == CCW && motor_I_UVW.Target_set == 0.0f )
    {
         motor_console.dir = CW;
    }
    float speed_rpm_temp = rad_s_to_rpm(motor_I_UVW.Target_set);
    speed_rpm_temp += 400;

    if (motor_I_UVW.Target_set == 0)
    {
        motor_console.run_flag = STOP;
        stop_motor1();                                  /* 停机 */
    }
    if (speed_rpm_temp >= 3200) {
        /* 最高不超过3200PRM */
        speed_rpm_temp = 3200;
        motor_I_UVW.Target_set = rpm_to_rad_s(speed_rpm_temp);
    } else {
        motor_I_UVW.Target_set += rpm_to_rad_s(400);
    }     
}

/**
 * @brief       设置目标值--
 * @param       无
 * @retval      无
 */
void step_down(void)
{
    motor_console.run_flag = RUN;                       /* 开启运行 */
    start_motor1();                                     /* 开启运行 */
    
    if ( motor_console.dir == CW && motor_I_UVW.Target_set == 0.0f )
    {
         motor_console.dir = CCW;
    }
    float speed_rpm_temp = rad_s_to_rpm(motor_I_UVW.Target_set);
    speed_rpm_temp -= 400;

    if (motor_I_UVW.Target_set == 0)
    {
        motor_console.run_flag = STOP;
        stop_motor1();                                  /* 停机 */
    }
    if (speed_rpm_temp <= 45) {
        /* 最高不超过3200PRM */
        speed_rpm_temp = 45;
        motor_I_UVW.Target_set = 5; //最低5rad/s
    } else {
        motor_I_UVW.Target_set -= rpm_to_rad_s(400);
    }     
}

int  BSP_XRFOC_Init() //( struct mc_adaptor_stm32_hw *stm32f407igtx )
{
#if 0	
    key_init();
    led_init();

    if (mc_adaptor_stm32_hw_init(stm32f407igtx) != 0) {
        return 1;
    }

    /* 设置时钟,168Mhz */
    if (mc_clk_init(stm32f407igtx, 336, 8, 2, 7 ) != 0) {
        return 2;
    }

    delay_init(168);  

    /* 串口初始化为115200 */
    if (mc_uart_init(stm32f407igtx, 115200) != 0) {
        return 3;
    }

    /* 电机使能引脚1、2 */
    mc_power_init(stm32f407igtx);

    /* 18khz pwm */
    if (mc_pwm_init(stm32f407igtx, 168000/18-1, 0) != 0) {
        return 4;
    }

    // /* timer init */
    // if (mc_timer_init(stm32f407igtx, 1000-1, 84-1) != 0) {
    //     return 5;
    // }

    start_motor1();
#endif	
    return 0;

}

void BSP_init_handle ( uint8_t res )
{
    switch(res)
    {
        case 0:
            printf("XRFOC Periphal Init Success\r\n");
        break;
        case 1:
            LED0_TOGGLE();  /* LED0(红灯) 翻转 表示抽象层初始化失败 */
        break;
        case 2:
            LED0_TOGGLE();  /* LED0(红灯) 翻转 表示时钟树初始化失败 */
        break;
        case 3:
            LED1_TOGGLE();  /* LED1 翻转 表示串口初始化失败 */
        break;
        case 4:
            printf("Periphal 18kHz PWM Init Failed\r\n");
        break;
        default:
            printf("Unknow Init Failed\r\n");
        break;
    }
}

/**
 * @brief       运行观测器计算
 * @param       无
 * @retval      无
 */
void xrfoc_run ( void ) {

    /* LED0(红灯) 翻转 表示运行一次FOC&观测器解算 */
    LED0_TOGGLE(); 
	
//	xrfoc_current_offset_sample();
    /* Update the current */
//    xrfoc_current_update();   
    
    motor_I_UVW.cntt++;
    // hfoc.eAngle = (hfoc.schedule < _CLOSE_LOOP_STAGE) ? hfoc.eAngle:SMO.Est_Theta; 
    // Cur_vol = cal_Iq_Id(motor_I_UVW.current_u, motor_I_UVW.current_v, hfoc.eAngle);

    Cur_vol = cal_Iq_Id(motor_I_UVW.current_u, motor_I_UVW.current_v, SMO.Est_Theta);
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

