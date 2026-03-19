/**
 *  @file:      main.c
 *  @brief:     Motor run task. Please use UTF-8 encoding！！！
 *  @author:    RunDong7582
 *  @date  :    2026 3/14 18:00
 *  @version:   XRFOC v1.0
 */
 
#include "../XRFOC_Lib/common/mc_common.h"
#include "../XRFOC_Lib/include/mc_adaptor.h"
#include "../XRFOC_Lib/include/mc_foc_core.h"
#include "../XRFOC_Lib/include/mc_time.h"
#include "../XRFOC_Lib/mcu_conf/stm32/mc_stm32_conf.h"
#include "../XRFOC_Lib/APP/app.h"

 /* 存放了旋转坐标系dq轴的分量，以及观测器观测电流αβ轴*/
 struct Cur_vol_s Cur_vol;

 /* 抽象层结构体 */
 struct mc_adaptor_stm32_hw stm32_adaptor;   

 /* 电机控制台 */
 Motor_Console motor_console = {
    .num = 1,                       /* 按键状态对应原来的 uint8_t key */
    .loop = 0,                      /* loop次数对应原来的 uint8_t t  */      
    .power = 0.0f,                  /* 就是电机驱动供电电压 */
    .pp = 2,                        /* polar pairs极对数的简称pp */
    .dir = CW,                      /* CW 为Clock Wise 顺时针 CCW 为Counter ~ 逆时针*/
    .svpwm_flag = 0,                /* 处理初始化情况 */  
    .run_flag = STOP,               /* 对应原bldc_obj中的电机运转标志 */ 
    .lock_flag = 0,                 /* 对应原bldc_obj中的电机堵转标志 */ 
 };

 /* 相电流&设定值数据 */
 Motor_Console_Data motor_I_UVW = {
    .current_u = 0.0f,
    .current_v = 0.0f,	 
    .current_w = 0.0f,	 
    .Target_flt = 0.0f,
    .Target_set = 0.0f,
    .TYPE = SPEED_Torque,
    .cntt = 0,
 };
 
 /* 滑模观测器和锁相环滤波结构体 */
 SMO_PLL_Observer SMO = {               /* 需要替换为油泵电机的实际参数 */
     .Rs = 0.4f,                        /* rs 为相电阻, 0.4 ohm */
     .Ls = 0.0008f,                     /* ls 为相电感, 0.8 mH */ 
     .h = 0.3f,                         /* 滑模增益h */
     .PLL_kp = 521.0f,                  /* 锁相环比例增益 */
     .PLL_ki = 40000.0f,                /* 锁相环积分增益 */
     .VF_acc = 0.1f,                    /* VF启动的加速度 */
     .VF_max_vel = 60,                 /* VF启动的最大速度（电角速度）2050rpm */
     .VF_uq_delta = 0.006f,             /* VF启动的uq增量 */
     .IF_current_ref = 1.5f,            /* I/F启动电流参考(A) */
     .IF_uq_min = 1.2f,                 /* I/F启动最小电压幅值(V) */
     .IF_uq_max = 6.0f,                 /* I/F启动最大电压幅值(V) */
     .startup_mode = XRFOC_STARTUP_IF,  /* 默认使用I/F启动 */
     .startup_fallback_en = 1U,         /* I/F失败自动回退 */
     .fallback_mode = XRFOC_STARTUP_VF, /* 回退模式 */
     .align_timeout_count = 1200U,      /* 对齐超时计数（与控制频率相关） */

     .observer_gain_schedule_en = 1U,   /* 观测器分段增益 */
     .sched_w1 = 120.0f,                /* 电角速度分段阈值1 */
     .sched_w2 = 260.0f,                /* 电角速度分段阈值2 */
     .h_low = 0.45f,
     .h_mid = 0.32f,
     .h_high = 0.22f,
     .pll_kp_low = 580.0f,
     .pll_kp_mid = 520.0f,
     .pll_kp_high = 430.0f,
     .pll_ki_low = 46000.0f,
     .pll_ki_mid = 40000.0f,
     .pll_ki_high = 32000.0f,
 };
 
 /* XRFOC状态机控制台 */
 XRFOC_Console hfoc = {
    .eAngle = _3PI_2,                /* 使用电角度 */
    .openloopVel = 0.0f,             /* 开环速度 */
    .angle_thresh = deg_to_rad(15),  /* 检测阈值 */
    .Uq_boost = 2.0f,                /* Q轴初始设置电压严格控制在2.0v！ */
    .angle_error = 0.0f,             /* 电角度误差 */
    .cntangle = 0,                   /* 对齐角度计数器 */
    .schedule = _V_F_STAGE,          /* FOC运行阶段 */
 };

 /* 原速度环滤波，涉及了编码器获取转速，故去除。如需要，请以<= 500hz的标准设计 */
 /* 电流环截止频率为 1/0.002 = 500 Hz */
 LowPassFilter CurrentQ_Flt =  { 0.002f, 0.0f, 0};   
 LowPassFilter CurrentD_Flt =  { 0.002f, 0.0f, 0};  
 
 PIDController speed_loop =    { .P = 2, .I = 0, .D = 0, .ramp = 100000, .limit = 6.0f, .error_prev = 0.0f, .output_prev = 0.0f, .integral_prev = 0.0f, .timestamp_prev = 0};
 PIDController position_loop = { .P = 2, .I = 0, .D = 0, .ramp = 100000, .limit = 6.0f, .error_prev = 0.0f, .output_prev = 0.0f, .integral_prev = 0.0f, .timestamp_prev = 0};
 
 PIDController current_q_loop = { .P = 1.2, .I = 0, .D = 0, .ramp = 100000, .limit = 6.0f, .error_prev = 0.0f, .output_prev = 0.0f, .integral_prev = 0.0f, .timestamp_prev = 0};
 PIDController current_d_loop = { .P = 1.2, .I = 0, .D = 0, .ramp = 100000, .limit = 6.0f, .error_prev = 0.0f, .output_prev = 0.0f, .integral_prev = 0.0f, .timestamp_prev = 0};

static void xrfoc_stm32_pwm_write_duty(float duty_a, float duty_b, float duty_c)
{
    ATIM_TIMX_PWM_CH1_CCR = duty_a * MAX_PWM_DUTY;
    ATIM_TIMX_PWM_CH2_CCR = duty_b * MAX_PWM_DUTY;
    ATIM_TIMX_PWM_CH3_CCR = duty_c * MAX_PWM_DUTY;
}
	 
void xrfoc_struct_timestamp_init ( void )
{
    uint32_t now_us = mc_time_us();
    speed_loop.timestamp_prev = now_us;
    position_loop.timestamp_prev = now_us;
    current_q_loop.timestamp_prev = now_us;
    current_d_loop.timestamp_prev = now_us;
    CurrentQ_Flt.timestamp_prev = now_us;
    CurrentD_Flt.timestamp_prev = now_us;
    SMO.last_time = now_us;
}

int main(void)
{
	HAL_Init();                              /* 初始化HAL库 */
    if (mc_adaptor_stm32_hw_init(&stm32_adaptor) != 0) {
        return 1;
    }
    if (mc_clk_init(&stm32_adaptor, 336, 8, 2, 7) != 0) {
        return 2;
    }
	delay_init(168);                         /* 延时初始化 */
    if (mc_uart_init(&stm32_adaptor, 115200) != 0) {
        return 3;
    }
    if (mc_power_init(&stm32_adaptor) != 0) {
        return 4;
    }
	led_init();                              /* 初始化LED */
	key_init();                              /* 初始化按键 */
	adc_init();
	bldc_init(168000/18/2-1,0);
    xrfoc_register_pwm_write_fn(xrfoc_stm32_pwm_write_duty);
	printf("MC_FOC TEST\r\n");
	
	xrfoc_vbus_set(24.0);                          /* 设定 BLDC驱动器额定电压 */
	xrfoc_pretarget(2, CCW);                       /* 预设定极对数和方向 */
    xrfoc_startup_set_mode(XRFOC_STARTUP_IF);      /* 可改为XRFOC_STARTUP_VF */
		
	/* 可能需要重新调参 - 此处对应2208 bldc */
	xrfoc_set_current_q_pid(1.5, 50, 0, 100000);    /* 设置电流环-Q轴PID参数 */
	xrfoc_set_current_d_pid(1.5, 50, 0, 100000);    /* 设置电流环-D轴PID参数 */
	xrfoc_set_speed_pid(0.02, 0.5, 0, 100000, 3);  /* 设置速度环-PID参数 */

	xrfoc_struct_timestamp_init();
	delay_ms(200);
	xrfoc_current_offset_sample();
	
	motor_I_UVW.Target_set = 60.0f;		
	
//	motor_I_UVW.Target_set = 1.0f;		//V
	
//	motor_I_UVW.Target_set = 0.5f;		//A

	motor_console.run_flag  = RUN;
	while (1)
	{
        /* XRFOC主链路已迁移至ADC注入转换中断，以保证采样-控制同频 */
        /* while(1) 仅保留低频任务 */
        /* User define */
	}
}

// 如果需要使用按键控制电机速度，请在加入按键中断。
#ifdef KEY_EXTI_SUPPORT
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
        case KEY0_GPIO_PIN:
            step_up();
            break;
        case KEY1_GPIO_PIN:
            step_down();            
            break;
        case KEY2_GPIO_PIN:
            stop_motor1();                              /* 停机 */
            motor_console.run_flag = STOP;
            break;
        default:        
            break;
    }
}

current_q_pid(3, 200, 0, 100000);    /* 设置电流环-Q轴PID参数 */
current_d_pid(3, 200, 0, 100000);    /* 设置电流环-D轴PID参数 */
speed_pid(0.02, 0.5, 0, 100000, 6);  /* 设置速度环-PID参数 */

#endif


