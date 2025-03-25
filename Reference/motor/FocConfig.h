#ifndef _FOCCONFIG_H
#define _FOCCONFIG_H
#define  M_2PI                          6.2831853072f
#define  PWM_PERIOD              				(4199)
#define  FOC_PWM_PERIOD          				(0.00005f)
//电机参数设置
#define  MOTOR_PHASE_RESISTANCE  				(3.5f)
#define  MOTOR_PHASE_INDUCTANCE  				(0.0010f)
#define  MOTOR_PLOES             				(2.0f)
#define  MOTOR_KE                				(4.2f)
//ADC采样参数
#define  ADC_SAMPLE_RESISTANCE  				(0.010f)
#define  ADC_AMP                				(5.0f)
#define  VBUS_RATE               				(21.0f)

//滑膜观测器参数设置
#define OB_SMC_GAIN                     2.0f
#define OB_SMC_LPF_K                    0.2f
#define OB_SMC_PLL_KP                   (40.0f)
#define OB_SMC_PLL_KI                   (400.0f)
//预定位角度
#define ALGIN_THETA                    (15.0f / 360.0f * 6.28f - 90.0 / 360.0f * 6.28f)
//if启动参数
#define IF_STARTUP_BEGIN_THETA         (280.0f / 360.0f * 6.28f)
#define IF_STARTUP_IQ_MIN              (0.5f)  //拖动最小电流
#define IF_STARTUP_IQ                  (0.7f)  //拖动电流
#define IF_STARTUP_IQ_KP               (0.1f)  //比例控制系数
#define IF_STARTUP_OMEGA               (50.0f * 6.28f) //拖动的电机电角速度
#define IF_STARTUP_OMEGA_ACC           (50.0f * 6.28f)//角加速度
#define IF_STARTUP_THETA_DIFF          (0.10f * 6.28f)//角度最小差值
#define IF_STARTUP_MAX_TIME            (3.0f)//最大启动时间
//电压开环
#define  MOTOR_CONTORL_VOLTAGE_OPEN_LOOP 0 
//电流闭环
#define  MOTOR_CONTORL_CURRENT_LOOP      1 
//速度环+电流环
#define  MOTOR_CONTORL_SPEED_LOOP        2 
//电机控制模式
#define  MOTOR_CONTORL_MODE              MOTOR_CONTORL_CURRENT_LOOP


//电流环参数
#define CURRENT_IQ_KP                    (MOTOR_PHASE_INDUCTANCE * 1024) 
#define CURRENT_IQ_KI                    (MOTOR_PHASE_RESISTANCE * 1024) 
#define CURRENT_IQ_OUT_MAX               (0.4f * 12.0f)
#define CURRENT_IQ_OUT_MIN               (-0.4f * 12.0f)

//电流环参数
#define CURRENT_ID_KP                    (MOTOR_PHASE_INDUCTANCE * 1024) 
#define CURRENT_ID_KI                    (MOTOR_PHASE_RESISTANCE * 1024) 
#define CURRENT_ID_OUT_MAX               (0.4f * 12.0f)
#define CURRENT_ID_OUT_MIN               (-0.4f * 12.0f)
#endif
