#ifndef __MC_MOTOR_H
#define __MC_MOTOR_H

#include <stdint.h>

typedef struct MotorParams{
	float rs;        //电机电阻
	float ls;        //电机电感
	float vb;        //电机母线电压
	float vi;        //最大母线电流
	int32_t Fsmopos; //滑膜常数F
	int32_t Gsmopos; //滑膜常数G
	int32_t Ialpha;  //alpha轴电流
	int32_t Ibeta;   //beta轴电流
	int32_t Valpha;  //alpha轴电压
	int32_t Vbeta;   //beta轴电压
	int32_t Ealpha;  //估算alpha轴反电动势
	int32_t Ebeta;   //估算beta轴反电动势
	int32_t Id;      //d轴电流
	int32_t Iq;		 //q轴电流
	// uint32_t count;  //时间计数器 ?
	int32_t PreTime1;//预定位时间1
	int32_t PreTime2;//预定位时间2
	int32_t VfTime;  //斜坡强拉时间
	int32_t PreVd;    //预定位的Vd大小
	int32_t MaxVfAngleSpeed; //Vf的最大角速度
	int32_t MaxVfVq;   //Vf的最电压
	int32_t StartVfVq; //Vf起始Vq
	int32_t VfAngleTime; //angle加速度增加的时间间隔
	int32_t VfVqTime; //Vq增加的时间间隔
	int32_t VfAngleTimeCount;//angle加速度增加的时间计数
	int32_t VfVqTimeCount; //Vq增加的时间计数
	int32_t VfAngleAdd; //Vf中的Angle增加的加速度
	int32_t VfAngle;    //Vf中的Angle
	int32_t VfVq;		//Vf中的Vq
	int32_t SMOAngle;   // SMO得到的角度
	int32_t SMOSpeed;   // SMO得到的速度
	int32_t VdISum;		// Vd积分器
	int32_t VdKp;		// VdKp
	int32_t VdKi;		// VdKi
	int32_t VqOut;		// 闭环输出Vq
	int32_t Iu;			//输入电流
	int32_t Iv;			//输入电流
	int32_t EstIalpha;  //滑膜估算Ialpha
	int32_t EstIbeta;	//滑膜估算Ibeta
	int32_t Zalpha;		//滑膜Zalpha
	int32_t Zbeta;		//滑膜Zbeta
	int32_t Kslide;		// 滑膜增益
	int32_t PLLLastEa;	// PLL Ea
	int32_t PLLLastEb;	// PLL Eb
	int32_t PLLKp;		//PLLKp
	int32_t PLLKi;		//PLLKi
	int32_t PLLISum;	// PLL积分器
	int32_t PWMOut[3];  // pwm输出
	int32_t PWMHalfPerMax; // 半pwm duty
	int8_t Dir;			// 电机方向，0负1正
	uint8_t Pole;		// 极对数
	int32_t MotorSpeed; // 电机机械转速 r/min
}MotorParams;

// typedef struct {
//     PWM_Interface *pwm;
//     ADC_Interface *adc;
//     PID_Controller pid_id;
//     PID_Controller pid_iq;
//     float est_angle;
//     float est_speed;
// } Motor;

// void Motor_Init(Motor *motor, PWM_Interface *pwm, ADC_Interface *adc);
// void Motor_Run(Motor *motor, float target_speed);

#endif 