/**
 *  @file:      XRFOC_Lib/src/mc_pid.c
 *  @brief:     Pid algorithm implemented.
 *  @author:    RunDong7582
 *  @date  :    2025 3/20 16:17
 *  @version:   XRFOC v0.1
 */

#include "../include/mc_pid.h"

#define _constrain(IMM, Min, Max) ((IMM)<(Min)?(Min) : ((IMM)>(Max)?(Max):(IMM)))

 /**
  * @brief 初始化PID控制器参数
  * 
  * @param pid 待初始化的PID控制器实例
  * @param P 比例增益系数（Kp）
  * @param I 积分增益系数（Ki）
  * @param D 微分增益系数（Kd）
  * @param ramp 输出斜坡时间（防止输出突变）
  * @param limit 输出限幅值（绝对值上限）
  * @note 
  * - 典型调用示例：`pid_init(&speed_pid, 2.0, 0.5, 0.1, 1000, 12.0)`
  * - 若`ramp > 0`，则输出变化率会被约束在`±ramp * Ts`范围内（Ts为控制周期）
  * - `limit`需根据PWM最大电压、抬升电压等合理设置
  */

void pid_init ( PIDController* pid, float P, float I, float D, float ramp, float limit ) {

    pid->P = P;
    pid->I = I;
    pid->D = D;
    pid->output_ramp = ramp;
    pid->limit = limit;
    pid->error_prev = 0;
    pid->output_prev = 0;
    pid->integral_prev = 0;
    pid->timestamp_prev = timer_counter;

}

/**
 * @brief 根据误差计算PID控制输出
 * 
 * @param pid 已初始化的PID控制器实例
 * @param error 当前测量值与目标值的误差
 * @return float 经积分限幅、微分滤波和输出斜坡限制后的控制量
 * 
 * 执行步骤：
 * 1. 计算比例项 (P * error)
 * 2. 积分项累加并进行积分限幅
 * 3. 微分项计算并滤波（使用上一时刻误差）
 * 4. 叠加三项输出并进行输出限幅和斜坡限制
 */
float pid_process ( PIDController* pid, float error )  
{
    // fetch current timestamp
    uint32_t timestamp = timer_counter;
    
    // calculate delta time
    uint32_t dt = timestamp - pid->timestamp_prev;
 
    // convert to second . 1 tick = 1ms. Ts: .s
    float Ts = (float) dt * 1e-3f;

    if ( Ts > 0.5f ) Ts = 1e-3f;
    
    /* Propotion branch */
    float proportional = pid->P * error;

    /* Intergration branch -> Tustin plot intergraion */ 
    /* 其实就是梯形近似法积分 */
    float intergral = pid->integral_prev + 0.5f * Ts * pid->I * ( error + pid->error_prev );
    
    intergral = _constrain( intergral, -pid->limit, pid->limit );
    /* Derivative branch */
    /* 其实就是用y的变化量近似y的微分 (一阶泰勒公式) */
    float derivative = pid->D * (error - pid->error_prev) / Ts;

    /* Accumulate */
    float Output = proportional + intergral + derivative;
    Output = _constrain(Output, -pid->limit, pid->limit);
    
    if ( pid->output_ramp > 0 ) {
        /* limit the rate of pid */
        float Output_rate = (Output - pid->output_prev) / Ts;
        if ( Output_rate > pid->output_ramp ) 
            Output = pid->output_prev + pid->output_ramp * Ts;
        else if ( Output_rate < -(pid->output_ramp) )
            Output = pid->output_prev - pid->output_ramp * Ts;
    }

    pid->integral_prev = intergral;
    pid->output_prev = Output;
    pid->error_prev = error;
    pid->timestamp_prev = timestamp;

    return Output;
}