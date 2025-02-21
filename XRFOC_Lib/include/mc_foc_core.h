#ifndef __MC_FOC_CORE_H
#define __MC_FOC_CORE_H

typedef struct{
    int adc1_raw, adc2_raw, adc3_raw; // Raw ADC Values 初始ADC值，采集的原始电流值
    float i_a, i_b, i_c;   // Phase currents 最终得到的相电流
    float v_bus; // DC link voltage  24V电压
    float theta_mech, theta_elec;                           // Rotor mechanical and electrical angle 机械角度和电角度
    float dtheta_mech, dtheta_elec, dtheta_elec_filt;       // 机械角速度，电角速度Rotor mechanical and electrical angular velocit
    float i_d, i_q, i_q_filt, i_d_filt;    // D/Q currents dq电流
    float v_d, v_q;     // D/Q voltages dq电压
    float dtc_u, dtc_v, dtc_w;    // Terminal duty cycles，占空比
    float v_u, v_v, v_w;     // Terminal voltages 
    float k_d, k_q, ki_d, ki_q, alpha;      // Current loop gains, current reference filter coefficient
    float d_int, q_int;     // Current error integrals 误差积分
    int adc1_offset, adc2_offset; // ADC offsets ADC补偿
    float i_d_ref, i_q_ref, i_d_ref_filt, i_q_ref_filt;     // Current references 参考电流
    int loop_count;    // Degubbing counter
    int timeout;      // Watchdog counter
    int mode;        //模式
    int ovp_flag;     // Over-voltage flag
    float p_des, v_des, kp, kd, t_ff; // Desired position, velocity, gians, torque 期望值
    float v_ref, fw_int;  // output voltage magnitude, field-weakening integral
    float cogging[128];    //查表
} ControllerStruct;
#endif // MC_FOC_CORE_H