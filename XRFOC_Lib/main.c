/**
 *  @file:      XRFOC.c
 *  @brief:     Motor run task
 *  @author:    RunDong7582
 *  @date  :    2025 3/21 14:32
 *  @version:   XRFOC v0.1
 */


/* ------------ inc ------------------- */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../common/mc_common.h"
#include "../include/mc_pid.h"
#include "../include/mc_motor.h"
#ifdef STM32
    #include "../include/mc_adaptor.h"
#endif
#include "../include/mc_foc_core.h"
#include "../include/mc_smo_pll.h"
#include "../include/mc_lowpass_filter.h"

#ifdef ESP32
    #include <inttypes.h>
    #include "sdkconfig.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_chip_info.h"
    #include "esp_flash.h"
    #include "esp_system.h"
    #include "esp_timer.h"
    #include "driver/ledc.h"
#endif 

#define _sqrt(a)                  ( _sqrtApprox(a) )
#define _constrain(IMM, Min, Max) ((IMM)<(Min)?(Min) : ((IMM)>(Max)?(Max):(IMM)))

extern struct mc_adaptor_i stm32_adaptor;


extern float   target_speed_openloop = 0.0f;
extern float   shaft_angle;
extern uint32_t open_loop_timestamp = 0;
extern uint32_t now_us     = 0;
extern uint32_t init_time  = 0;

/* ------ Global User Variable --------- */
float voltage_power_supply;
float serial_target;
float filter_ouput;


Motor_params motor_bldc = {
    .pp =  7,
    .dir = 1,
};

SMO_params para = {

    .Rs = 8.25f,
    .Ls = 0.004256f,
    .h = 0.3f,
    .PLL_kp = 521.0f,
    .PLL_ki = 40000.0f,
    .VF_acc = 0.3f,         // 加速度
    .VF_max_vel = 210,
    .VF_uq_delta = 0.006f,
    
};

xrfoc_origin_t  xfoc_orgin;
xrfoc_cur_vol_s Cur_Vol;

PIDController cur_q_loop;
PIDController cur_d_loop;
PIDController speed_loop;
PIDController pos_loop;

LowPassFilter Speed_Flt;
LowPassFilter CurrentQ_Flt;
LowPassFilter CurrentD_Flt;

SMO_PLL_Observer SMO;


/* ========================== ESP32 Periphal =========================== */
#ifdef ESP32
    void setGpioOUTPUT(int pin) {
        gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
        printf("将引脚%d设置为输出模式!\n", pin);
    }

    void setTimer0() {
        ledc_timer_config_t ledc_timer = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_num = LEDC_TIMER_0,
            .duty_resolution = LEDC_TIMER_8_BIT,
            .freq_hz = 30000,
            .clk_cfg = LEDC_AUTO_CLK,
        };
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    }

    void setGpioPWM(int pin, int channel) {
        setGpioOUTPUT(pin);
        ledc_channel_config_t ledc_channel = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = channel,
            .timer_sel = LEDC_TIMER_0, //选择定时器0
            .intr_type = LEDC_INTR_DISABLE,
            .gpio_num = pin,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }

#endif // ESP32

void app_setup()
{
    #ifdef STM32
    /* Periphal config */
        mc_button_init  (&stm32_adaptor);
        mc_pwm_init     (&stm32_adaptor, PWM_FREQ, PWM_ARR_BIT);
        mc_adc_init     (&stm32_adaptor);
        mc_timer_init   (&stm32_adaptor, TIMER_FREQ);
        mc_timer_start  (&stm32_adaptor);
    #endif 
    #ifdef ESP32
        setGpioOUTPUT(ENABLE);
        gpio_set_level(ENABLE, 1);
        printf("完成电机使能初始化设置!\n");
        setTimer0();
        setGpioPWM(pwmA, LEDC_CHANNEL_3);
        setGpioPWM(pwmB, LEDC_CHANNEL_4);
        setGpioPWM(pwmC, LEDC_CHANNEL_5);
        printf("完成PWM初始化设置!\n");
    #endif
}
void XRFOC_Init()
{
    xrfoc_module_struct_init();
}


void xrfoc_module_struct_clear  ( void )
{
    memset(&SMO,            0,          sizeof(SMO));
    memset(&Cur_Vol,        0,      sizeof(Cur_Vol));
    memset(&pos_loop,       0,     sizeof(pos_loop));
    memset(&Speed_Flt,      0,    sizeof(Speed_Flt));
	memset(&speed_loop,     0,   sizeof(speed_loop));
	memset(&cur_q_loop,     0,   sizeof(cur_q_loop));
    memset(&cur_d_loop,     0,   sizeof(cur_d_loop));
	memset(&CurrentQ_Flt,   0,  sizeof(CurrentQ_Flt));
	memset(&CurrentD_Flt,   0,  sizeof(CurrentD_Flt));

    printf("XRFOC: Clear the struct!\n");
}

int xrfoc_module_struct_init    ( void )
{
    xrfoc_module_struct_clear();
    
    /* fill the SMO with the set para */
    smo_para_update(&SMO, para);

    /* lpf speed d-q initialize */
    lowpass_filter_init(&Speed_Flt, 0.01f);         //速度环滤波 Tc = 0.01s <=> 带宽100Hz

    /* lpf curr d-q initialize */
    lowpass_filter_init(&CurrentQ_Flt, 0.002f);     //电流环 Q轴滤波 Tc = 0.002s <=> 带宽500Hz   
    lowpass_filter_init(&CurrentD_Flt, 0.002f);     //电流环 D轴滤波 Tc = 0.002s <=> 带宽500Hz
    
    /* pid speed (w omega) initialize */
    pid_init(&speed_loop,  2.0f, 0.0f, 0.0f, 100000.0f, voltage_power_supply / 2.0f);
   
    /* pid position (angle) intialize */
    pid_init(&pos_loop,    2.0f, 0.0f, 0.0f, 100000.0f, voltage_power_supply / 2.0f);

    /* pid curr d-q initialize */
    pid_init(&cur_q_loop,  1.2f, 0.0f, 0.0f, 100000.0f, voltage_power_supply / 2.0f);
    pid_init(&cur_d_loop,  1.2f, 0.0f, 0.0f, 100000.0f, voltage_power_supply / 2.0f);

    return 1;
}



int main (void)
{
    if (xrfoc_module_init())
    {
        printf("Foc module init success.\n");
    }
    else
    {
        printf("Foc module init failed.\n");
        return 2;
    }

    return 0;
}


/* 用于shell控制、解析上位机发送指令 */
/* refer to my project " Gesture recognition based on Raspiberry 4B & STM32 G4 "*/
/* strtok is very equal to this job */
void serialReceiveUserCommand ( void ) 
{

}

float serial_motor_target ( void ) 
{
    return serial_target;
}