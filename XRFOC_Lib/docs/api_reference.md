# XRFOC API Reference

## 1. 初始化与平台适配

- `mc_adaptor_stm32_hw_init(struct mc_adaptor_stm32_hw *mcu)`
	- 作用：注册 STM32 平台的时钟/UART/电源/PWM 初始化函数到 adaptor。
	- 返回：`0` 成功，非 `0` 失败。

- `mc_clk_init(void *mcu, uint32_t plln, uint32_t pllm, uint32_t pllp, uint32_t pllq)`
	- 作用：通过 adaptor 初始化系统时钟。

- `mc_uart_init(void *mcu, uint32_t baudrate)`
	- 作用：通过 adaptor 初始化串口。

- `mc_power_init(void *mcu)`
	- 作用：通过 adaptor 初始化功率相关 GPIO（如 SHUTDOWN 引脚）。

## 2. FOC 核心 API

- `xrfoc_vbus_set(float power_supply)`
	- 作用：设置母线电压（V）。

- `xrfoc_pretarget(int pp, int dir)`
	- 作用：设置极对数与方向，并执行初始定向动作。

- `xrfoc_set_current_q_pid(float P, float I, float D, float ramp)`
- `xrfoc_set_current_d_pid(float P, float I, float D, float ramp)`
- `xrfoc_set_speed_pid(float P, float I, float D, float ramp, float limit)`
	- 作用：设置电流环/速度环参数。

- `xrfoc_startup_set_mode(uint8_t mode)`
	- 作用：设置启动模式。
	- 可选：`XRFOC_STARTUP_VF`、`XRFOC_STARTUP_IF`。

- `xrfoc_run(void)`
	- 作用：执行一次控制计算（当前建议在 ADC 注入转换完成回调中调用）。

## 3. PWM 硬件解耦 API

- `xrfoc_register_pwm_write_fn(xrfoc_pwm_write_duty_fn_t fn)`
	- 作用：注册占空比写入回调。
	- 说明：算法层只输出 `duty_a/duty_b/duty_c`，具体寄存器写入由平台层实现。

## 4. SMO/启动模式相关参数

`SMO_PLL_Observer` 关键字段：

- 启动基础：`VF_acc`、`VF_max_vel`、`VF_uq_delta`
- I/F 启动：`IF_current_ref`、`IF_uq_min`、`IF_uq_max`
- 模式选择：`startup_mode`
- 回退机制：`startup_fallback_en`、`fallback_mode`、`align_timeout_count`
- 分段增益：`observer_gain_schedule_en`、`sched_w1`、`sched_w2`
- SMO 分段参数：`h_low/mid/high`
- PLL 分段参数：`pll_kp_low/mid/high`、`pll_ki_low/mid/high`

## 5. 推荐调用顺序（单电机）

1. `HAL_Init`
2. `mc_adaptor_stm32_hw_init`
3. `mc_clk_init`
4. `delay_init`
5. `mc_uart_init`
6. `mc_power_init`
7. 板级外设初始化（`adc_init`、`bldc_init` 等）
8. `xrfoc_register_pwm_write_fn`
9. 设置 `vbus/pp/dir/pid/startup mode`
10. `xrfoc_current_offset_sample`
11. 运行（由中断调度 `xrfoc_run + APP_StateMachine_Handler`）
