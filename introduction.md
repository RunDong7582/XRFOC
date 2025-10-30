 <div align = "center">
	<a name ="Top"></a>
	<h1>XRFOC</h1>
		<strong>
			<p>👉👉👉&nbsp;&nbsp;
				基于DengFOC的二次开发库
				&nbsp;&nbsp;
		👈👈👈</p>
	</strong>
</div>

> 本项目主要参考自：[DengFoc](https://github.com/ToanTech/DengFOC_Lib),对其Cpp算法库进行移植为C库。

对于XRFOC_Lib的使用说明如下：

- 实际的运行平台，在STM32F4正点原子的电机控制板上，相关的外设配置（adc、tim）自行学习，建议使用cubemx进行配置。（自行实现adc.c，tim.c）
- main.c无任何参考价值。（参考dengfoc的初始化）
- 作者二次开发过程可以参考Stage Summary文件夹的周报，写的很详细。
- 请注意获取时间戳的方式，如果你在stm32平台上，应该使用`HAL_GetTick()`.注意各种细节。
- 最终实现的效果与dengfoc有区别，作者认为是硬件平台的设计有关。问题在于uqud很小，只能选择限幅在（0~2v），而实际应该在0-1/2\*供电电压（0~12v） 之间，所以无感foc运行转速受限，按理来说可以和Dengfoc一样运行至2100转左右。
- 将V/F启动算法替换为I/F启动，这是目前基本所有做电控厂商的启动方式。
- 后续DengFOC会推出simulink模型，有助于推动上述问题。建议购买DengFOC板子结合配套视频学习灯哥的思想，这对于入门无感FOC有巨大的帮助（很客观）。
- 如果你觉得对你有帮助的话，请给我一个star🌟吧，谢谢。

---

Usage instructions for XRFOC_Lib are as follows:
- The actual runtime platform is the STM32F4 Zhengdian Atom motor control board. Configure relevant peripherals (ADC, TIM) independently; it is recommended to use Cubemx for configuration. (Implement adc.c and tim.c yourself)
- main.c has no reference value. (Refer to dengfoc's initialization) 
- The author's secondary development process is documented in the weekly reports within the Stage Summary folder, which are very detailed. 
- Pay attention to the method for obtaining timestamps. On the STM32 platform, you should use `HAL_GetTick()`. Note all relevant details. 
- The final implementation differs from DengFOC's results, which the author attributes to hardware platform design. The issue lies in the small UQUD range, limiting operation to (0~2V) instead of the intended 0-1/2*supply voltage (0~12V). Consequently, the sensorless FOC's rotational speed is constrained, whereas it should theoretically reach around 2100 RPM like DengFOC. 
- Replacing the V/F startup algorithm with I/F startup is now the standard approach adopted by virtually all manufacturers in the field of electronic control. 
- DengFOC will release a Simulink model in the future, which will help address the aforementioned issues. It is recommended to purchase the DengFOC board and study the accompanying videos to understand Dengge's methodology. This is immensely helpful for beginners learning sensorless FOC (very objectively). 
- If you found this helpful, please give it a star🌟. Thank you.