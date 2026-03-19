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

对于XRFOC_Lib的整体架构及使用说明如下：

### 🎯 架构概览 (Architecture Overview)

XRFOC_Lib 是一个基于 DengFOC 的 C 语言移植版，主要针对嵌入式微控制器进行了底层解耦与算法整理：
1. **驱动层解耦**：将与MCU强相关的 ADC 和 TIM 外设功能与核心 FOC 算法剥离。需要使用者自行实现针对当地平台的硬件初始化和读取 (如 `adc.c`，`tim.c`)。
2. **核心算法层**：包含基本的 FOC 运算（如 Park/Clark 变换、PID 控制、SVPWM），通过标准 C 库函数实现，具有高度可移植性。
3. **接口层**：定义了如获取时间戳等底层调用的接口（在 STM32 平台上需对接为 `HAL_GetTick()`）。

### 🛠️ 新增改动与开发须知 (Changes & Development Notes)

- **算法替换**：原先的 V/F 启动算法已被替换为目前电控厂商更常用的 **I/F 启动算法**。
- **运行平台验证**：目前代码的主要实验平台是 STM32F4（正点原子电机控制板），`main.c` 主要是基础测试入口，并无实际完整的商用级初始化，请务必参考 DengFOC 的初始化逻辑进行自我重构。
- **自定义二次开发**：⚠️ **注意：本库包含了一系列新增功能改动，但并非“开箱即用”。** 使用者需要基于自己的具体硬件和电机自行完成 **二次开发、参数验证和闭环调试**。
- **性能差异提示**：最终实现效果与原始 DengFOC 略有不同（主要涉及 $U_q/U_d$ 幅值设定与当地硬件分压采样设计的匹配问题）。可能存在高速运行受限的情况，需要使用者针对自身硬件（如供电电压）来调整限幅和比例系数。
- **开发过程参考**：作者的二次开发详细过程和踩坑记录可以参考 `Stage Summary` 文件夹下的周报，其中包含详细的理论思考及调试日志；代码流程图请参考 `assets/diagram.png`。

---

Usage instructions for XRFOC_Lib are as follows:
- The actual runtime platform is the STM32F4 Zhengdian Atom motor control board. Configure relevant peripherals (ADC, TIM) independently; it is recommended to use Cubemx for configuration. (Implement adc.c and tim.c yourself)
- main.c has no reference value. (Refer to dengfoc's initialization) 
- The author's secondary development process is documented in the weekly reports within the Stage Summary folder, which are very detailed. 
- Pay attention to the method for obtaining timestamps. On the STM32 platform, you should use `HAL_GetTick()`. Note all relevant details. 
- The final implementation differs from DengFOC's results, which the author attributes to hardware platform design. The issue lies in the small UQUD range, limiting operation to 0-2V instead of the intended 0-1/2*supply voltage (0-12V). Consequently, the sensorless FOC's rotational speed is constrained, whereas it should theoretically reach around 2100 RPM like DengFOC. 
- Replacing the V/F startup algorithm with I/F startup is now the standard approach adopted by virtually all manufacturers in the field of electronic control. 
- DengFOC will release a Simulink model in the future, which will help address the aforementioned issues. It is recommended to purchase the DengFOC board and study the accompanying videos to understand Dengge's methodology. This is immensely helpful for beginners learning sensorless FOC (very objectively). 
- If you found this helpful, please give it a star🌟. Thank you.

## 📄 License & Terms of Use / 开源协议与使用条款

本项目遵循 **PolyForm Noncommercial License 1.0.0** 协议开源。

**允许 (Permitted):**
- ✅ 学习研究 (Learning & Study)
- ✅ 个人项目 (Personal Projects)
- ✅ 代码二次开发及验证 (Secondary Development & Evaluation)
- ✅ 学术用途 (Academic Research)

**禁止 (Prohibited):**
- ❌ 任何形式的商业用途 (Any Commercial Use)
- ❌ 闭源作为商业项目的一部分发布 (Distribution as part of a closed-source commercial product)

详见项目根目录的 `LICENSE` 文件。任何由本代码衍生出的开发，在再分发时必须保留原作者信息（署名）并同时包含同样的免责及非商业限制声明。