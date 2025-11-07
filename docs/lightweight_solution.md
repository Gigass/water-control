# 轻量级水箱液位控制实现说明

## 模型与假设
- **单水箱模型**：截面积 `A = 0.5 m^2`，水位高度 `h` 作为被控变量。
- **流量关系**：进水 `Q_in = 0.12 m^3/s + 扰动`；出水 `Q_out = k_v u √h`，其中 `k_v = 0.7`，`u` 为 0~1 的阀门开度。
- **测量信号**：压力传感器等效为直接读取液位，叠加小幅高斯噪声（标准差 2 mm）。
- **扰动场景**：350~450 s 之间进水额外增加 `0.04 m^3/s`，检验控制器的扰动抑制能力。

## 控制策略
- **控制器**：PI 控制，`Kp = 1.8`，`Ki = 0.12`，默认启用前馈并设置 `ctrl.ff_gain = 0.5`，用于抵消稳态进水流量；可按需调整系数或关闭前馈。
- **抗积分饱和**：当阀门达到饱和时暂停积分，避免风up。
- **执行器建模**：阀门输出经过一阶滞后 (`τ = 1.0 s`)，默认死区关闭（`deadband = 0`），再乘以 `√h` 形成真实出流量；若需考察阀门间隙，可重新启用死区。
- **设定值**：0~200 s 追踪 `0.5 m`，200 s 后阶跃到 `0.65 m`。

- **测量滤波**：对传感器读数通过一阶低通 (`τ = 3.5 s`)，在抑制噪声和响应速度之间取得折中。
- **传感器噪声**：高斯噪声（2 mm）叠加在滤波输出上。
- **进水扰动**：在阶跃扰动之外加入均值 0、标准差 `0.001 m^3/s` 的随机噪声，用于验证鲁棒性。
- **可扩展性**：`params` 结构中可独立调节噪声、滤波、执行器动态以匹配不同工况。

## 仿真脚本
- 位置：`src/tank_level_sim.m`。
- 积分方式：固定步长 `0.1 s` 的前向欧拉法，仿真总时长 600 s。
- 输出：三组曲线（液位/设定值/滤波测量，控制器与执行器开度，功率 proxy 与进水噪声）以及关键指标（超调、调节时间、IAE、阀门平均值、能量）。
- 运行后会在工作区生成 `tank_level_result` 结构，便于后续绘图或导出报告。

## 使用方法
1. 在 MATLAB 中将仓库根目录加入路径：`addpath(genpath(pwd));`
2. 运行脚本：`tank_level_sim`
3. 查看生成的图像窗口与命令行结果；可根据需要调整脚本顶部的参数。

## 可调参数
- `params` 结构中的截面积、阀门系数、进水基值、噪声强度、滤波时间常数、执行器滞后/死区。
- `ctrl` 结构中的 `Kp/Ki`、阀门上下限、前馈开关，实现不同响应速度或更保守的控制策略。
- `setpoint`、`disturbance` 序列，可自定义其他测试场景（如多段设定值、随机扰动）。
- `metrics` 计算逻辑可根据需要扩展（例如添加 ISE、稳态误差等），便于课程报告引用。

## 后续扩展建议
- 进一步比较不同控制策略（PID、带前馈的 PI、MPC 等）的指标表现，自动生成表格或 PDF 报告。
- 在既有模型基础上引入液位上下限报警、传感器冗余、故障注入等，构建更完善的仿真试验集。
- 将 `tank_level_result` 导出为 CSV/LaTeX 表格，为实验报告提供可复现的数据支撑。

## Simulink 模型特性
- `build_tank_level_model.m` 通过 API 自动生成 `tank_level_control.slx`，结构与脚本保持一致。
- 控制/执行器链路：`Setpoint → InvSqrt → FFGain → FFProduct` 与 `PI_Controller` 输出在 `ControlSum` 叠加后，经过 `ValveSaturation → ActuatorLag → DeadbandMap → KvGain`；默认启用前馈 (`FFGain = 0.6`)，DeadbandMap 为线性通道，可按需修改。
- 测量链路：`LevelIntegrator → LevelFilter → Measurement + Noise`，在控制器前实现低通滤波。
- 扰动链路：`QinBase + 阶跃扰动 + 随机噪声` 输入到水箱，噪声通过 `Random Number + Gain` 模块实现。
- 监控链路：`LevelScope` 展示设定值/测量，`ValveScope` 展示控制器/执行器/有效开度，`FlowScope` 展示总进水与随机扰动。

## Simulink 使用方法
1. 在 MATLAB 中运行 `build_tank_level_model`（文件位于 `src/`）；脚本会生成 `tank_level_control.slx` 并在 Simulink 中打开。
2. 如需立即仿真，可执行 `build_tank_level_model(true)`，脚本会在生成后调用 `sim` 并将结果保存到 `tank_level_simout` 变量。
3. 模型顶层包含：
   - `Setpoint` 阶跃源（0.5→0.65 m）
   - `QinBase`、阶跃扰动以及 `QinNoise` 随机噪声共同构成的进水
   - 控制链路：`PI_Controller`、`ValveSaturation`、`ActuatorLag`、`DeadbandMap`
   - `LevelIntegrator` + `LevelFilter` + 测量噪声
   - 三个 Scope：`LevelScope`、`ValveScope`、`FlowScope`
4. 如需调整参数，可编辑脚本顶部的 `params` 结构或在生成后的模型中修改各 block 的 `Gain/Step` 参数，再保存模型。
