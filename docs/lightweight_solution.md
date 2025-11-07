# 轻量级水箱液位控制实现说明

## 模型与假设
- **单水箱模型**：截面积 `A = 0.5 m^2`，水位高度 `h` 作为被控变量。
- **流量关系**：进水 `Q_in = 0.12 m^3/s + 扰动`；出水 `Q_out = k_v u √h`，其中 `k_v = 0.7`，`u` 为 0~1 的阀门开度。
- **测量信号**：压力传感器等效为直接读取液位，叠加小幅高斯噪声（标准差 2 mm）。
- **扰动场景**：350~450 s 之间进水额外增加 `0.04 m^3/s`，检验控制器的扰动抑制能力。

## 控制策略
- **控制器**：PI 控制，`Kp = 4.5`，`Ki = 0.25`，带上下限饱和 (`0~1`)。
- **抗积分饱和**：当阀门达到饱和时暂停积分，避免风up。
- **设定值**：0~200 s 追踪 `0.5 m`，200 s 后阶跃到 `0.65 m`。

## 仿真脚本
- 位置：`src/tank_level_sim.m`。
- 积分方式：固定步长 `0.1 s` 的前向欧拉法，仿真总时长 600 s。
- 输出：液位/设定值曲线、阀门开度与进水扰动曲线，并在命令行给出最终液位与阀门状态。

## 使用方法
1. 在 MATLAB 中将仓库根目录加入路径：`addpath(genpath(pwd));`
2. 运行脚本：`tank_level_sim`
3. 查看生成的图像窗口与命令行结果；可根据需要调整脚本顶部的参数。

## 可调参数
- `params` 结构中的截面积、阀门系数、进水基值、噪声强度。
- `ctrl` 结构中的 `Kp/Ki`、阀门上下限，实现不同响应速度或更保守的控制策略。
- `setpoint`、`disturbance` 序列，可自定义其他测试场景（如多段设定值、随机扰动）。

## 后续扩展建议
- 将脚本转换为 Simulink 模块化实现，方便与更复杂的物理模型对接。
- 引入非线性阀门特性、测量延迟、传感器漂移等，更贴合实际工况。
- 尝试自适应控制或 MPC，比较与基础 PI 策略的性能差异。

## Simulink 模型规划
- **模型名称**：`tank_level_control_slx`；顶层包含参数配置、设定值/扰动信号、被控对象、PI 控制器以及监测模块。
- **子系统划分**：
  - `Setpoint_Disturbance`：输出设定值阶跃和进水扰动；使用 Step/Signal Builder 或 Function-Call Subsystem 生成。
  - `TankPlant`：输入为进水和阀门开度，内部按 `dh/dt = (Q_in - k_v u √h)/A` 积分得到 `h` 并输出液位与传感器信号。
  - `PI_Controller`：以液位测量与设定值为输入，输出阀门开度，包含抗饱和逻辑。
  - `Scopes`：用于显示液位响应与阀门开度。
- **实现方式**：通过 MATLAB 脚本自动搭建（调用 Simulink API 创建系统、块、连线），便于在无现成 `.slx` 文件的情况下复现模型，并确保参数与 `tank_level_sim.m` 保持一致。

## Simulink 使用方法
1. 在 MATLAB 中运行 `build_tank_level_model`（文件位于 `src/`）；脚本会生成 `tank_level_control.slx` 并在 Simulink 中打开。
2. 如需立即仿真，可执行 `build_tank_level_model(true)`，脚本会在生成后调用 `sim` 并将结果保存到 `tank_level_simout` 变量。
3. 模型顶层包含：
   - `Setpoint` 阶跃源（0.5→0.65 m）
   - `QinBase` 常量与两个扰动阶跃组成的进水信号
   - `PI_Controller` 与 `ValveSaturation`（0~100%）
   - `LevelIntegrator`（受限在 0~2 m）
   - `Noise`（Random Number）与 `NoiseGain` 叠加到测量端
   - 两个 Scope：`LevelScope` 显示设定值/液位，`ValveScope` 显示阀门开度与进水流量
4. 如需调整参数，可编辑脚本顶部的 `params` 结构或在生成后的模型中修改各 block 的 `Gain/Step` 参数，再保存模型。
