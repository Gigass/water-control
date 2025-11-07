# Tank Level Control Project

Comprehensive MATLAB/Simulink implementation for controlling the liquid level of a single tank via an actuated outlet valve.  
This repository documents the full workflow from requirement analysis to multiple control refinements, including visualization assets and reproducible scripts.

---

## Repository Structure

```
.
├── docs/
│   ├── requirements_analysis.md   # 初步需求与难度评估 + 轻量方案
│   ├── lightweight_solution.md    # 实现说明、参数配置、Simulink 使用方法
│   ├── final_report.md            # 全流程详尽报告（需求→建模→调优→指标）
│   └── system_mindmap.md          # Mermaid 思维导图，源自手绘框架
├── src/
│   ├── tank_level_sim.m           # MATLAB 仿真脚本（含指标输出、绘图）
│   └── build_tank_level_model.m   # 生成/仿真 Simulink 模型的脚本
├── Tank Level Control.png         # 最终仿真响应曲线
└── README.md
```

Key deliverables:
- **MATLAB脚本**：脚本化仿真、参数调整、指标记录 (`tank_level_sim.m`)。
- **Simulink脚本**：自动搭建 `tank_level_control.slx` 并可直接运行 (`build_tank_level_model.m`)。
- **文档体系**：从需求分析到最终报告、思维导图和最佳实践说明。

---

## Getting Started

### Environment
- MATLAB R2023b+ (tested with R2025a) with Simulink.
- Ensure MATLAB runs with JVM enabled (Simulink construction relies on Java services).

### Quick Start
1. Clone this repo and open MATLAB at the repository root.
2. Add paths:
   ```matlab
   addpath(genpath(pwd));
   ```
3. Run the MATLAB simulation:
   ```matlab
   tank_level_sim
   ```
   This spawns plots, prints performance metrics, and stores a `tank_level_result` struct in the workspace.
4. Generate the Simulink model:
   ```matlab
   build_tank_level_model       % creates tank_level_control.slx
   build_tank_level_model(true) % optional: create + simulate
   ```
   The generated model matches the MATLAB script parameters, includes scopes for level, valve stages, and inflow signals.

---

## MATLAB Simulation Details (`src/tank_level_sim.m`)
- **Physics**: `dh/dt = (Q_in - k_v u √h)/A` with configurable tank area, valve coefficient, and inflow statistics.
- **Control**: Tuned PI (`Kp = 1.8`, `Ki = 0.12`) + feedforward gain 0.5, anti-windup, actuator lag τ=1.0 s (deadband disabled by default).
- **Disturbances**: 0.5→0.65 m setpoint step at 200 s, inflow step disturbance (350–450 s) plus Gaussian noise (`σ = 0.001 m³/s`), measurement noise 2 mm filtered by τ=3.5 s LPF.
- **Outputs**:
  - Figure with three panels: level tracking, controller/actuator signals, control energy vs inflow.
  - Command-window metrics (overshoot, settling time, IAE, valve statistics, control energy).
  - Workspace variable `tank_level_result` for further analysis/reporting.

To experiment:
- Modify `params`, `ctrl`, `filter`, `actuator`, `setpoint`, `disturbance` in the script header.
- Re-run `tank_level_sim` to evaluate changes; metrics update automatically.

---

## Simulink Model Generation (`src/build_tank_level_model.m`)
- Programmatically builds **`tank_level_control.slx`** with blocks:
  - `Setpoint`, `Disturbance` (steps + random inflow).
  - `PI_Controller`, `Feedforward` (inverse sqrt + gain), `ValveSaturation`, `ActuatorLag`, `DeadbandMap`.
  - `Tank plant` (integrator + √h), measurement LPF + noise, scopes for level/valve/flow.
- Parameters synchronized with MATLAB script (Kp/Ki, τ values, noise gains).
- `build_tank_level_model(true)` runs `sim` after building and stores the `Simulink.SimulationOutput` in `tank_level_simout`.

If you manually edit the model, re-run the script to regenerate from the canonical configuration.

---

## Documentation Roadmap
- **`docs/requirements_analysis.md`**: 原始需求、建模难度评估、轻量方案概述。
- **`docs/lightweight_solution.md`**: 参数详解、运行步骤、Simulink 结构说明、调参建议。
- **`docs/final_report.md`**: 题目理解→建模→多轮调优→指标对比的完整文字报告，包含关键参数演进表与最终结果描述，可直接用于课程提交。
- **`docs/system_mindmap.md`**: Mermaid 思维导图（对应手绘框架“设定值→误差→PID→执行器→水箱→测量→输出”），预览即可可视化整体架构。

---

## Results Snapshot

- **Figure**: `Tank Level Control.png`（最终仿真响应）。三幅图展示液位、阀门信号、功率 vs. 随机扰动，验证系统在阶跃与扰动下的跟踪能力。
- **Typical metrics** (latest tuning):
  - Final level ≈ 0.652 m
  - Overshoot ≈ 0.10 m
  - Settling time ≈ 200–220 s
  - IAE ≈ 25
  - Average valve opening ≈ 22%
  - Control energy ≈ 30 (`∫u² dt`)

---

## Extending the Project
1. **Parameter sweeps**: Loop over `ctrl`/`params` to benchmark different tunings; capture metrics from `tank_level_result`.
2. **Advanced control**: Replace PI with MPC, adaptive control, or fuzzy logic by editing the MATLAB script and corresponding Simulink controller block.
3. **Fault scenarios**: Introduce sensor bias, actuator saturation/deadband, or delayed measurements (Simulink offers Delay blocks) to test robustness.
4. **Reporting**: Export `tank_level_result` to CSV/LaTeX tables, or embed figures into `docs/final_report.md` for academic submissions.

---

## License
This repository is provided for educational use. Adapt and extend it as needed for your course or research work.
