% tank_level_sim.m
% 轻量级单水箱液位控制仿真（PI 控制 + 过滤/非线性/指标输出）

%% 参数定义
rng(0);                       % 固定随机种子，保证可重复性
params.A = 0.5;               % 水箱截面积 (m^2)
params.kv = 0.7;              % 阀门流量系数
params.rho = 1000;            % 水密度 (kg/m^3)
params.g = 9.81;              % 重力加速度 (m/s^2)
params.q_in_base = 0.12;      % 基础进水流量 (m^3/s)
params.q_in_noise_std = 0.001; % 进水随机扰动强度

ctrl.Kp = 2.0;                % PI 控制器比例系数
ctrl.Ki = 0.10;               % PI 控制器积分系数
ctrl.u_min = 0.0;           % 阀门最小开度 (0-1)
ctrl.u_max = 1.0;           % 阀门最大开度 (0-1)
ctrl.enable_feedforward = true;
ctrl.ff_gain = 0.6;

filter.tau = 4.0;           % 一阶低通滤波器时间常数 (s)

actuator.tau = 1.0;         % 执行器一阶滞后 (s)
actuator.deadband = 0.0;    % 阀门死区

sim.t_end = 600;            % 仿真时长 (s)
sim.dt = 0.1;               % 时间步长 (s)
sim.time = 0:sim.dt:sim.t_end;

%% 设定值与扰动
setpoint = 0.5 * ones(size(sim.time));            % 初始液位设定 (m)
setpoint(sim.time >= 200) = 0.65;                 % 200s 时阶跃上升

disturbance = zeros(size(sim.time));
disturbance(sim.time >= 350 & sim.time < 450) = 0.04;  % 进水短暂升高

noise_std = 0.002;  % 传感器噪声标准差 (m)

%% 仿真状态变量
N = numel(sim.time);
h = zeros(1, N);            % 液位高度 (m)
h(1) = 0.5;                 % 初始液位
measurement_raw = zeros(1, N);
measurement_filt = zeros(1, N);
measurement_filt(1) = h(1);
u_cmd = zeros(1, N);        % 控制器输出（饱和后）
u_act = zeros(1, N);        % 执行器实际输出
u_eff = zeros(1, N);        % 死区后的有效开度
qin_history = zeros(1, N);
integral_term = 0;
alpha = sim.dt / max(filter.tau, sim.dt);
deadband_den = max(1 - actuator.deadband, eps);
u_eq = params.q_in_base / max(params.kv * sqrt(max(h(1), 1e-4)), 1e-4);
u_eq = min(max(u_eq, ctrl.u_min), ctrl.u_max);
u_act(1) = u_eq;
u_eff(1) = min(max(u_eq, 0), 1);
qin_history(1) = params.q_in_base;

%% 主仿真循环
for k = 2:N
    t = sim.time(k);
    measurement_raw(k) = h(k-1) + noise_std * randn;     % 压力传感器读数
    measurement_filt(k) = measurement_filt(k-1) + alpha * (measurement_raw(k) - measurement_filt(k-1));
    error = measurement_filt(k) - setpoint(k);            % 液位高于设定值 -> 误差为正
    integral_term = integral_term + error * sim.dt;

    % Feedforward：根据设定值估算保持液位所需的阀门开度
    if ctrl.enable_feedforward
        desired_head = max(setpoint(k), 1e-4);
        u_ff = ctrl.ff_gain * params.q_in_base / max(params.kv * sqrt(desired_head), 1e-4);
        u_ff = min(max(u_ff, ctrl.u_min), ctrl.u_max);
    else
        u_ff = 0;
    end

    % PI 控制律：误差为正时增大阀门开度
    raw_u = u_ff + ctrl.Kp * error + ctrl.Ki * integral_term;
    u_cmd(k) = min(max(raw_u, ctrl.u_min), ctrl.u_max);

    % 抗积分饱和：当饱和发生时不积累误差
    if raw_u ~= u_cmd(k)
        integral_term = integral_term - error * sim.dt;
    end

    % 执行器一阶滞后
    u_act(k) = u_act(k-1) + (sim.dt / max(actuator.tau, sim.dt)) * (u_cmd(k) - u_act(k-1));
    u_act(k) = min(max(u_act(k), ctrl.u_min), ctrl.u_max);

    % 阀门死区与非线性映射
    u_eff(k) = u_act(k);
    u_eff(k) = min(max(u_eff(k), 0), 1);

    qin_history(k) = params.q_in_base + disturbance(k) + params.q_in_noise_std * randn;
    qin_history(k) = max(qin_history(k), 0);
    q_in = qin_history(k);
    q_out = params.kv * u_eff(k) * sqrt(max(h(k-1), 0));
    dh = (q_in - q_out) / params.A;
    h(k) = max(h(k-1) + dh * sim.dt, 0);
end

%% 可视化
figure('Name','Tank Level Control','Color','w','Position',[100 80 950 700]);
subplot(3,1,1);
plot(sim.time, h, 'LineWidth', 1.5); hold on;
plot(sim.time, setpoint, '--', 'LineWidth', 1.2);
plot(sim.time, measurement_filt, ':', 'LineWidth', 1);
ylabel('液位高度 (m)');
legend('实际液位','设定值','滤波测量','Location','best');
title('水箱液位响应');
grid on;

subplot(3,1,2);
plot(sim.time, u_cmd, 'LineWidth', 1.2); hold on;
plot(sim.time, u_act, '--', 'LineWidth', 1.2);
plot(sim.time, u_eff, ':', 'LineWidth', 1.2);
ylabel('阀门开度 (0-1)');
legend('控制器输出','执行器输出','有效开度','Location','best');
title('控制器与执行器');
grid on;

subplot(3,1,3);
yyaxis left;
plot(sim.time, u_eff.^2, 'Color',[0.85 0.33 0.1], 'LineWidth', 1);
ylabel('功率 proxy (u^2)');
yyaxis right;
plot(sim.time, qin_history, 'LineWidth', 1.2);
ylabel('进水流量 (m^3/s)');
xlabel('时间 (s)');
legend('u^2','进水流量','Location','best');
title('功率与随机扰动');
grid on;

%% 性能指标
final_setpoint = setpoint(end);
overshoot = max(h) - final_setpoint;
overshoot = max(overshoot, 0);
iae = trapz(sim.time, abs(h - setpoint));
valve_avg = mean(u_eff);
valve_rms = sqrt(mean(u_eff.^2));
control_energy = trapz(sim.time, u_eff.^2);

settling_tol = 0.01;
settling_time = sim.t_end;
for k = 1:N
    if all(abs(h(k:end) - setpoint(k:end)) < settling_tol)
        settling_time = sim.time(k);
        break;
    end
end

fprintf('仿真完成：最终液位 %.3f m，阀门有效开度 %.2f%%。\n', h(end), u_eff(end)*100);
fprintf('指标：超调 %.2f cm，调节时间 %.1f s，IAE %.3f，平均阀门 %.1f%%，能量 %.3f。\n', ...
    overshoot*100, settling_time, iae, valve_avg*100, control_energy);

sim_result = struct( ...
    'time', sim.time, ...
    'level', h, ...
    'setpoint', setpoint, ...
    'measurement', measurement_filt, ...
    'u_cmd', u_cmd, ...
    'u_act', u_act, ...
    'u_eff', u_eff, ...
    'qin', qin_history, ...
    'metrics', struct( ...
        'final_level', h(end), ...
        'overshoot', overshoot, ...
        'settling_time', settling_time, ...
        'iae', iae, ...
        'valve_avg', valve_avg, ...
        'valve_rms', valve_rms, ...
        'control_energy', control_energy));
assignin('base','tank_level_result',sim_result);
