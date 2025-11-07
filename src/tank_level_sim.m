% tank_level_sim.m
% 轻量级单水箱液位控制仿真（PI 控制 + 阀门开度饱和）

%% 参数定义
rng(0);                     % 固定随机种子，保证可重复性
params.A = 0.5;             % 水箱截面积 (m^2)
params.kv = 0.7;            % 阀门流量系数
params.rho = 1000;          % 水密度 (kg/m^3)
params.g = 9.81;            % 重力加速度 (m/s^2)
params.q_in_base = 0.12;    % 基础进水流量 (m^3/s)

ctrl.Kp = 4.5;              % PI 控制器比例系数
ctrl.Ki = 0.25;             % PI 控制器积分系数
ctrl.u_min = 0.0;           % 阀门最小开度 (0-1)
ctrl.u_max = 1.0;           % 阀门最大开度 (0-1)

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
u = zeros(1, N);            % 阀门开度 (0-1)
integral_term = 0;

%% 主仿真循环
for k = 2:N
    t = sim.time(k);
    measurement = h(k-1) + noise_std * randn;     % 压力传感器读数
    error = measurement - setpoint(k);            % 液位高于设定值 -> 误差为正
    integral_term = integral_term + error * sim.dt;

    % PI 控制律：误差为正时增大阀门开度
    raw_u = ctrl.Kp * error + ctrl.Ki * integral_term;
    u(k) = min(max(raw_u, ctrl.u_min), ctrl.u_max);

    % 抗积分饱和：当饱和发生时不积累误差
    if raw_u ~= u(k)
        integral_term = integral_term - error * sim.dt;
    end

    q_in = params.q_in_base + disturbance(k);
    q_out = params.kv * u(k) * sqrt(max(h(k-1), 0));
    dh = (q_in - q_out) / params.A;
    h(k) = max(h(k-1) + dh * sim.dt, 0);
end

%% 可视化
figure('Name','Tank Level Control','Color','w','Position',[100 100 900 500]);
subplot(2,1,1);
plot(sim.time, h, 'LineWidth', 1.5); hold on;
plot(sim.time, setpoint, '--', 'LineWidth', 1.2);
ylabel('液位高度 (m)');
legend('实际液位','设定值','Location','best');
title('水箱液位响应');
grid on;

subplot(2,1,2);
plot(sim.time, u, 'LineWidth', 1.5); hold on;
yyaxis right;
plot(sim.time, params.q_in_base + disturbance, ':', 'LineWidth', 1.2);
ylabel('进水流量 (m^3/s)');
yyaxis left;
ylabel('阀门开度 (0-1)');
xlabel('时间 (s)');
legend('阀门开度','进水流量','Location','best');
title('执行器与扰动');
grid on;

fprintf('仿真完成：最终液位 %.3f m，阀门开度 %.2f%%。\n', h(end), u(end)*100);
