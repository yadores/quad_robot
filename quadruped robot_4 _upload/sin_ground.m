% sin_ground.m
% 通用地形预览工具 (已升级：完全同步 init_params 配置)
clc; clear; close all;

%% 1. 自动加载参数
% 直接读取配置文件，确保和仿真一致
p = init_params();

fprintf('================================================\n');
fprintf('  当前检测到的地形模式: [%s]\n', p.terrain_type);
fprintf('  振幅: %.2fm | 频率: %.1f | 阶梯宽: %.2fm\n', p.wave_amp, p.wave_freq, p.stair_width);
fprintf('================================================\n');

%% 2. 生成采样点
% 采样稍微密集一点，以便画出平滑曲线或锐利台阶
x = 0:0.005:4.0; 
z_actual = zeros(size(x));

% --- 核心修改：调用统一的接口 ---
% 不再在脚本里手写公式，而是直接问 get_terrain_z "这里高度是多少？"
% 这样无论你是 flat, slope, wave 还是 wave_stairs，这里都能算对
for i = 1:length(x)
    z_actual(i) = get_terrain_z(x(i), 0, p); % 假设在中心线 y=0
end

%% 3. 绘图分析
figure('Color', 'w', 'Position', [100, 100, 1000, 600]);

% --- 子图 1: 侧视图 (剖面) ---
subplot(2, 1, 1); hold on; grid on; box on;

% 【智能辅助线】
% 只有在 "阶梯波浪" 模式下，才额外画一条绿色的虚线(原始正弦波)作为对比
if strcmp(p.terrain_type, 'wave_stairs')
    z_ref = p.wave_amp * sin(p.wave_freq * (x - 0.5));
    z_ref(x<=0.5) = 0;
    plot(x, z_ref, 'g--', 'LineWidth', 1, 'DisplayName', '原始波浪参考');
end

% 画实际地形 (这是机器人真正踩到的地面)
plot(x, z_actual, 'b-', 'LineWidth', 2, 'DisplayName', ['实际地形: ' p.terrain_type]);

% 填充颜色，增加视觉实感
area(x, z_actual, 'FaceColor', [0.8 0.9 1], 'EdgeColor', 'none', 'FaceAlpha', 0.5);

% 动态调整坐标轴范围
y_max = max(abs(z_actual));
if y_max == 0, y_max = 0.1; end % 防止平地时坐标轴太扁
ylim([-y_max*1.5, y_max*1.5]);

xlabel('前进距离 X (m)');
ylabel('高度 Z (m)');
title(sprintf('地形剖面预览 (模式: %s)', p.terrain_type));
legend('Location', 'best');

% 画一个虚拟的足端摆动轨迹 (示意图)
% 仅在非平地模式下显示，帮你判断步长是否合适
if ~strcmp(p.terrain_type, 'flat')
    step_start = 1.2;
    step_len = 0.4;
    traj_x = linspace(step_start, step_start+step_len, 50);
    s = linspace(0, 1, 50);
    % 获取起步点的地形高度
    z_ground_start = get_terrain_z(step_start, 0, p);
    % 简单的正弦摆动示意
    traj_z = z_ground_start + p.h_swing * sin(pi * s); 
    plot(traj_x, traj_z, 'r--', 'LineWidth', 1.5, 'DisplayName', '足端摆动示意');
end

% --- 子图 2: 3D 效果预览 ---
subplot(2, 1, 2); 
[X, Y] = meshgrid(0:0.05:4.0, -0.5:0.1:0.5);
Z = zeros(size(X));

% 快速计算全场高度
for i = 1:numel(X)
    Z(i) = get_terrain_z(X(i), Y(i), p);
end

surf(X, Y, Z, 'EdgeColor', 'none', 'FaceAlpha', 0.8);
colormap turbo; 
light; lighting gouraud;
axis equal; view(45, 30);
xlabel('X'); ylabel('Y'); zlabel('Z');
title(['3D 地形预览: ' p.terrain_type]);