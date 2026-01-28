function p = init_params()
    % init_params: 初始化四足机器人仿真参数
    
    %% 1. 机器人物理参数
    p.m = 12.0;            % 质量 (kg)
    % 惯性张量
    p.Ibody = diag([0.05, 0.1, 0.12]); 
    p.g = 9.81;            % 重力加速度
    
    % 腿部几何 (相对于质心: FL, FR, HL, HR)
    p.hip_offset = [ 0.19,  0.1, 0;    % Front Left
                     0.19, -0.1, 0;    % Front Right
                    -0.19,  0.1, 0;    % Hind Left
                    -0.19, -0.1, 0 ]'; % Hind Right 

    %% 2. 步态与仿真参数
    p.T_gait = 0.5;        % 步态周期 (s)
    p.dt_sim = 0.002;      % 物理仿真步长 (500Hz)
    p.dt_mpc = 0.03;       % MPC 控制步长 (33Hz)
    p.N = 10;              % 预测步数 (Horizon)
    
    %% 3. 物理约束
    p.mu = 0.6;            % 摩擦系数
    p.f_min = 0;           % 最小足端力
    p.f_max = 300;         % 最大足端力
    
    %% 4. 期望运动状态
    p.h_des = 0.3;         % 期望机身相对于地面的高度 (m)
    p.v_x_des = 0.4;       % 期望前进速度 (m/s)
    
    %% [新增] 5. 地形参数 (基于 V3.2 文档)
    p.slope_degree = 8.0;  % 坡度 (度)
    p.slope_rad = deg2rad(p.slope_degree); 
    p.slope_start_x = 1.0; % 斜坡起始位置 (m)

    %% 6. MPC 权重矩阵
    % 状态: [Theta(3), p(3), omega(3), v(3)]
    Q_diag = [20, 20, 50, ...     % 姿态 (Roll, Pitch, Yaw)
              50, 50, 500, ...    % 位置 (X, Y, Z) - Z权重较高以维持高度
              0.5, 0.5, 0.5, ...  % 角速度
              1, 1, 1];           % 线速度
    p.Q = diag(Q_diag);
    p.R = 1e-3 * eye(12);         % 控制输入权重
end