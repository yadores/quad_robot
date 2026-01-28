function p = init_params()
    % init_params: 初始化四足机器人仿真参数
    
    %% 1. 机器人物理参数
    p.m = 12.0;            % 质量 (kg)
    p.Ibody = diag([0.05, 0.1, 0.12]); 
    p.g = 9.81;            % 重力加速度
    
    % 地形参数(设为平地，把斜坡推的足够远) 
    p.slope_degree = 0;         % 地形坡度 (度)
    p.slope_rad = deg2rad(p.slope_degree); 
    p.slope_start_x = 100.0;        % 斜坡起点 (m)
    % 
    
    % 摆动参数
    p.h_swing = 0.10;             % 摆动高度 (10cm)

    % 腿部几何 (相对于质心: FL, FR, HL, HR)
    p.hip_offset = [ 0.19,  0.1, 0;    
                     0.19, -0.1, 0;    
                    -0.19,  0.1, 0;    
                    -0.19, -0.1, 0 ]'; 

    %% 2. 步态与仿真参数
    p.T_gait = 0.5;        % 步态周期 (s)
    p.dt_sim = 0.002;      % 物理仿真步长
    p.dt_mpc = 0.03;       % MPC 控制步长
    p.N = 10;              % 预测步数
    
    %% 3. 物理约束
    p.mu = 0.6;            % 摩擦系数
    p.f_min = 0;           % 最小足端力
    p.f_max = 300;         % 最大足端力
    
    %% 4. 期望运动状态
    p.h_des = 0.3;         % 期望高度 (m)
    p.v_x_des = 0.4;       % 期望前进速度 (m/s)
    
    %% 5. MPC 权重矩阵
    Q_diag = [20, 20, 50, ...     % 姿态
              50, 50, 500, ...    % 位置
              0.5, 0.5, 0.5, ...  % 角速度
              1, 1, 1];           % 线速度
    p.Q = diag(Q_diag);
    p.R = 1e-3 * eye(12);         
end