function p = init_params()
    % init_params: 初始化四足机器人仿真参数
    
    %% 1. 机器人物理参数
    p.m = 12.0;            % 质量 (kg)
    p.Ibody = diag([0.05, 0.1, 0.12]); 
    p.g = 9.81;            % 重力加速度
    
    %% 2. 地形参数 (波浪 Wave)
    % 类型开关: 'flat', 'slope', 'wave','wave_stairs' 
    p.terrain_type = 'wave_stairs';

    % --- 波浪参数 ---
    p.wave_amp = 0.1;     % 振幅  (波峰到波谷落差是振幅两倍)
    p.wave_freq = 3.0;     % 频率 (控制波长，值越大波越密)

    % --- 阶梯参数 ---
    p.stair_width = 0.20;  % 台阶宽度 (m)，即每 20cm 高度突变一次
                           % 这个宽度大约等于半个步长，很有挑战性

    % 地形参数(设为平地，把斜坡推的足够远) 
    p.slope_degree = 20.0;         % 地形坡度 (度)
    p.slope_rad = deg2rad(p.slope_degree); 
    p.slope_start_x = 1.0;        % 斜坡起点 (m)
    % 
    
    % 摆动参数
    p.h_swing = 0.12;             % 摆动高度 (10cm)

    % 腿部几何 (相对于质心: FL, FR, HL, HR)
    p.hip_offset = [ 0.19,  0.1, 0;    
                     0.19, -0.1, 0;    
                    -0.19,  0.1, 0;    
                    -0.19, -0.1, 0 ]'; 

    %% 3. 步态与仿真参数
    p.T_gait = 0.5;        % 步态周期 (s)
    p.dt_sim = 0.002;      % 物理仿真步长
    p.dt_mpc = 0.03;       % MPC 控制步长
    p.N = 10;              % 预测步数
    
    %% 4. 物理约束
    p.mu = 0.6;            % 摩擦系数
    p.f_min = 0;           % 最小足端力
    p.f_max = 300;         % 最大足端力
    
    %% 5. 期望运动状态
    p.h_des = 0.3;         % 期望高度 (m)
    p.v_x_des = 0.4;       % 期望前进速度 (m/s)
    
    %% 6. MPC 权重矩阵
    Q_diag = [20, 20, 50, ...     % 姿态
              50, 50, 500, ...    % 位置
              0.5, 0.5, 0.5, ...  % 角速度
              1, 1, 1];           % 线速度
    p.Q = diag(Q_diag);
    p.R = 1e-3 * eye(12);    

    %% 7. 感知与滤波参数 (动态自适应版)
    p.sensor_noise = 0.03;      % 模拟传感器噪声 3cm
    p.sensor_drop_prob = 0.1;   % 模拟 10% 丢包
    
    % 卡尔曼基准参数 (Base Parameters)
    % 这里设置一个比较保守的基准值，因为我们有动态调节机制
    p.kf_Q = 0.02^2;            % 过程噪声基准
    p.kf_R = 0.03^2;            % 测量噪声基准 (对应传感器 3cm)
end