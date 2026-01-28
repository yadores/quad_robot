function quadruped_trot_jump_angle_v2()
    % 四足机器人Trot步态混杂系统仿真（V2 - 修正稳态高度误差 + PID控制）
    
    %% 1. 参数定义（优化支撑性能）
    params = struct();
    params.m = 10;         % 机体质量 (kg)
    params.L1 = 0.08;      % 髋-膝连杆长度 (m)
    params.L2 = 0.07;      % 膝-脚连杆长度 (m)
    params.k = 10000;      % 地面刚度（增强支撑力）
    params.b = 600;        % 地面阻尼（减少震荡）
    params.mu = 0.6;       % 摩擦系数
    params.T = 0.4;        % 步态周期
    params.T_half = 0.2;   % 支撑相持续时间（占比50%）
    params.v = 0.1;        % 目标前进速度 (m/s)
    params.g = 9.81;       % 重力加速度
    params.ff_force_per_leg = (params.m * params.g) / 2.0; % Trot = 2 legs
    params.ff_delta_z = params.ff_force_per_leg / params.k; 
    
    % 物理约束参数
    params.ground_level = 0;      % 地面高度
    params.nominal_height = 0.14; % 标称高度 (m)
    params.hip_range = [-60, 60]; % 髋关节角度范围 (°)
    params.knee_range = [-30, 120];% 膝关节角度范围 (°)
    params.joint_damping = 3;     % 关节阻尼
    params.Kp = 100;              % 关节PD比例增益（增强跟踪）
    params.Kd = 10;               % 关节PD微分增益
    
    % 【新增】高度补偿 PID 增益
    params.Kp_height = 10.0;       % P增益
    params.Ki_height = 0.01;       % I增益
    params.Kd_height = 0.1;        % D增益 

    %% 2. 初始状态 - 12维状态向量
    q0 = zeros(13, 1);
    q0(1) = 0;                     % X初始位置 (m)
    q0(2) = params.nominal_height; % Z初始高度 (m)
    q0(3) = params.v;              % 初始前进速度 (m/s)
    q0(4) = 0;                     % 初始垂直速度 (m/s)
    
    % 初始关节角度 - 第一对角支撑相姿态
    % 传递 0 高度误差, 0 积分, 0 垂直速度
    [LF_hip_des, LF_knee_des] = get_desired_joint(1, true, 0, params, 0, 0, 0); 
    
    % 摆动腿初始化
    RF_hip_des = 0;     RF_knee_des = 19.1;
    LH_hip_des = 0;     LH_knee_des = 19.1;
    [RH_hip_des, RH_knee_des] = get_desired_joint(4, true, 0, params, 0, 0, 0);
    
    q0(5) = LF_hip_des;   q0(6) = LF_knee_des;  % 左前
    q0(7) = RF_hip_des;   q0(8) = RF_knee_des;  % 右前
    q0(9) = LH_hip_des;   q0(10) = LH_knee_des; % 左后
    q0(11) = RH_hip_des;  q0(12) = RH_knee_des; % 右后
    q0(13) = 0;           % 【新增】高度误差积分项的初始值

    %% 3. 仿真配置
    t_span = [0, 0.8];        % 仿真时间
    options = odeset('RelTol',1e-5, 'AbsTol',1e-5, 'MaxStep',0.001);

    %% 4. 运行仿真
    try
        [t, q] = ode45(@(t,q) trot_dynamics_corrected(t,q,params), t_span, q0, options);
    catch ME
        fprintf('仿真失败: %s\n', ME.message);
        fprintf('错误详情: %s\n', ME.getReport);
        return;
    end

    %% 5. 结果可视化
    visualize_trot_results_corrected(t, q, params);
end

%% 步态控制器 - 时间驱动对角交替
function [is_LF_support, is_RF_support, is_LH_support, is_RH_support] = gait_controller(t, params)
    t_in_cycle = mod(t, params.T);  % 周期内归一化时间
    
    if t_in_cycle < params.T_half
        % 第一对角支撑：左前+右后
        is_LF_support = true;
        is_RF_support = false;  
        is_LH_support = false;
        is_RH_support = true;
    else
        % 第二对角支撑：右前+左后
        is_LF_support = false;
        is_RF_support = true;
        is_LH_support = true;
        is_RH_support = false;
    end
end

%% 修正的Trot步态动力学模型
function dq = trot_dynamics_corrected(t, q, params)
    % 解析状态变量
    X = q(1); Z = q(2); dX = q(3); dZ = q(4);
    th_LF_hip = q(5); th_LF_knee = q(6);
    th_RF_hip = q(7); th_RF_knee = q(8);
    th_LH_hip = q(9); th_LH_knee = q(10);
    th_RH_hip = q(11); th_RH_knee = q(12);
    height_error_integral = q(13);

    % 获取支撑状态
    [is_LF_support, is_RF_support, is_LH_support, is_RH_support] = gait_controller(t, params);
    
    % 计算高度误差
    height_error = params.nominal_height - Z;
    
    % 关节控制 (传入 dZ 用于 D 控制)
    [dth_LF_hip, dth_LF_knee] = joint_control(th_LF_hip, th_LF_knee, 1, is_LF_support, t, params, height_error, height_error_integral, dZ);
    [dth_RF_hip, dth_RF_knee] = joint_control(th_RF_hip, th_RF_knee, 2, is_RF_support, t, params, height_error, height_error_integral, dZ);
    [dth_LH_hip, dth_LH_knee] = joint_control(th_LH_hip, th_LH_knee, 3, is_LH_support, t, params, height_error, height_error_integral, dZ);
    [dth_RH_hip, dth_RH_knee] = joint_control(th_RH_hip, th_RH_knee, 4, is_RH_support, t, params, height_error, height_error_integral, dZ);
    
    % 计算接触力
    F_LF = calc_leg_force_with_support(Z, dZ, th_LF_hip, th_LF_knee, is_LF_support, params);
    F_RF = calc_leg_force_with_support(Z, dZ, th_RF_hip, th_RF_knee, is_RF_support, params);
    F_LH = calc_leg_force_with_support(Z, dZ, th_LH_hip, th_LH_knee, is_LH_support, params);
    F_RH = calc_leg_force_with_support(Z, dZ, th_RH_hip, th_RH_knee, is_RH_support, params);
    
    % 质心动力学
    sum_Fx = F_LF(1) + F_RF(1) + F_LH(1) + F_RH(1);
    damping_coeff = 30;
    sum_Fx = sum_Fx - damping_coeff * (dX - params.v);
    
    sum_Fz = F_LF(2) + F_RF(2) + F_LH(2) + F_RH(2);
    
    ddX = sum_Fx / params.m;
    ddZ = (sum_Fz - params.m * params.g) / params.m;
    
    % 组装状态导数
    dq = zeros(13, 1);
    dq(1) = dX;      dq(2) = dZ;
    dq(3) = ddX;     dq(4) = ddZ;
    dq(5) = dth_LF_hip;  dq(6) = dth_LF_knee;
    dq(7) = dth_RF_hip;  dq(8) = dth_RF_knee;
    dq(9) = dth_LH_hip;  dq(10) = dth_LH_knee;
    dq(11) = dth_RH_hip; dq(12) = dth_RH_knee;
    dq(13) = height_error - 1.0 * height_error_integral;  % 【关键】带泄漏的积分器
end

%% 辅助函数：计算脚掌接触深度
function [delta_z, foot_z_absolute] = calc_foot_depth(Z, hip, knee, params)
    L1 = params.L1; L2 = params.L2;
    hip_rad = deg2rad(hip); knee_rad = deg2rad(knee);
    foot_z_relative = L1 * cos(hip_rad) + L2 * cos(hip_rad + knee_rad);
    foot_z_absolute = Z - foot_z_relative;
    delta_z = max(0, -foot_z_absolute);
end

%% 辅助函数：计算单腿接触力
function F = calc_leg_force_with_support(Z, dZ, hip, knee, is_support, params)
    F = [0, 0];
    if is_support
        [delta_z, ~] = calc_foot_depth(Z, hip, knee, params);
        if delta_z > 0
            Fz = params.k * delta_z - params.b * dZ;
            Fz = max(Fz, 0);
            Fx = params.mu * Fz;
            F = [Fx, Fz];
        end
    end
end

%% 辅助函数：关节PD控制
function dth = pd_control(th, th_des, params)
    error = th_des - th;
    dth = params.Kp * error;
    dth = clamp(dth, -250, 250);
end

%% 辅助函数：关节控制 wrapper
function [dth_hip, dth_knee] = joint_control(th_hip, th_knee, leg_id, is_support, t, params, height_error, height_error_integral, dZ)
    [hip_des, knee_des] = get_desired_joint(leg_id, is_support, t, params, height_error, height_error_integral, dZ);
    dth_hip = pd_control(th_hip, hip_des, params);
    dth_knee = pd_control(th_knee, knee_des, params);
end

%% 【核心】获取关节期望角度 (含 PID 高度控制)
function [hip_des, knee_des] = get_desired_joint(leg_id, is_support, t, params, height_error, height_error_integral, dZ)
    t_in_cycle = mod(t, params.T);
    phase = t_in_cycle / params.T;
    
    if leg_id == 1 || leg_id == 4; phase_offset = 0; else; phase_offset = 0.5; end
    phase_adj = mod(phase + phase_offset, 1.0);
    
    if is_support
        % --- 支撑相 ---
        hip_swing = 3 * sin(2 * pi * phase_adj);
        hip_des = hip_swing;
        
        % 基础前馈
        equilibrium_length = params.nominal_height + params.ff_delta_z*1.05;
        
        % PID 控制
        P_term = params.Kp_height * height_error;
        I_term = params.Ki_height * height_error_integral;
        D_term = params.Kd_height * (-dZ); % D项
        
        % Anti-Windup
        max_I = 0.005; 
        I_term = clamp(I_term, -max_I, max_I);
        
        desired_leg_length = equilibrium_length + P_term + I_term + D_term;
            
        % IK
        L1 = params.L1; L2 = params.L2;
        hip_rad = deg2rad(hip_des);
        cos_arg = (desired_leg_length - L1 * cos(hip_rad)) / L2;
        cos_arg = clamp(cos_arg, -0.98, 0.98);
        
        hip_plus_knee_rad = acos(cos_arg);
        knee_rad = hip_plus_knee_rad - hip_rad;
        knee_des = rad2deg(knee_rad);
        
    else
        % --- 摆动相 ---
        swing_phase = (phase_adj - 0.5) / 0.5;
        if swing_phase < 0.2
            lift_ratio = swing_phase / 0.2;
            hip_des = 25 * lift_ratio;
            knee_des = 80 + 40 * lift_ratio;
        elseif swing_phase < 0.5
            swing_ratio = (swing_phase - 0.2) / 0.3;
            hip_des = 25 + 15 * swing_ratio;
            knee_des = 120 - 20 * swing_ratio;
        else
            prepare_ratio = (swing_phase - 0.5) / 0.5;
            hip_start = 40; hip_end = 0;
            knee_start = 100; knee_end = 19.1;
            hip_des = hip_start + (hip_end - hip_start) * prepare_ratio;
            knee_des = knee_start + (knee_end - knee_start) * prepare_ratio;
        end
    end
    
    hip_des = clamp(hip_des, params.hip_range(1), params.hip_range(2));
    knee_des = clamp(knee_des, params.knee_range(1), params.knee_range(2));
end

%% 完整可视化函数
function visualize_trot_results_corrected(t, q, params)
    figure('Name','Trot步态仿真结果（V2-高度修正版）','Position',[100,100,1200,900]);
    
    [F_LF, F_RF, F_LH, F_RH, F_total, support_pattern, ...
     th_LF_hip, th_RF_hip, th_LH_hip, th_RH_hip, ...
     th_LF_knee, th_RF_knee, th_LH_knee, th_RH_knee] = compute_all_data_corrected(t, q, params);
    
    % 1. 接触力
    subplot(3,3,1);
    plot(t, F_LF, 'g-'); hold on; plot(t, F_RF, 'r-');
    plot(t, F_LH, 'b-'); plot(t, F_RH, 'm-');
    yline(50, 'k--'); ylabel('接触力 (N)'); title('各腿垂直接触力');
    legend('LF', 'RF', 'LH', 'RH'); grid on; ylim([0, 150]);
    
    % 2. 机身高度
    subplot(3,3,2);
    plot(t, q(:,2), 'k-', 'LineWidth',2);
    yline(params.nominal_height, 'r--'); ylabel('高度 (m)'); title('机身高度变化');
    grid on; ylim([0.13, 0.15]);
    
    % 3. 总力
    subplot(3,3,3);
    plot(t, F_total, 'm-', 'LineWidth',2); hold on;
    yline(98.1, 'k--'); ylabel('总力 (N)'); title('总支撑力 vs 重力');
    grid on; ylim([80, 120]);
    
    % 4. 支撑模式
    subplot(3,3,4);
    imagesc(t, 1:4, support_pattern);
    colormap([1 1 1; 0.7 0.9 0.7]);
    ylabel('腿编号'); xlabel('时间 (s)'); title('支撑模式');
    yticks(1:4); yticklabels({'LF', 'RF', 'LH', 'RH'});
    
    % 5. 对角力
    subplot(3,3,5);
    plot(t, F_LF+F_RH, 'g-', 'LineWidth',2); hold on;
    plot(t, F_RF+F_LH, 'r-', 'LineWidth',2);
    yline(98.1, 'k--'); ylabel('力 (N)'); title('对角腿合力');
    legend('Diag1', 'Diag2'); grid on; ylim([0, 150]);
    
    % 6. 速度
    subplot(3,3,6);
    plot(t, q(:,3), 'b-'); hold on; plot(t, q(:,4), 'r-');
    yline(params.v, 'g--'); ylabel('速度 (m/s)'); title('机身速度');
    legend('Vx', 'Vz'); grid on;
    
    % 7. 左前腿关节
    subplot(3,3,7);
    plot(t, th_LF_hip, 'g-'); hold on; plot(t, th_LF_knee, 'g--');
    ylabel('角度 (°)'); title('左前腿关节'); legend('Hip', 'Knee'); grid on;
    
    % 8. 右前腿关节
    subplot(3,3,8);
    plot(t, th_RF_hip, 'r-'); hold on; plot(t, th_RF_knee, 'r--');
    ylabel('角度 (°)'); title('右前腿关节'); legend('Hip', 'Knee'); grid on;
    
    % 9. 文字分析
    analyze_trot_performance_corrected(t, F_LF, F_RF, F_LH, F_RH, F_total, q, params);
end

%% 数据重算函数
function [F_LF, F_RF, F_LH, F_RH, F_total, support_pattern, ...
          th_LF_hip, th_RF_hip, th_LH_hip, th_RH_hip, ...
          th_LF_knee, th_RF_knee, th_LH_knee, th_RH_knee] = compute_all_data_corrected(t, q, params)
    
    n = length(t);
    F_LF = zeros(n,1); F_RF = zeros(n,1); F_LH = zeros(n,1); F_RH = zeros(n,1);
    support_pattern = zeros(4, n);
    th_LF_hip = zeros(n,1); th_RF_hip = zeros(n,1); th_LH_hip = zeros(n,1); th_RH_hip = zeros(n,1);
    th_LF_knee = zeros(n,1); th_RF_knee = zeros(n,1); th_LH_knee = zeros(n,1); th_RH_knee = zeros(n,1);
    
    for i = 1:n
        Z = q(i,2); dZ = q(i,4);
        
        th_LF_hip(i) = q(i,5); th_LF_knee(i) = q(i,6);
        th_RF_hip(i) = q(i,7); th_RF_knee(i) = q(i,8);
        th_LH_hip(i) = q(i,9); th_LH_knee(i) = q(i,10);
        th_RH_hip(i) = q(i,11); th_RH_knee(i) = q(i,12);
        
        [is_LF, is_RF, is_LH, is_RH] = gait_controller(t(i), params);
        support_pattern(1,i) = is_LF; support_pattern(2,i) = is_RF;
        support_pattern(3,i) = is_LH; support_pattern(4,i) = is_RH;
        
        f_LF = calc_leg_force_with_support(Z, dZ, th_LF_hip(i), th_LF_knee(i), is_LF, params);
        f_RF = calc_leg_force_with_support(Z, dZ, th_RF_hip(i), th_RF_knee(i), is_RF, params);
        f_LH = calc_leg_force_with_support(Z, dZ, th_LH_hip(i), th_LH_knee(i), is_LH, params);
        f_RH = calc_leg_force_with_support(Z, dZ, th_RH_hip(i), th_RH_knee(i), is_RH, params);
        
        F_LF(i) = f_LF(2); F_RF(i) = f_RF(2);
        F_LH(i) = f_LH(2); F_RH(i) = f_RH(2);
    end
    F_total = F_LF + F_RF + F_LH + F_RH;
end

%% 性能分析文字输出
function analyze_trot_performance_corrected(t, F_LF, F_RF, F_LH, F_RH, F_total, q, params)
    fprintf('=== Trot步态性能分析 (V2) ===\n');
    
    avg_total_force = mean(F_total);
    fprintf('总支撑力分析:\n');
    fprintf('  平均总支撑力: %.1f N\n', avg_total_force);
    fprintf('  机体重力: %.1f N\n', params.m * params.g);
    
    height_std = std(q(:,2));
    fprintf('机身高度稳定性:\n');
    fprintf('  高度均值: %.4f m (目标: %.4f m)\n', mean(q(:,2)), params.nominal_height);
    fprintf('  高度标准差: %.4f m\n', height_std);
    fprintf('======================\n');
end

function y = clamp(x, min_val, max_val)
    y = max(min(x, max_val), min_val);
end