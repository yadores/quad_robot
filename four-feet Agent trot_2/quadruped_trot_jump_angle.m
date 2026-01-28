function quadruped_trot_jump_angle()
    % 四足机器人Trot步态混杂系统仿真（V2 - 修正稳态高度误差）
    
    %% 1. 参数定义（优化支撑性能）
    params = struct();
    params.m = 10;         % 机体质量 (kg)
    params.L1 = 0.08;      % 髋-膝连杆长度 (m)
    params.L2 = 0.07;      % 膝-脚连杆长度 (m)
    params.k =10000;      % 地面刚度（增强支撑力）
    params.b = 600;        % 地面阻尼（减少震荡）
    params.mu = 0.6;       % 摩擦系数
    params.T = 0.4;        % 步态周期
    % 【修改】移除 T_half，改用 duty_factor > 0.5
    params.duty_factor = 0.65;  % 65% 支撑时间 (0.15T 重叠)
    params.v = 0.1;        % 目标前进速度 (m/s)
    params.g = 9.81;       % 重力加速度
    params.ff_force_per_leg = (params.m * params.g) / 2.0; % Trot = 2 legs
    params.ff_delta_z = params.ff_force_per_leg / params.k; % (0.00613m)
    
    % 物理约束参数
    params.ground_level = 0;      % 地面高度
    params.nominal_height = 0.14; % 标称高度 (m)
    params.hip_range = [-60, 60]; % 髋关节角度范围 (°)
    params.knee_range = [-30, 120];% 膝关节角度范围 (°)
    params.joint_damping = 3;     % 关节阻尼
    params.Kp = 100;              % 关节PD比例增益（增强跟踪）
    params.Kd = 10;               % 关节PD微分增益
    
    % 【新增】高度补偿P增益
    params.Kp_height = 10.0;       % (可调 )
    params.Ki_height = 0.01;       % 【新增】高度补偿I增益
    params.Kd_height = 0.1;       % 【新增】高度补偿D增益 

    %% 2. 初始状态 - 12维状态向量
    q0 = zeros(13, 1);
    q0(1) = 0;                % X初始位置 (m)
    q0(2) = params.nominal_height;  % Z初始高度 (m)
    q0(3) = params.v;         % 初始前进速度 (m/s)
    q0(4) = 0;                % 初始垂直速度 (m/s)
    
    % 初始关节角度 - 第一对角支撑相姿态
    % 传递 0 高度误差
    [LF_hip_des, LF_knee_des] = get_desired_joint(1, true, 0, params, 0,0,0); 
    % 【核心修复】在t=0时，摆动腿(RF+LH)应处于“即将着陆”状态，
    % 而不是 刚刚抬起 状态。
    RF_hip_des = 0;     RF_knee_des = 19.1;
    LH_hip_des = 0;     LH_knee_des = 19.1;
    [RH_hip_des, RH_knee_des] = get_desired_joint(4, true, 0, params, 0,0,0);
    
    q0(5) = LF_hip_des;   q0(6) = LF_knee_des;  % 左前腿（髋/膝）
    q0(7) = RF_hip_des;   q0(8) = RF_knee_des;  % 右前腿（髋/膝）
    q0(9) = LH_hip_des;   q0(10) = LH_knee_des; % 左后腿（髋/膝）
    q0(11) = RH_hip_des;  q0(12) = RH_knee_des; % 右后腿（髋/膝）
    q0(13) = 0;               % 【新增】高度误差积分项的初始值

    %% 3. 仿真配置
    t_span = [0, 0.8];        % 仿真时间（2个步态周期）
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
    %visualize_trot_results_corrected(t, q, params);
    % 【新增】启动 3D 动画播放
    visualize_2d(t, q, params);
end

%% 【修改】步态控制器 - 支持重叠相
function [is_LF_support, is_RF_support, is_LH_support, is_RH_support] = gait_controller(t, params)
    % 归一化时间 [0, 1)
    tau = mod(t, params.T) / params.T;
    beta = params.duty_factor;
    
    % 第一组对角 (LF + RH): 0 ~ beta
    is_diag1 = (tau < beta);
    
    % 第二组对角 (RF + LH): 0.5 ~ 0.5+beta (需处理周期循环)
    tau_2 = mod(tau + 0.5, 1.0);
    is_diag2 = (tau_2 < beta);
    
    % 分配状态 (重叠期两者都为 true)
    is_LF_support = is_diag1; is_RH_support = is_diag1;
    is_RF_support = is_diag2; is_LH_support = is_diag2;
end

%% 修正的Trot步态动力学模型
function dq = trot_dynamics_corrected(t, q, params)
    % 解析状态变量
    X = q(1); Z = q(2); dX = q(3); dZ = q(4);  % 机身位置与速度
    th_LF_hip = q(5); th_LF_knee = q(6);       % 左前腿关节角度
    th_RF_hip = q(7); th_RF_knee = q(8);       % 右前腿关节角度
    th_LH_hip = q(9); th_LH_knee = q(10);      % 左后腿关节角度
    th_RH_hip = q(11); th_RH_knee = q(12);     % 右后腿关节角度
    height_error_integral = q(13);            % 【新增】获取当前的积分状态

    % 获取支撑状态
    [is_LF_support, is_RF_support, is_LH_support, is_RH_support] = gait_controller(t, params);
    
    % 【修改点 1】计算高度误差 (允许正负误差)
    height_error = params.nominal_height - Z;  % 正值表示机身偏低
    
    % 【修改点 2】关节控制提前，并传入 height_error
    [dth_LF_hip, dth_LF_knee] = joint_control(th_LF_hip, th_LF_knee, 1, is_LF_support, t, params, height_error, height_error_integral, dZ);
    [dth_RF_hip, dth_RF_knee] = joint_control(th_RF_hip, th_RF_knee, 2, is_RF_support, t, params, height_error, height_error_integral, dZ);
    [dth_LH_hip, dth_LH_knee] = joint_control(th_LH_hip, th_LH_knee, 3, is_LH_support, t, params, height_error, height_error_integral, dZ);
    [dth_RH_hip, dth_RH_knee] = joint_control(th_RH_hip, th_RH_knee, 4, is_RH_support, t, params, height_error, height_error_integral, dZ);
    
    % 计算四条腿的接触力
    F_LF = calc_leg_force_with_support(Z, dZ, th_LF_hip, th_LF_knee, is_LF_support, params);
    F_RF = calc_leg_force_with_support(Z, dZ, th_RF_hip, th_RF_knee, is_RF_support, params);
    F_LH = calc_leg_force_with_support(Z, dZ, th_LH_hip, th_LH_knee, is_LH_support, params);
    F_RH = calc_leg_force_with_support(Z, dZ, th_RH_hip, th_RH_knee, is_RH_support, params);
    
    % 质心动力学（含水平阻尼）
    sum_Fx = F_LF(1) + F_RF(1) + F_LH(1) + F_RH(1);  % 总水平力
    damping_coeff = 30;  % 水平阻尼系数
    sum_Fx = sum_Fx - damping_coeff * (dX - params.v);  % 速度跟踪
    
    sum_Fz = F_LF(2) + F_RF(2) + F_LH(2) + F_RH(2);  % 总垂直力
    
    % 加速度计算（牛顿第二定律）
    ddX = sum_Fx / params.m;
    ddZ = (sum_Fz - params.m * params.g) / params.m;
    
    % 组装状态导数
    dq = zeros(13, 1);
    dq(1) = dX;      dq(2) = dZ;         % 位置导数=速度
    dq(3) = ddX;     dq(4) = ddZ;        % 速度导数=加速度
    dq(5) = dth_LF_hip;  dq(6) = dth_LF_knee;  % 左前腿关节角速度
    dq(7) = dth_RF_hip;  dq(8) = dth_RF_knee;  % 右前腿关节角速度
    dq(9) = dth_LH_hip;  dq(10) = dth_LH_knee; % 左后腿关节角速度
    dq(11) = dth_RH_hip; dq(12) = dth_RH_knee; % 右后腿关节角速度
    dq(13) = height_error - 1.0 * height_error_integral;  % 【新增】积分项的导数就是误差本身
end

%% 辅助函数：计算脚掌接触深度（修正运动学）
function [delta_z, foot_z_absolute] = calc_foot_depth(Z, hip, knee, params)
    % 连杆长度与角度转弧度
    L1 = params.L1;
    L2 = params.L2;
    hip_rad = deg2rad(hip);
    knee_rad = deg2rad(knee);
    
    % 脚掌相对机身的垂直位置（向下为正）
    foot_z_relative = L1 * cos(hip_rad) + L2 * cos(hip_rad + knee_rad);
    
    % 脚掌绝对高度（世界坐标系）
    foot_z_absolute = Z - foot_z_relative;  % 机身高度减去向下伸出的距离
    
    % 接触深度（脚掌低于地面时为正）
    delta_z = max(0, -foot_z_absolute);  % 地面高度0 - 脚掌高度（穿透为正）
end

%% 辅助函数：基于支撑状态计算单腿接触力
function F = calc_leg_force_with_support(Z, dZ, hip, knee, is_support, params)
    F = [0, 0];  % 默认零力（摆动相）
    
    if is_support
        % 支撑相计算接触力
        [delta_z, ~] = calc_foot_depth(Z, hip, knee, params);
        if delta_z > 0  % 仅当脚掌穿透地面时
            % 弹簧-阻尼力模型
            Fz = params.k * delta_z - params.b * dZ;
            Fz = max(Fz, 0);  % 避免地面提供拉力
            Fx = params.mu * Fz;  % 水平摩擦力（推进力）
            F = [Fx, Fz];
        end
    end
end

%% 辅助函数：关节PD控制
function dth = pd_control(th, th_des, params)
    error = th_des - th;  % 角度误差
    dth = params.Kp * error;  % P控制律 (Kd项是错误的，必须移除)
    dth = clamp(dth, -250, 250);  % 角速度限幅（°/s）
end

%% 【V2 修改】辅助函数：关节控制（传递高度误差）
function [dth_hip, dth_knee] = joint_control(th_hip, th_knee, leg_id, is_support, t, params, height_error, height_error_integral, dZ)
    % 获取期望角度 (将 height_error 传递下去)
    [hip_des, knee_des] = get_desired_joint(leg_id, is_support, t, params, height_error, height_error_integral, dZ);
    
    
    % 计算关节角速度
    dth_hip = pd_control(th_hip, hip_des, params);
    dth_knee = pd_control(th_knee, knee_des, params);
end

%% 【V2 核心修改】辅助函数：获取关节期望角度（基于IK和高度补偿）
function [hip_des, knee_des] = get_desired_joint(leg_id, is_support, t, params, height_error, height_error_integral,dZ) % <-- 增加输入
    t_in_cycle = mod(t, params.T);
    phase = t_in_cycle / params.T;  % 归一化相位 [0,1)
    
    % 对角腿相位偏移（确保同步）
    if leg_id == 1 || leg_id == 4  % 第一对角（左前+右后）
        phase_offset = 0;
    else  % 第二对角（右前+左后）
        phase_offset = 0.5;
    end
    phase_adj = mod(phase + phase_offset, 1.0);
    
    if is_support
        % 支撑相：【核心修改】
        
        % 1. 期望的髋关节（不变）
        hip_swing = 3 * sin(2 * pi * phase_adj);  % 髋关节小幅摆动
        hip_des = hip_swing;
        
        % 2. 计算动态期望腿长
        % 标称高度 + 平衡重力所需的前馈压缩量
        equilibrium_length = params.nominal_height + params.ff_delta_z*1.05; 

        % 3. PID控制器补偿腿长
        P_term = params.Kp_height * height_error;            % P项 (比例)
        I_term = params.Ki_height * height_error_integral;   % I项 (积分)
        
        % 【新增】D项 (微分)
        % 误差的导数 = (目标高度导数) - (当前高度导数) = 0 - dZ
        D_term = params.Kd_height * (-dZ); 

        % 【新增】Anti-Windup (反积分饱和)
        max_I_contribution = 0.005; 
        I_term = clamp(I_term, -max_I_contribution, max_I_contribution);
        
        % 最终腿长 = 物理平衡点 + PID修正
        desired_leg_length = equilibrium_length + P_term + I_term + D_term; % <-- 加上 D_term
            
        % 4. 逆运动学：根据 desired_leg_length 和 hip_des 反解 knee_des
        % 运动学公式: L1*cos(hip) + L2*cos(hip+knee) = LegLength
        
        L1 = params.L1;
        L2 = params.L2;
        hip_rad = deg2rad(hip_des);
            
        % cos(hip+knee) = (LegLength - L1*cos(hip)) / L2
        cos_arg = (desired_leg_length - L1 * cos(hip_rad)) / L2;
            
        % 约束cos_arg在[-1, 1]之间, 防止acos计算出错 (留点余量)
        cos_arg = clamp(cos_arg, -0.98, 0.98); 
            
        hip_plus_knee_rad = acos(cos_arg);
            
        knee_rad = hip_plus_knee_rad - hip_rad;
        knee_des = rad2deg(knee_rad);
        
    else
    % 摆动相：【逻辑修正】
    % 摆动相的 phase_adj 范围是 [0.5, 1.0]
    % 需要把这个范围重新映射到 [0, 1] 来规划轨迹
    swing_phase = (phase_adj - 0.5) / 0.5; % 映射 [0.5, 1.0] -> [0, 1.0]

    if swing_phase < 0.2  % 抬腿阶段 (映射后 0 -> 0.2)
        lift_ratio = swing_phase / 0.2;
        hip_des = 25 * lift_ratio;  % 向前摆腿
        knee_des = 80 + 40 * lift_ratio;  % 弯曲抬高
    elseif swing_phase < 0.5  % 前摆阶段 (映射后 0.2 -> 0.5)
        swing_ratio = (swing_phase - 0.2) / 0.3;
        hip_des = 25 + 15 * swing_ratio;
        knee_des = 120 - 20 * swing_ratio;% 摆动时，让腿稍微弯曲
    else  % 落地准备阶段 (映射后 0.5 -> 1.0)
    prepare_ratio = (swing_phase - 0.5) / 0.5;

    % 【核心修复】 
    % 旧目标 (hip=5, knee=30) 导致腿长0.137m (在空中)
    % 新目标 (hip=0, knee=31) 导致腿长0.140m (准备触地)

    hip_start = 40;     hip_end = 0;
    knee_start = 100;   knee_end = 19.1; % 31度是IK反解的0.14m腿长

    hip_des = hip_start + (hip_end - hip_start) * prepare_ratio;
    knee_des = knee_start + (knee_end - knee_start) * prepare_ratio;  % 缓慢放下
end
end
    
    % 关节角度限位
    hip_des = clamp(hip_des, params.hip_range(1), params.hip_range(2));
    knee_des = clamp(knee_des, params.knee_range(1), params.knee_range(2));
end

%% 完整可视化函数
function visualize_trot_results_corrected(t, q, params)
    figure('Name','Trot步态仿真结果（V2-高度修正版）','Position',[100,100,1200,900]);
    
    % 计算所有分析数据
    [F_LF, F_RF, F_LH, F_RH, F_total, support_pattern, ...
     th_LF_hip, th_RF_hip, th_LH_hip, th_RH_hip, ...
     th_LF_knee, th_RF_knee, th_LH_knee, th_RH_knee] = compute_all_data_corrected(t, q, params);
    
    % 子图1：各腿接触力
    subplot(3,3,1);
    plot(t, F_LF, 'g-', 'LineWidth',1.5); hold on;
    plot(t, F_RF, 'r-', 'LineWidth',1.5);
    plot(t, F_LH, 'b-', 'LineWidth',1.5);
    plot(t, F_RH, 'm-', 'LineWidth',1.5);
    yline(50, 'k--', '目标力50N', 'LineWidth',1);
    ylabel('接触力 (N)'); title('各腿垂直接触力');
    legend('左前腿', '右前腿', '左后腿', '右后腿', '目标力', 'Location','best');
    grid on; ylim([0, 150]);
    
    % 子图2：机身高度
    subplot(3,3,2);
    plot(t, q(:,2), 'k-', 'LineWidth',2);
    yline(params.nominal_height, 'r--', '标称高度', 'LineWidth',1);
    xlabel('时间 (s)'); ylabel('机身高度 (m)'); 
    title('机身高度变化（应稳定在0.14m）');
    grid on; ylim([0.13, 0.15]); % 保持Y轴范围以便对比
    
    % 子图3：总支撑力与重力平衡
    subplot(3,3,3);
    plot(t, F_total, 'm-', 'LineWidth',2); hold on;
    yline(98.1, 'k--', '重力98.1N', 'LineWidth',2);
    xlabel('时间 (s)'); ylabel('总支撑力 (N)'); 
    title('总支撑力 vs 重力');
    grid on; ylim([80, 120]);
    legend('总支撑力', '重力', 'Location','best');
    
    % 子图4：支撑模式
    subplot(3,3,4);
    imagesc(t, 1:4, support_pattern);
    colormap([1 1 1; 0.7 0.9 0.7]); % 白色=摆动，绿色=支撑
    ylabel('腿编号'); xlabel('时间 (s)'); title('支撑模式（对角交替）');
    yticks(1:4); yticklabels({'左前', '右前', '左后', '右后'});
    
    % 子图5：对角腿合力
    subplot(3,3,5);
    F_diag1 = F_LF + F_RH;  % 第一对角
    F_diag2 = F_RF + F_LH;  % 第二对角
    plot(t, F_diag1, 'g-', 'LineWidth',2); hold on;
    plot(t, F_diag2, 'r-', 'LineWidth',2);
    yline(98.1, 'k--', '重力98.1N', 'LineWidth',1);
    xlabel('时间 (s)'); ylabel('对角腿合力 (N)'); 
    title('对角腿支撑力交替');
    legend('对角1(左前+右后)', '对角2(右前+左后)', '重力', 'Location','best');
    grid on; ylim([0, 150]);
    
    % 子图6：机身速度
    subplot(3,3,6);
    plot(t, q(:,3), 'b-', 'LineWidth',1.5); hold on;
    plot(t, q(:,4), 'r-', 'LineWidth',1.5);
    yline(params.v, 'g--', '目标速度', 'LineWidth',1);
    xlabel('时间 (s)'); ylabel('速度 (m/s)'); 
    title('机身速度');
    legend('水平速度', '垂直速度', '目标速度', 'Location','best');
    grid on;
    
    % 只保留左右前腿关节角度图
    leg_data = {th_LF_hip, th_LF_knee;  % 左前腿
                th_RF_hip, th_RF_knee};  % 右前腿
    
    leg_names = {'左前腿', '右前腿'};
    leg_colors = ['g', 'r'];
    
    for i = 1:2
        subplot(3,3,6+i);
        hip_data = leg_data{i,1};
        knee_data = leg_data{i,2};
        
        plot(t, hip_data, [leg_colors(i) '-'], 'LineWidth',1.5); hold on;
        plot(t, knee_data, [leg_colors(i) '--'], 'LineWidth',1.5);
        
        % 关节角度限位线
        yline(params.hip_range(1), 'k:', 'LineWidth',0.5);
        yline(params.hip_range(2), 'k:', 'LineWidth',0.5);
        yline(params.knee_range(1), 'k:', 'LineWidth',0.5);
        yline(params.knee_range(2), 'k:', 'LineWidth',0.5);
        
        xlabel('时间 (s)'); ylabel('角度 (°)'); 
        title([leg_names{i} '关节角度']);
        legend('髋关节', '膝关节', 'Location','best');
        grid on;
    end
    
    % 输出性能分析
    analyze_trot_performance_corrected(t, F_LF, F_RF, F_LH, F_RH, F_total, q, params);
end

%% 完整数据计算函数
function [F_LF, F_RF, F_LH, F_RH, F_total, support_pattern, ...
          th_LF_hip, th_RF_hip, th_LH_hip, th_RH_hip, ...
          th_LF_knee, th_RF_knee, th_LH_knee, th_RH_knee] = compute_all_data_corrected(t, q, params)
    
    n = length(t);
    % 初始化存储变量
    F_LF = zeros(n,1); F_RF = zeros(n,1); F_LH = zeros(n,1); F_RH = zeros(n,1);
    support_pattern = zeros(4, n);
    th_LF_hip = zeros(n,1); th_RF_hip = zeros(n,1); th_LH_hip = zeros(n,1); th_RH_hip = zeros(n,1);
    th_LF_knee = zeros(n,1); th_RF_knee = zeros(n,1); th_LH_knee = zeros(n,1); th_RH_knee = zeros(n,1);
    
    for i = 1:n
        Z = q(i,2); dZ = q(i,4);
        
        % 提取关节角度
        th_LF_hip(i) = q(i,5); th_LF_knee(i) = q(i,6);
        th_RF_hip(i) = q(i,7); th_RF_knee(i) = q(i,8);
        th_LH_hip(i) = q(i,9); th_LH_knee(i) = q(i,10);
        th_RH_hip(i) = q(i,11); th_RH_knee(i) = q(i,12);
        
        % 获取支撑状态
        [is_LF_support, is_RF_support, is_LH_support, is_RH_support] = gait_controller(t(i), params);
        support_pattern(1,i) = is_LF_support; support_pattern(2,i) = is_RF_support;
        support_pattern(3,i) = is_LH_support; support_pattern(4,i) = is_RH_support;
        
        % 计算接触力（取垂直分量）
        force_LF = calc_leg_force_with_support(Z, dZ, th_LF_hip(i), th_LF_knee(i), is_LF_support, params);
        force_RF = calc_leg_force_with_support(Z, dZ, th_RF_hip(i), th_RF_knee(i), is_RF_support, params);
        force_LH = calc_leg_force_with_support(Z, dZ, th_LH_hip(i), th_LH_knee(i), is_LH_support, params);
        force_RH = calc_leg_force_with_support(Z, dZ, th_RH_hip(i), th_RH_knee(i), is_RH_support, params);
        
        F_LF(i) = force_LF(2); F_RF(i) = force_RF(2);
        F_LH(i) = force_LH(2); F_RH(i) = force_RH(2);
    end
    
    F_total = F_LF + F_RF + F_LH + F_RH;  % 总垂直支撑力
end

%% 完整性能分析函数
function analyze_trot_performance_corrected(t, F_LF, F_RF, F_LH, F_RH, F_total, q, params)
    fprintf('=== Trot步态性能分析 (V2) ===\n');
    
    % (性能分析逻辑不变)
    is_first_diagonal = (F_LF > 0) & (F_RH > 0);
    if any(is_first_diagonal)
        avg_F_LF = mean(F_LF(is_first_diagonal));
        avg_F_RH = mean(F_RH(is_first_diagonal));
        fprintf('第一对角支撑相(左前+右后):\n');
        fprintf('  左前腿平均力: %.1f N\n', avg_F_LF);
        fprintf('  右后腿平均力: %.1f N\n', avg_F_RH);
        fprintf('  对角合力: %.1f N\n', avg_F_LF + avg_F_RH);
    end
    
    is_second_diagonal = (F_RF > 0) & (F_LH > 0);
    if any(is_second_diagonal)
        avg_F_RF = mean(F_RF(is_second_diagonal));
        avg_F_LH = mean(F_LH(is_second_diagonal));
        fprintf('第二对角支撑相(右前+左后):\n');
        fprintf('  右前腿平均力: %.1f N\n', avg_F_RF);
        fprintf('  左后腿平均力: %.1f N\n', avg_F_LH);
        fprintf('  对角合力: %.1f N\n', avg_F_RF + avg_F_LH);
    end
    
    avg_total_force = mean(F_total);
    fprintf('总支撑力分析:\n');
    fprintf('  平均总支撑力: %.1f N\n', avg_total_force);
    fprintf('  机体重力: %.1f N\n', params.m * params.g);
    
    height_std = std(q(:,2));
    fprintf('机身高度稳定性:\n');
    fprintf('  高度均值: %.4f m (目标: %.4f m)\n', mean(q(:,2)), params.nominal_height);
    fprintf('  高度标准差: %.4f m (越小越稳定)\n', height_std);
    
    fprintf('======================\n');
end

% 辅助函数：数值限幅
function y = clamp(x, min_val, max_val)
    y = max(min(x, max_val), min_val);
end

% (verify_leg_kinematics 函数保持不变)

