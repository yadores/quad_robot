function quadruped_trot_jump()
    % 四足机器人Trot步态混杂系统仿真（完整四腿版本，为简化模型，不考虑关节角度的变化而驱使步态相位变化）
    % 不考虑水平速度闭环控制
    %% 1. 参数定义
    params = struct();
    params.m = 10;         % 机体质量 (kg)
    params.l = 0.15;       % 腿等效长度 (m)
    params.I = 0.004;      % 单腿转动惯量
    params.k = 8000;       % 地面刚度
    params.b = 200;        % 地面阻尼
    params.mu = 0.6;       
    params.T = 0.4;        % 步态周期
    params.T_half = 0.2;   % 支撑相持续时间
    params.v = 0.1;        
    params.g = 9.81;       
    
    % 物理约束参数
    params.ground_level = 0;      % 地面高度
    params.nominal_height = 0.14; % 标称高度

    %% 2. 初始状态 - 扩展到8维状态（包含四条腿）
    % [X, Z, dX, dZ, th_LF, th_RF, th_LH, th_RH]
    q0 = zeros(8, 1);
    q0(1) = 0;                % X初始位置
    q0(2) = params.nominal_height;  % Z初始高度
    q0(3) = params.v;         % 初始前进速度
    q0(4) = 0;                % 初始垂直速度
    q0(5) = 0;                % 左前腿角度
    q0(6) = 0;                % 右前腿角度  
    q0(7) = 0;                % 左后腿角度
    q0(8) = 0;                % 右后腿角度

    %% 3. 仿真配置
    t_span = [0, 0.8];        % 2个完整周期观察模式
    options = odeset('RelTol',1e-5, 'AbsTol',1e-5, 'MaxStep',0.001);

    %% 4. 运行仿真
    try
        [t, q] = ode45(@(t,q) trot_dynamics(t,q,params), t_span, q0, options);
    catch ME
        fprintf('仿真失败: %s\n', ME.message);
        fprintf('错误详情: %s\n', ME.getReport);
        return;
    end

    %% 5. 结果可视化
    visualize_trot_results(t, q, params);
end

%% Trot步态动力学模型
function dq = trot_dynamics(t, q, params)
    % 状态：X, Z, dX, dZ, th_LF, th_RF, th_LH, th_RH
    X = q(1); Z = q(2); dX = q(3); dZ = q(4);
    th_LF = q(5); th_RF = q(6); th_LH = q(7); th_RH = q(8);
    
    % Trot步态相位判断
    t_in_cycle = mod(t, params.T);
    is_first_diagonal = (t_in_cycle < params.T_half);  % 第一对角支撑相
    
    % 计算四条腿的接触力
    F_LF = [0, 0];  % 左前腿
    F_RF = [0, 0];  % 右前腿  
    F_LH = [0, 0];  % 左后腿
    F_RH = [0, 0];  % 右后腿
    
    % 第一对角支撑相：左前+右后腿支撑，右前+左后腿摆动
    if is_first_diagonal
        % 左前腿支撑
        foot_height_LF = Z - params.l * cos(th_LF);
        delta_z_LF = params.ground_level - foot_height_LF;
        if delta_z_LF > 0
            Fz_LF = params.k * delta_z_LF - params.b * dZ;
            Fz_LF = max(Fz_LF, 0);
            Fx_LF = params.mu * Fz_LF;
            F_LF = [Fx_LF, Fz_LF];
        end
        
        % 右后腿支撑
        foot_height_RH = Z - params.l * cos(th_RH);
        delta_z_RH = params.ground_level - foot_height_RH;
        if delta_z_RH > 0
            Fz_RH = params.k * delta_z_RH - params.b * dZ;
            Fz_RH = max(Fz_RH, 0);
            Fx_RH = params.mu * Fz_RH;
            F_RH = [Fx_RH, Fz_RH];
        end
        
        % 摆动腿：右前腿和左后腿（无接触力）
        F_RF = [0, 0];
        F_LH = [0, 0];
        
    else
        % 第二对角支撑相：右前+左后腿支撑，左前+右后腿摆动
        % 右前腿支撑
        foot_height_RF = Z - params.l * cos(th_RF);
        delta_z_RF = params.ground_level - foot_height_RF;
        if delta_z_RF > 0
            Fz_RF = params.k * delta_z_RF - params.b * dZ;
            Fz_RF = max(Fz_RF, 0);
            Fx_RF = params.mu * Fz_RF;
            F_RF = [Fx_RF, Fz_RF];
        end
        
        % 左后腿支撑
        foot_height_LH = Z - params.l * cos(th_LH);
        delta_z_LH = params.ground_level - foot_height_LH;
        if delta_z_LH > 0
            Fz_LH = params.k * delta_z_LH - params.b * dZ;
            Fz_LH = max(Fz_LH, 0);
            Fx_LH = params.mu * Fz_LH;
            F_LH = [Fx_LH, Fz_LH];
        end
        
        % 摆动腿：左前腿和右后腿（无接触力）
        F_LF = [0, 0];
        F_RH = [0, 0];
    end
    
    % 质心动力学
    sum_Fx = F_LF(1) + F_RF(1) + F_LH(1) + F_RH(1);
    sum_Fz = F_LF(2) + F_RF(2) + F_LH(2) + F_RH(2);
    
    ddX = sum_Fx / params.m;
    ddZ = (sum_Fz - params.m * params.g) / params.m;
    
    % 腿角度动力学（简化模型，假设腿角度由步态控制）
    % 这里我们固定腿角度，实际中应该由步态控制器控制
    dth_LF = 0;
    dth_RF = 0;
    dth_LH = 0;
    dth_RH = 0;

    % 组装状态导数
    dq = zeros(8, 1);
    dq(1) = dX;      % dX
    dq(2) = dZ;      % dZ
    dq(3) = ddX;     % ddX
    dq(4) = ddZ;     % ddZ
    dq(5) = dth_LF;  % dth_LF
    dq(6) = dth_RF;  % dth_RF
    dq(7) = dth_LH;  % dth_LH
    dq(8) = dth_RH;  % dth_RH
end

%% 可视化函数
function visualize_trot_results(t, q, params)
    figure('Name','Trot步态完整仿真','Position',[100,100,1000,800]);
    
    % 计算各腿接触力
    [F_LF, F_RF, F_LH, F_RH, F_total] = compute_all_forces(t, q, params);
    
    % 子图1：各腿接触力
    subplot(3,2,1);
    plot(t, F_LF, 'g-', 'LineWidth',1.5); hold on;
    plot(t, F_RF, 'r-', 'LineWidth',1.5);
    plot(t, F_LH, 'b-', 'LineWidth',1.5);
    plot(t, F_RH, 'm-', 'LineWidth',1.5);
    yline(50, 'k--', '目标力50N', 'LineWidth',1);
    ylabel('接触力 (N)'); title('各腿接触力');
    legend('左前腿', '右前腿', '左后腿', '右后腿', '目标力', 'Location','best');
    grid on; ylim([0, 100]);
    
    % 子图2：机身高度
    subplot(3,2,2);
    plot(t, q(:,2), 'k-', 'LineWidth',2);
    yline(params.nominal_height, 'r--', '标称高度', 'LineWidth',1);
    xlabel('时间 (s)'); ylabel('机身高度 (m)'); 
    title('机身高度变化');
    grid on; ylim([0.13, 0.15]);
    
    % 子图3：总支撑力与重力平衡
    subplot(3,2,3);
    plot(t, F_total, 'm-', 'LineWidth',2); hold on;
    yline(98.1, 'k--', '重力98.1N', 'LineWidth',2);
    xlabel('时间 (s)'); ylabel('总支撑力 (N)'); 
    title('总支撑力 vs 重力');
    grid on; ylim([80, 120]);
    legend('总支撑力', '重力', 'Location','best');
    
    % 子图4：支撑模式
    subplot(3,2,4);
    support_pattern = get_support_pattern(t, params);
    imagesc(t, 1:4, support_pattern);
    colormap([1 1 1; 0.8 0.8 0.8]); % 白色=摆动，灰色=支撑
    ylabel('腿编号'); xlabel('时间 (s)'); title('支撑模式');
    yticks(1:4); yticklabels({'左前', '右前', '左后', '右后'});
    
    % 子图5：对角腿力对比
    subplot(3,2,5);
    F_diag1 = F_LF + F_RH;  % 第一对角
    F_diag2 = F_RF + F_LH;  % 第二对角
    plot(t, F_diag1, 'g-', 'LineWidth',2); hold on;
    plot(t, F_diag2, 'r-', 'LineWidth',2);
    yline(98.1, 'k--', '重力98.1N', 'LineWidth',1);
    xlabel('时间 (s)'); ylabel('对角腿合力 (N)'); 
    title('对角腿支撑力交替');
    legend('对角1(左前+右后)', '对角2(右前+左后)', '重力', 'Location','best');
    grid on; ylim([0, 150]);
    
    % 子图6：速度分析
    subplot(3,2,6);
    plot(t, q(:,3), 'b-', 'LineWidth',1.5); hold on;
    plot(t, q(:,4), 'r-', 'LineWidth',1.5);
    yline(params.v, 'g--', '目标速度', 'LineWidth',1);
    xlabel('时间 (s)'); ylabel('速度 (m/s)'); 
    title('机身速度');
    legend('水平速度', '垂直速度', '目标速度', 'Location','best');
    grid on;
    
    % 输出统计信息
    analyze_trot_performance(t, F_LF, F_RF, F_LH, F_RH, F_total, params);
end

%% 计算所有腿的接触力
function [F_LF, F_RF, F_LH, F_RH, F_total] = compute_all_forces(t, q, params)
    F_LF = zeros(size(t));
    F_RF = zeros(size(t));
    F_LH = zeros(size(t));
    F_RH = zeros(size(t));
    
    for i = 1:length(t)
        Z = q(i,2); dZ = q(i,4);
        t_in_cycle = mod(t(i), params.T);
        is_first_diagonal = (t_in_cycle < params.T_half);
        
        if is_first_diagonal
            % 第一对角支撑：左前+右后
            F_LF(i) = compute_leg_force(Z, dZ, q(i,5), params);
            F_RH(i) = compute_leg_force(Z, dZ, q(i,8), params);
            F_RF(i) = 0;
            F_LH(i) = 0;
        else
            % 第二对角支撑：右前+左后
            F_RF(i) = compute_leg_force(Z, dZ, q(i,6), params);
            F_LH(i) = compute_leg_force(Z, dZ, q(i,7), params);
            F_LF(i) = 0;
            F_RH(i) = 0;
        end
    end
    
    F_total = F_LF + F_RF + F_LH + F_RH;
end

%% 单腿力计算
function Fz = compute_leg_force(Z, dZ, th, params)
    foot_height = Z - params.l * cos(th);
    delta_z = params.ground_level - foot_height;
    if delta_z > 0
        Fz = params.k * delta_z - params.b * dZ;
        Fz = max(Fz, 0);  % 力不能为负
    else
        Fz = 0;
    end
end

%% 获取支撑模式
function pattern = get_support_pattern(t, params)
    pattern = zeros(4, length(t));
    for i = 1:length(t)
        t_in_cycle = mod(t(i), params.T);
        is_first_diagonal = (t_in_cycle < params.T_half);
        
        if is_first_diagonal
            pattern(1, i) = 1;  % 左前支撑
            pattern(4, i) = 1;  % 右后支撑
        else
            pattern(2, i) = 1;  % 右前支撑
            pattern(3, i) = 1;  % 左后支撑
        end
    end
end

%% Trot步态性能分析
function analyze_trot_performance(t, F_LF, F_RF, F_LH, F_RH, F_total, params)
    fprintf('=== Trot步态性能分析 ===\n');
    
    % 分析第一对角支撑相
    t_in_cycle = mod(t, params.T);
    phase1_mask = (t_in_cycle < params.T_half);
    
    if any(phase1_mask)
        avg_F_LF = mean(F_LF(phase1_mask));
        avg_F_RH = mean(F_RH(phase1_mask));
        fprintf('第一对角支撑相(左前+右后):\n');
        fprintf('  左前腿平均力: %.1f N\n', avg_F_LF);
        fprintf('  右后腿平均力: %.1f N\n', avg_F_RH);
        fprintf('  目标值: 每条腿50N\n');
    end
    
    % 分析第二对角支撑相
    phase2_mask = (t_in_cycle >= params.T_half);
    if any(phase2_mask)
        avg_F_RF = mean(F_RF(phase2_mask));
        avg_F_LH = mean(F_LH(phase2_mask));
        fprintf('第二对角支撑相(右前+左后):\n');
        fprintf('  右前腿平均力: %.1f N\n', avg_F_RF);
        fprintf('  左后腿平均力: %.1f N\n', avg_F_LH);
        fprintf('  目标值: 每条腿50N\n');
    end
    
    % 总支撑力分析
    avg_total_force = mean(F_total);
    fprintf('总支撑力分析:\n');
    fprintf('  平均总支撑力: %.1f N\n', avg_total_force);
    fprintf('  机体重力: %.1f N\n', params.m * params.g);
    fprintf('  平衡偏差: %.1f N\n', avg_total_force - params.m * params.g);
    
    fprintf('======================\n');
end