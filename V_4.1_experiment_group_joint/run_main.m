function run_main()
    % [实验组] 12维 MPC 四足机器人仿真主程序 
    % 特征：地形适应版 V6.0 (MPC+KF+避险) + IMU噪声注入 + 足端力可视化 + 真值/估计值分离 + 【新增】速度滤波
    clc; close all; clear;
    
    %% 1. 初始化
    params = init_params();

    %==========================================gif开关
    is_record_gif = false;             
    gif_filename  = 'robot_sim_experiment_final_filtered.gif'; 
    %===============================================
    
    x_curr = zeros(12, 1);
    
    % 初始状态设置
    x_curr(4) = 0; % x
    x_curr(6) = get_terrain_z(x_curr(4), 0, params) + 0.2; % z
    
    t_span = 0:params.dt_sim:7.0;
    n_steps = length(t_span);
    
    % 数据记录
    log.x = zeros(12, n_steps);
    log.f = zeros(12, n_steps);
    log.feet = zeros(3, 4, n_steps);     
    log.t = t_span;                      
    
    % =================================================================
    % 【可视化初始化】
    % =================================================================
    h_fig = figure('Color', 'w', 'Position', [100, 100, 1200, 700]); 
    
    subplot(2,2,2); 
    hold on; grid on; box on;
    h_line_h = animatedline('Color', 'b', 'LineWidth', 1.5);
    h_line_v = animatedline('Color', 'r', 'LineWidth', 1.5);
    legend('Height (World)', 'Velocity (Body-X)'); 
    title('State Tracking');
    
    subplot(2,2,4); 
    hold on; grid on; box on;
    h_line_f1 = animatedline('Color', 'm', 'LineWidth', 1.5); 
    h_line_f2 = animatedline('Color', [0 0.7 0], 'LineWidth', 1.5); 
    legend('FL+HR (Grp1)', 'FR+HL (Grp2)');
    title('Diagonal Vertical Forces (Sum Fz)');
    xlabel('Time (s)'); ylabel('Force (N)');
    ylim([0, 300]); 
    
    f_cmd = zeros(12, 1); 
    
    % --- 初始化状态记忆变量 ---
    feet_pos_planned = zeros(3, 4);
    for i = 1:4
        r_hip = params.hip_offset(:, i);
        p_hip = x_curr(4:6) + r_hip;
        feet_pos_planned(1:2, i) = p_hip(1:2);
        feet_pos_planned(3, i)   = get_terrain_z(p_hip(1), p_hip(2), params); 
    end
    
    swing_start_pos = feet_pos_planned; 
    contact_prev = [1; 1; 1; 1];      

    % =================================================================
    % 【新增】滤波器初始化
    % =================================================================
    vel_filtered = zeros(3, 1); % 初始化滤波后的线速度
    alpha_v = 0.15;             % 滤波系数 (0~1)
                                % 0.15 表示非常平滑 (相信历史值 85%)，能有效消除锯齿
                                % 如果觉得延迟太大，可以调高到 0.3
    
    %% 2. 主循环
    mpc_counter = 0; 
    ratio_mpc_sim = round(params.dt_mpc / params.dt_sim);
    
    fprintf('仿真开始 (实验组)...\n');
    
    for k = 1:n_steps
        t = t_span(k);
        
        % ==========================================================
        % 【环节 A】: 传感器噪声与信号处理 (Sensor Processing)
        % ==========================================================
        % 1. 生成原始噪声
        noise_orientation = 0.01 * randn(3, 1); 
        noise_velocity = 0.05 * randn(6, 1); % 前3维是角速度，后3维是线速度
        
        % 2. 获取带噪的原始线速度 (Raw Measurement)
        vel_raw = x_curr(10:12) + noise_velocity(4:6);
        
        % 3. 【新增】低通滤波 (Low Pass Filter)
        % 公式: v_now = alpha * v_raw + (1-alpha) * v_last
        vel_filtered = alpha_v * vel_raw + (1 - alpha_v) * vel_filtered;
        
        % 4. 构造"感知状态" (Sensed State) 喂给 MPC
        x_sensed = x_curr;
        x_sensed(1:3) = x_sensed(1:3) + noise_orientation;   
        
        % 角速度这里暂时不做强滤波 (保持灵敏度)
        x_sensed(7:9) = x_sensed(7:9) + noise_velocity(1:3); 
        
        % 【关键】线速度使用滤波后的值！
        x_sensed(10:12) = vel_filtered;    
        % ==========================================================

        % ==========================================================
        % 【环节 B】: 感知与规划 (Perception & Planning)
        % ==========================================================
        [~, feet_pos_planned_new, swing_start_new, contact_curr, contact_seq_mpc] = ...
            low_level_control(t, x_sensed, f_cmd, params, feet_pos_planned, swing_start_pos, contact_prev);
        
        % MPC 规划 (Control Layer)
        if mod(mpc_counter, ratio_mpc_sim) == 0
            [f_cmd, ~] = mpc_controller(x_sensed, contact_seq_mpc, feet_pos_planned_new, params);
        end
        mpc_counter = mpc_counter + 1;
        
        % ==========================================================
        % 【环节 C】: 物理引擎更新 (Physics Engine Update)
        % 注意：物理层必须用 x_curr (真值)，不能受滤波影响
        % ==========================================================
        [x_next, feet_pos_true, ~, ~, ~] = ...
            low_level_control(t, x_curr, f_cmd, params, feet_pos_planned_new, swing_start_pos, contact_prev);
        
        % ==========================================================
        % 【环节 D】: 数据记录与状态迭代
        % ==========================================================
        log.feet(:, :, k) = feet_pos_true; 
        log.x(:, k) = x_curr; 
        log.f(:, k) = f_cmd;
        
        x_curr = x_next;
        feet_pos_planned = feet_pos_planned_new; 
        swing_start_pos = swing_start_new;
        contact_prev = contact_curr; 
        
        % ==========================================================
        % 【环节 E】: 可视化
        % ==========================================================
        if mod(k, 30) == 0 && isvalid(h_fig)
            draw_robot_S(t, x_curr, f_cmd, contact_curr, params, ...
                       h_line_h, h_line_v, h_line_f1, h_line_f2, ...
                       is_record_gif, gif_filename, k, ...
                       feet_pos_true); 
        end
    end
    fprintf('仿真结束。\n');
    
    %% 3. 分析
    evaluate_scrapes(log, params, 'Experiment Group (KF+2.5+自适应超参数)');
end