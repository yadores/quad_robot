function run_main()
    % [对照组] 12维 MPC 四足机器人仿真主程序 
    % 特征：盲走策略 (Blind) + IMU噪声注入 + 足端力可视化 + 真值分离
    clc; close all; clear;
    
    %% 1. 初始化
    params = init_params(); 
    % 【重要提醒】请去 init_params 确认 terrain_type 是 'wave_stairs'

    %==========================================gif开关
    is_record_gif = false;             
    gif_filename  = 'robot_sim_control_final.gif'; 
    %===============================================
    
    x_curr = zeros(12, 1);
    
    % 初始状态
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
    % 可视化初始化
    % =================================================================
    h_fig = figure('Color', 'w', 'Position', [100, 100, 1200, 700]); 
    
    subplot(2,2,2); 
    hold on; grid on; box on;
    h_line_h = animatedline('Color', 'b', 'LineWidth', 1.5);
    h_line_v = animatedline('Color', 'r', 'LineWidth', 1.5);
    legend('Height (World)', 'Velocity (Body-X)'); 
    title('State Tracking (Control Group)');
    
    subplot(2,2,4); 
    hold on; grid on; box on;
    h_line_f1 = animatedline('Color', 'm', 'LineWidth', 1.5); 
    h_line_f2 = animatedline('Color', [0 0.7 0], 'LineWidth', 1.5); 
    legend('FL+HR (Grp1)', 'FR+HL (Grp2)');
    title('Diagonal Vertical Forces (Sum Fz)');
    xlabel('Time (s)'); ylabel('Force (N)');
    ylim([0, 300]); 
    
    f_cmd = zeros(12, 1); 
    
    % 状态记忆
    feet_pos_planned = zeros(3, 4);
    for i = 1:4
        r_hip = params.hip_offset(:, i);
        p_hip = x_curr(4:6) + r_hip;
        feet_pos_planned(1:2, i) = p_hip(1:2);
        feet_pos_planned(3, i)   = get_terrain_z(p_hip(1), p_hip(2), params); 
    end
    
    swing_start_pos = feet_pos_planned; 
    contact_prev = [1; 1; 1; 1];      
    
    %% 2. 主循环
    mpc_counter = 0; 
    ratio_mpc_sim = round(params.dt_mpc / params.dt_sim);
    
    fprintf('仿真开始 (对照组: 盲走+噪声+真值记录)...\n');
    
    for k = 1:n_steps
        t = t_span(k);
        
        % ==========================================================
        % 1. 噪声注入 (Sensor Noise)
        % ==========================================================
        noise_orientation = 0.01 * randn(3, 1); 
        noise_velocity = 0.05 * randn(6, 1);
        
        x_sensed = x_curr;
        x_sensed(1:3) = x_sensed(1:3) + noise_orientation;   
        x_sensed(7:12) = x_sensed(7:12) + noise_velocity;    
        
        % ==========================================================
        % 2. 规划层 (Blind Planning)
        % 注意：调用的是 low_level_control
        % ==========================================================
        [~, feet_pos_planned_new, swing_start_new, contact_curr, contact_seq_mpc] = ...
            low_level_control(t, x_sensed, f_cmd, params, feet_pos_planned, swing_start_pos, contact_prev);
        
        % MPC 规划
        if mod(mpc_counter, ratio_mpc_sim) == 0
            [f_cmd, ~] = mpc_controller(x_sensed, contact_seq_mpc, feet_pos_planned_new, params);
        end
        mpc_counter = mpc_counter + 1;
        
        % ==========================================================
        % 3. 物理引擎 (Physics Update)
        % 使用真值 x_curr，并捕获 feet_pos_true
        % ==========================================================
        [x_next, feet_pos_true, ~, ~, ~] = ...
            low_level_control(t, x_curr, f_cmd, params, feet_pos_planned_new, swing_start_pos, contact_prev);
        
        % ==========================================================
        % 4. 数据记录
        % ==========================================================
        log.feet(:, :, k) = feet_pos_true; % 【关键修正】记录物理真值
        log.x(:, k) = x_curr; 
        log.f(:, k) = f_cmd;
        
        % 状态更新
        x_curr = x_next;
        feet_pos_planned = feet_pos_planned_new; % 记忆更新
        swing_start_pos = swing_start_new;
        contact_prev = contact_curr; 
        
        % 可视化
        if mod(k, 30) == 0 && isvalid(h_fig)
            draw_robot_S(t, x_curr, f_cmd, contact_curr, params, ...
                       h_line_h, h_line_v, h_line_f1, h_line_f2, ...
                       is_record_gif, gif_filename, k, ...
                       feet_pos_true); % 【关键修正】画真值
        end
    end
    fprintf('仿真结束。\n');
    
    %% 3. 分析
    evaluate_scrapes(log, params, 'Control Group (Blind MPC+Noise)');
end