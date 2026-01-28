function run_main()
    % 12维 MPC 四足机器人仿真主程序 (合并版 V3.3)
    clc; close all; clear;
    
    %% 1. 初始化
    params = init_params();

    %==========================================gif开关
    is_record_gif = false;             
    gif_filename  = 'robot_sim_integrated.gif'; 
    %===============================================
    
    x_curr = zeros(12, 1);
    x_curr(6) = 0.2; 
    
    t_span = 0:params.dt_sim:5.0;
    n_steps = length(t_span);
    
    log.x = zeros(12, n_steps);
    log.f = zeros(12, n_steps);
    
    h_fig = figure('Color', 'w', 'Position', [100, 100, 1000, 500]);
    subplot(1,2,2); 
    hold on; grid on; box on;
    h_line_h = animatedline('Color', 'b', 'LineWidth', 1.5);
    h_line_v = animatedline('Color', 'r', 'LineWidth', 1.5);
    legend('Height', 'Velocity'); 
    
    f_cmd = zeros(12, 1); 
    
    % --- 初始化状态记忆变量 ---
    feet_pos_world = zeros(3, 4);
    for i = 1:4
        feet_pos_world(:, i) = x_curr(4:6) + params.hip_offset(:, i); 
        feet_pos_world(3, i) = 0; 
    end
    
    swing_start_pos = feet_pos_world; 
    contact_prev = [1; 1; 1; 1];      
    
    %% 2. 主循环
    mpc_counter = 0; 
    ratio_mpc_sim = round(params.dt_mpc / params.dt_sim);
    
    fprintf('仿真开始...\n');
    
    for k = 1:n_steps
        t = t_span(k);
        
        % ====================================================
        % 【调用合并后的底层】 
        % 所有的步态逻辑、轨迹更新、动力学都在这里完成
        % ====================================================
        [x_next, feet_pos_world, swing_start_pos, contact_curr, contact_seq_mpc] = ...
            low_level_control(t, x_curr, f_cmd, params, feet_pos_world, swing_start_pos, contact_prev);
        
        % 3. MPC 规划
        if mod(mpc_counter, ratio_mpc_sim) == 0
            % MPC 现在只需要从 low_level 接收 contact_seq_mpc 即可
            [f_cmd, ~] = mpc_controller(x_curr, contact_seq_mpc, params);
        end
        mpc_counter = mpc_counter + 1;

        % 4. 状态更新
        log.x(:, k) = x_curr;
        log.f(:, k) = f_cmd;
        x_curr = x_next;
        contact_prev = contact_curr; % 更新历史接触状态
        
        % 5. 可视化
        if mod(k, 25) == 0 && isvalid(h_fig)
            draw_robot_S(t, x_curr, f_cmd, contact_curr, params, ...
                       h_line_h, h_line_v, ...
                       is_record_gif, gif_filename, k, ...
                       feet_pos_world); 
        end
    end
    fprintf('仿真结束。\n');
end