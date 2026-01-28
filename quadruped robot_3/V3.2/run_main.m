function run_main()
    % 12维 MPC 四足机器人仿真主程序
    clc; close all; clear;
    

    
    %% 1. 初始化
    params = init_params();
    
    % =========== GIF 开关设置 (用户在这里控制) ============
    is_record_gif = false;             % true: 开启录制; false: 关闭录制
    gif_filename  = 'robot_demo.gif'; % 输出文件名
    % ====================================================
    
    % 初始状态
    x_curr = zeros(12, 1);
    x_curr(6) = 0.2; 
    
    % 仿真设置
    t_sim_end = 7.0;
    t_span = 0:params.dt_sim:t_sim_end;
    n_steps = length(t_span);
    
    % 数据记录
    log.x = zeros(12, n_steps);
    log.f = zeros(12, n_steps);
    log.t = t_span;
    
    % --- 绘图初始化 ---
    h_fig = figure('Color', 'w', 'Position', [100, 100, 1000, 500]);
    if is_record_gif
        fprintf('GIF 录制已开启，保存为: %s\n', gif_filename);
    end
    
    % 预先设置右边的图例
    subplot(1,2,2); 
    hold on; grid on; box on;
    h_line_h = animatedline('Color', 'b', 'LineWidth', 1.5, 'DisplayName', 'Height (Z)');
    h_line_v = animatedline('Color', 'r', 'LineWidth', 1.5, 'DisplayName', 'Velocity (Vx)');
    legend([h_line_h, h_line_v], 'Location', 'best'); 
    xlabel('Time (s)'); ylabel('State'); title('Tracking Performance');
    
    f_cmd = zeros(12, 1); 
    
    %% 2. 主循环
    mpc_counter = 0; 
    ratio_mpc_sim = round(params.dt_mpc / params.dt_sim);
    
    for k = 1:n_steps
        t = t_span(k);
        
        % 获取未来接触序列
        [contact_curr, x_next, contact_seq_mpc] = low_level_control(t, x_curr, f_cmd, params);
        
        % MPC 高层规划
        if mod(mpc_counter, ratio_mpc_sim) == 0
            [f_cmd, ~] = mpc_controller(x_curr, contact_seq_mpc, params);
        end
        mpc_counter = mpc_counter + 1;
        
        % 更新与记录
        log.x(:, k) = x_curr;
        log.f(:, k) = f_cmd;
        x_curr = x_next;
        
        % --- 可视化 (20Hz 刷新) ---
        if mod(k, 25) == 0 && isvalid(h_fig)
            % 【关键修改】将开关、文件名、当前步数k 传给 draw_robot_S(或者draw_robot_D)
            draw_robot_S(t, x_curr, f_cmd, contact_curr, params, ...
                       h_line_h, h_line_v, ...
                       is_record_gif, gif_filename, k);
        end
    end
    
    fprintf('仿真结束。\n');
end