function total_cost = run_main_joint(opt_vars)
    % RUN_MAIN_JOINT: 全物理蒸馏联合优化 (8维)
    % 目标: J = J_Task(Scrape, Energy, Stab) + J_Distill(TV, Param)
    
    p = init_params(); 
    
    %% 1. 解析 8 维参数
    % 感知层
    p.kf_thresh_noise = opt_vars.kf_thresh_noise;
    p.kf_scale_noise  = opt_vars.kf_scale_noise;
    p.kf_thresh_step  = opt_vars.kf_thresh_step;
    p.kf_scale_step   = opt_vars.kf_scale_step;
    % 【关键】应用 Base R Scale
    p.kf_R = (0.03^2) * opt_vars.kf_base_R_scale;
    
    % 规划层
    p.sm_decay    = opt_vars.sm_decay;
    p.sm_penalty  = opt_vars.sm_penalty;
    p.sm_max      = opt_vars.sm_max;
    
    %% 2. 仿真运行
    x_curr = zeros(12, 1);
    x_curr(4) = 0; x_curr(6) = get_terrain_z(x_curr(4), 0, p) + 0.2; 
    
    t_span = 0:p.dt_sim:7.0;
    n_steps = length(t_span);
    
    log.scrape = zeros(1, n_steps);
    log.f = zeros(12, n_steps);
    log.x = zeros(12, n_steps);
    log.z_est = zeros(4, n_steps); % 记录 KF 输出用于计算 TV
    
    f_cmd = zeros(12, 1);
    feet_pos_planned = zeros(3, 4);
    for i=1:4, feet_pos_planned(:,i) = x_curr(4:6) + p.hip_offset(:,i); feet_pos_planned(3,i)=0; end
    swing_start_pos = feet_pos_planned;
    contact_prev = [1;1;1;1];
    vel_filtered = zeros(3, 1);
    mpc_counter = 0; ratio_mpc = round(p.dt_mpc / p.dt_sim);
    
    for k = 1:n_steps
        t = t_span(k);
        
        % 传感器噪声注入 (模拟盲走)
        noise_vel = 0.05 * randn(6,1);
        vel_raw = x_curr(10:12) + noise_vel(4:6);
        vel_filtered = 0.15 * vel_raw + 0.85 * vel_filtered;
        x_sensed = x_curr; x_sensed(10:12) = vel_filtered;
        
        % --- 控制层 (输入是带噪的 x_sensed) ---
        [~, feet_pos_new, swing_new, contact_curr, contact_seq, scrape_flag, z_est_out] = ...
            low_level_control(t, x_sensed, f_cmd, p, feet_pos_planned, swing_start_pos, contact_prev);
            
        if mod(mpc_counter, ratio_mpc) == 0
            [f_cmd, ~] = mpc_controller(x_sensed, contact_seq, feet_pos_new, p);
        end
        mpc_counter = mpc_counter + 1;
        
        % --- 物理层 (输入是真值 x_curr) ---
        [x_next, ~, ~, ~, ~, ~, ~] = ...
            low_level_control(t, x_curr, f_cmd, p, feet_pos_new, swing_new, contact_prev);
            
        log.x(:, k) = x_curr;
        log.f(:, k) = f_cmd;
        log.scrape(k) = sum(scrape_flag);
        log.z_est(:, k) = z_est_out; % 记录 KF 轨迹
        
        x_curr = x_next; feet_pos_planned = feet_pos_new; swing_start_pos = swing_new; contact_prev = contact_curr; 
    end
    
    %% 3. 计算 Cost (全物理蒸馏 - 双重蒸馏版)
    
    % --- A. 任务性能 (Task Performance) ---
    % 1. 安全性 (Scrape)
    cost_safety = sum(log.scrape) * 2.0;
    
    % 2. 能量效率 (Energy)
    cost_energy = sum(sum(log.f.^2)) / n_steps * 0.001;
    
    % 3. 稳定性 (Stability)
    cost_stability = (var(log.x(1,:)) + var(log.x(2,:))) * 500;
    
    % --- B. 物理蒸馏 (Physical Distillation) ---
    % 4. 信号平滑蒸馏 (TV Norm) - 防止R过小抖动
    z_diff = abs(diff(log.z_est, 1, 2)); 
    avg_jitter = mean(z_diff(:));        % 平均值，自带 1/N
    cost_tv = avg_jitter * 20.0;
    
    % 5. 参数物理蒸馏 (Param Prior) - 防止阈值过大装瞎
    % 惩罚台阶阈值超过 15cm 的部分
    violation = max(0, p.kf_thresh_step - 0.15);
    cost_param = violation * 1000.0;     % 强惩罚
    
    % === 总分 ===
    total_cost = cost_safety + cost_energy + cost_stability + cost_tv + cost_param;
    
    % 摔倒惩罚
    if min(log.x(6, :)) < 0.1
        total_cost = total_cost + 10000;
    end
end