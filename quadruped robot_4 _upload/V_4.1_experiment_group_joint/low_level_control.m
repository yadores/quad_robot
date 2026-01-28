function [x_next, feet_pos_next, swing_start_next, contact_curr, contact_seq_mpc, scrape_flag, z_est_out] = low_level_control(t, x_curr, f_cmd, p, feet_pos_prev, swing_start_prev, contact_prev)
    % LOW_LEVEL_CONTROL 底层控制模块 (V6 联合优化适配版)
    % 修改点：新增输出 z_est_out (4x1)，用于计算 TV 正则化指标
    
    %% 0. 卡尔曼滤波器状态记忆
    persistent z_est_ground P_est_ground
    persistent safety_margin_adaptive 
    
    % t=0 时重置记忆
    if t == 0 || isempty(z_est_ground)
        z_est_ground = zeros(4, 1);    
        P_est_ground = ones(4, 1) * 1; 
        safety_margin_adaptive = 0.0; 
    end

    scrape_flag = zeros(4, 1); 

    %% 1. 计算步态与接触状态
    contact_curr = get_gait_contact(t, p);
    
    %% 2. 预测未来 N 步接触
    contact_seq_mpc = zeros(4, p.N);
    for k = 1:p.N
        t_pred = t + (k-1) * p.dt_mpc; 
        contact_seq_mpc(:, k) = get_gait_contact(t_pred, p);
    end
    
    %% 3. 更新足端位置
    feet_pos_next = feet_pos_prev;       
    swing_start_next = swing_start_prev; 
    
    % 自适应安全余量衰减
    safety_margin_adaptive = safety_margin_adaptive - p.sm_decay; 
    if safety_margin_adaptive < 0, safety_margin_adaptive = 0; end
    if safety_margin_adaptive > p.sm_max, safety_margin_adaptive = p.sm_max; end
    
    for i = 1:4
        % =======================
        % Part A: 地形感知 (KF)
        % =======================
        com_vel = x_curr(10:12);
        T_stance = p.T_gait * 0.5;
        capture_point = com_vel * (T_stance / 2); 
        
        yaw = x_curr(3);
        R_z = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
        hip_pos_world = x_curr(4:6) + R_z * p.hip_offset(:, i);
        p_land_nominal = hip_pos_world(1:2) + capture_point(1:2);
        
        % 读取传感器 (带噪声)
        [z_meas, q_flag] = get_sensor_reading(p_land_nominal(1), p_land_nominal(2), p);
        
        % KF 融合 (更新估计值)
        z_pred = z_est_ground(i);
        P_prev = P_est_ground(i);
        [z_new, P_new] = adaptive_kalman_filter(z_pred, P_prev, z_meas, q_flag, p);
        
        z_est_ground(i) = z_new;
        P_est_ground(i) = P_new;
        
        % =======================
        % Part B: 摆动相规划
        % =======================
        if contact_curr(i) == 0 
            if contact_prev(i) == 1
                swing_start_next(:, i) = feet_pos_prev(:, i);
            end
            
            % 落点搜索
            p_target_xy = p_land_nominal; 
            [~, q_flag_nom] = get_sensor_reading(p_land_nominal(1), p_land_nominal(2), p);
            
            if q_flag_nom == 1
                best_dist = inf; found_safe = false;
                radius = 0.10; step = 0.02;
                for dx = -radius:step:radius
                    for dy = -radius:step:radius
                        test_xy = p_land_nominal + [dx; dy];
                        [~, q_flag_test] = get_sensor_reading(test_xy(1), test_xy(2), p);
                        if q_flag_test == 0
                            dist = norm([dx; dy]);
                            if dist < best_dist
                                best_dist = dist; p_target_xy = test_xy; found_safe = true;
                            end
                        end
                    end
                end
            end
            
            % 生成轨迹 (基于 z_new 规划，不作弊)
            feet_pos_next(:, i) = get_swing_pos_cycloid_V6(t, i, x_curr, p, ...
                                    swing_start_next(:, i), p_target_xy, z_new, safety_margin_adaptive);
            
            % 痛觉检测 (Physics Check - 只有这里用了真值模拟触觉)
            z_real = get_terrain_z(feet_pos_next(1, i), feet_pos_next(2, i), p);
            if feet_pos_next(3, i) < z_real - 0.002
                scrape_flag(i) = 1;
                safety_margin_adaptive = safety_margin_adaptive + p.sm_penalty; 
            end
            
        else
            % =======================
            % Part C: 支撑相
            % =======================
            f_leg = f_cmd((i-1)*3+1 : i*3);
            fx = f_leg(1); fy = f_leg(2); fz = f_leg(3);
            f_hor = sqrt(fx^2 + fy^2); f_lim = p.mu * fz;
            
            if f_hor > f_lim && fz > 1e-3
                feet_pos_next(1:2, i) = feet_pos_prev(1:2, i) - [fx; fy] * 0.01; 
            else
                feet_pos_next(1:2, i) = feet_pos_prev(1:2, i); 
            end
            
            z_real = get_terrain_z(feet_pos_next(1, i), feet_pos_next(2, i), p);
            feet_pos_next(3, i) = z_real;
            z_est_ground(i) = z_real; 
            P_est_ground(i) = 0.001; 
        end
    end
    
    % --- 输出当前所有腿的 KF 估计值 ---
    z_est_out = z_est_ground; 
    
    %% 4. 动力学仿真 (Physics Engine - 使用真值)
    dt = p.dt_sim;
    vel = x_curr(10:12); omega = x_curr(7:9); psi = x_curr(3);
    R_z = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    
    F_total = zeros(3, 1); Tau_total = zeros(3, 1); pos_com = x_curr(4:6);
    
    for i = 1:4
        if contact_curr(i) == 1
            f_leg = f_cmd((i-1)*3+1 : i*3);
            r = feet_pos_next(:, i) - pos_com;
            F_total = F_total + f_leg;
            Tau_total = Tau_total + cross(r, f_leg);
        end
    end
    
    acc_lin = F_total / p.m + [0; 0; -p.g];
    I_world = R_z * p.Ibody * R_z'; 
    acc_ang = I_world \ Tau_total;
    
    x_next = x_curr;
    x_next(1:3)   = x_curr(1:3)   + R_z * omega * dt; 
    x_next(4:6)   = x_curr(4:6)   + vel * dt;         
    x_next(7:9)   = x_curr(7:9)   + acc_ang * dt;     
    x_next(10:12) = x_curr(10:12) + acc_lin * dt;     
    
    ground_height_com = get_terrain_z(x_next(4), x_next(5), p);
    if x_next(6) < ground_height_com + 0.05
        x_next(6) = ground_height_com + 0.05; x_next(12) = 0;
    end
end

% 辅助函数保持在同一个文件里
function contact = get_gait_contact(t, p)
    T = p.T_gait;
    phase = mod(t, T) / T;
    contact = zeros(4, 1);
    if phase < 0.5, contact(1) = 1; contact(4) = 1; else, contact(2) = 1; contact(3) = 1; end
end

function p_foot_cmd = get_swing_pos_cycloid_V6(t, i, x_curr, p, p_start, target_xy, z_est, safe_margin)
    T = p.T_gait; phase_global = mod(t, T) / T; s = 0;
    if i == 1 || i == 4, s = (phase_global - 0.5)/0.5; else, s = (phase_global - 0.0)/0.5; end
    s = max(0, min(1, s));
    
    z_end = z_est + safe_margin; % 基于估计值规划
    p_end = [target_xy; z_end];
    s_map = s - sin(2*pi*s)/(2*pi);
    xy_curr = p_start(1:2) + (p_end(1:2) - p_start(1:2)) * s_map;
    z_start = p_start(3);
    step_diff = z_end - z_start;
    h_lift_peak = p.h_swing;
    if step_diff > 0.02, h_lift_peak = p.h_swing + step_diff * 2.5; end
    h_lift = h_lift_peak * (1 - cos(2*pi*s)) / 2; 
    z_base = z_start + (z_end - z_start) * s_map; 
    p_foot_cmd = [xy_curr; z_base + h_lift];
end