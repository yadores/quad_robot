function [x_next, feet_pos_next, swing_start_next, contact_curr, contact_seq_mpc, scrape_flag] = low_level_control(t, x_curr, f_cmd, p, feet_pos_prev, swing_start_prev, contact_prev)
    % LOW_LEVEL_CONTROL 底层控制模块 (V6 落点感知避险版)
    % 包含：KF融合、自适应安全余量、自适应高抬腿、以及【新增】落点安全搜索
    
    %% 0. 卡尔曼滤波器状态记忆
    persistent z_est_ground P_est_ground
    persistent safety_margin_adaptive % 自适应安全余量记忆
    
    if t == 0 || isempty(z_est_ground)
        z_est_ground = zeros(4, 1);    
        P_est_ground = ones(4, 1) * 1; 
        safety_margin_adaptive = 0.0; % 初始余量
    end

    scrape_flag = zeros(4, 1); 

    %% 1. 计算步态与接触状态
    contact_curr = get_gait_contact(t, p);
    
    %% 2. 预测未来 N 步接触 (供 MPC 使用)
    contact_seq_mpc = zeros(4, p.N);
    for k = 1:p.N
        t_pred = t + (k-1) * p.dt_mpc; 
        contact_seq_mpc(:, k) = get_gait_contact(t_pred, p);
    end
    
    %% 3. 更新足端位置 (核心规划层)
    feet_pos_next = feet_pos_prev;       
    swing_start_next = swing_start_prev; 
    
    % --- 自适应安全余量衰减 (遗忘机制) ---
    safety_margin_adaptive = safety_margin_adaptive - 0.00005; 
    if safety_margin_adaptive < 0, safety_margin_adaptive = 0; end
    if safety_margin_adaptive > 0.08, safety_margin_adaptive = 0.08; end
    
    for i = 1:4
        % =======================
        % Part A: 地形感知 (KF)
        % =======================
        % 预测落点用于读取传感器（粗略估计）
        com_vel = x_curr(10:12);
        T_stance = p.T_gait * 0.5;
        capture_point = com_vel * (T_stance / 2); 
        
        yaw = x_curr(3);
        R_z = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
        hip_pos_world = x_curr(4:6) + R_z * p.hip_offset(:, i);
        p_land_nominal = hip_pos_world(1:2) + capture_point(1:2);
        
        % 读取传感器
        [z_meas, q_flag] = get_sensor_reading(p_land_nominal(1), p_land_nominal(2), p);
        
        % 卡尔曼滤波融合
        z_pred = z_est_ground(i);
        P_prev = P_est_ground(i);
        [z_new, P_new] = adaptive_kalman_filter(z_pred, P_prev, z_meas, q_flag, p);
        
        z_est_ground(i) = z_new;
        P_est_ground(i) = P_new;
        
        % =======================
        % Part B: 摆动相规划 (V6 核心)
        % =======================
        if contact_curr(i) == 0 
            if contact_prev(i) == 1
                swing_start_next(:, i) = feet_pos_prev(:, i);
            end
            
            % ----------------------------------------------------
            % 【V6 新增】基于置信度的落点搜索 (Confidence-Aware Search)
            % ----------------------------------------------------
            % 1. 检查名义落点 p_land_nominal 是否安全
            % 我们利用 quality_flag (丢包标志) 或 P_new (协方差) 来判断
            % 这里使用 q_flag (1=丢包/悬崖/深坑)
            
            p_target_xy = p_land_nominal; % 默认目标
            
            [~, q_flag_nom] = get_sensor_reading(p_land_nominal(1), p_land_nominal(2), p);
            
            if q_flag_nom == 1
                % 警告：前方名义落点数据丢失（可能是狭缝/深坑）！
                % 启动局部搜索：在周围找一个有信号的落点
                
                best_dist = inf;
                found_safe = false;
                
                % 搜索参数：半径 10cm，步长 2cm
                radius = 0.10; 
                step = 0.02;
                
                for dx = -radius:step:radius
                    for dy = -radius:step:radius
                        test_xy = p_land_nominal + [dx; dy];
                        [~, q_flag_test] = get_sensor_reading(test_xy(1), test_xy(2), p);
                        
                        if q_flag_test == 0
                            % 找到实地了！
                            dist = norm([dx; dy]);
                            if dist < best_dist
                                best_dist = dist;
                                p_target_xy = test_xy; % 更新为安全落点
                                found_safe = true;
                            end
                        end
                    end
                end
                
                % 如果周围全是坑，只能保持原样（听天由命），或者触发急停
                if ~found_safe
                    % fprintf('Leg %d: Panic! No safe foothold found.\n', i);
                end
            end
            % ----------------------------------------------------
            
            % 生成摆动轨迹 (传入修正后的 p_target_xy)
            feet_pos_next(:, i) = get_swing_pos_cycloid_V6(t, i, x_curr, p, ...
                                    swing_start_next(:, i), p_target_xy, z_new, safety_margin_adaptive);
            
            % 在线刮擦检测
            z_real = get_terrain_z(feet_pos_next(1, i), feet_pos_next(2, i), p);
            if feet_pos_next(3, i) < z_real - 0.002
                scrape_flag(i) = 1;
                safety_margin_adaptive = safety_margin_adaptive + 0.005; % 痛觉学习
            end
            
        else
            % =======================
            % Part C: 支撑相 (Stance)
            % =======================
            f_leg = f_cmd((i-1)*3+1 : i*3);
            fx = f_leg(1); fy = f_leg(2); fz = f_leg(3);
            
            f_hor = sqrt(fx^2 + fy^2);
            f_lim = p.mu * fz;
            
            if f_hor > f_lim && fz > 1e-3
                feet_pos_next(1:2, i) = feet_pos_prev(1:2, i) - [fx; fy] * 0.01; % 简单打滑
            else
                feet_pos_next(1:2, i) = feet_pos_prev(1:2, i); 
            end
            
            % 支撑相 Z 轴贴地
            z_real = get_terrain_z(feet_pos_next(1, i), feet_pos_next(2, i), p);
            feet_pos_next(3, i) = z_real;
            
            % 落地瞬间重置 KF 不确定性 (相信接触)
            z_est_ground(i) = z_real; 
            P_est_ground(i) = 0.001; 
        end
    end
    
    %% 4. 动力学仿真 (保持不变)
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

%% 辅助函数 (Gait)
function contact = get_gait_contact(t, p)
    T = p.T_gait;
    phase = mod(t, T) / T;
    contact = zeros(4, 1);
    if phase < 0.5, contact(1) = 1; contact(4) = 1; else, contact(2) = 1; contact(3) = 1; end
end

%% 辅助函数 (V6 摆动规划 - 接收明确的 target_xy)
function p_foot_cmd = get_swing_pos_cycloid_V6(t, i, x_curr, p, p_start, target_xy, z_est, safe_margin)
    % 这里的 target_xy 是我们在主循环里经过"安全搜索"后的最优落点
    
    T = p.T_gait; phase_global = mod(t, T) / T; s = 0;
    if i == 1 || i == 4, s = (phase_global - 0.5)/0.5; else, s = (phase_global - 0.0)/0.5; end
    s = max(0, min(1, s));
    
    % 目标高度 = KF估计高度 + 自适应安全余量
    z_end = z_est + safe_margin; 
    
    p_end = [target_xy; z_end];
    
    % 摆动轨迹插值
    s_map = s - sin(2*pi*s)/(2*pi);
    xy_curr = p_start(1:2) + (p_end(1:2) - p_start(1:2)) * s_map;
    
    z_start = p_start(3);
    step_diff = z_end - z_start;
    
    % 自适应高抬腿策略
    h_lift_peak = p.h_swing;
    if step_diff > 0.02
        h_lift_peak = p.h_swing + step_diff * 2.5; % 遇到台阶抬更高
    end
    
    h_lift = h_lift_peak * (1 - cos(2*pi*s)) / 2; 
    z_base = z_start + (z_end - z_start) * s_map; 
    
    p_foot_cmd = [xy_curr; z_base + h_lift];
end