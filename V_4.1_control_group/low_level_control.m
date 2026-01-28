function [x_next, feet_pos_next, swing_start_next, contact_curr, contact_seq_mpc] = low_level_control(t, x_curr, f_cmd, p, feet_pos_prev, swing_start_prev, contact_prev)
    % ... (前两部分 gait 和 MPC prediction 代码不变) ...
    
    %% 1. 计算步态与接触状态
    contact_curr = get_gait_contact(t, p);
    
    %% 2. 预测未来 N 步接触
    contact_seq_mpc = zeros(4, p.N);
    for k = 1:p.N
        t_pred = t + (k-1) * p.dt_mpc; 
        contact_seq_mpc(:, k) = get_gait_contact(t_pred, p);
    end
    
    %% 3. 更新足端位置 (Kinematics)
    feet_pos_next = feet_pos_prev;       
    swing_start_next = swing_start_prev; 
    
    for i = 1:4
        if contact_curr(i) == 0 
            % === 摆动相 (Swing) ===
            if contact_prev(i) == 1
                swing_start_next(:, i) = feet_pos_prev(:, i);
            end
            
            % 摆动轨迹计算 (xy平滑插值，z是拱形)
            % 注意：get_swing_pos_cycloid 内部会自动处理起止点高度不同的情况
            % 只要传入的 x_curr 和 p_start 是对的，它算出来的 z 就会自然适应
            feet_pos_next(:, i) = get_swing_pos_cycloid(t, i, 0, x_curr, p, swing_start_next(:, i));
            
        else
            % === 支撑相 (Stance) ===
            
            % 1. 获取 MPC 计算出的足端力
            f_leg = f_cmd((i-1)*3+1 : i*3);
            fx = f_leg(1); fy = f_leg(2); fz = f_leg(3);
            
            % 2. 计算实际摩擦需求
            f_horizontal = sqrt(fx^2 + fy^2);
            f_limit = p.mu * fz; % 最大静摩擦力
            
            % 3. 打滑判定 logic
            if f_horizontal > f_limit && fz > 1e-3
                % 【发生打滑】
                % 摩擦力不足以维持静止，脚开始漂移
                % 漂移方向与受力方向相反
                slip_factor = 0.01; % 打滑系数 (模拟滑动速度)
                slip_vec = -[fx; fy] * slip_factor;
                
                % 更新位置 (脚滑走了)
                feet_pos_next(1:2, i) = feet_pos_prev(1:2, i) + slip_vec;
                
                % 可以在控制台打印一下，看看什么时候滑了
                % fprintf('Leg %d SLIPPING! Force: %.1f > Limit: %.1f\n', i, f_horizontal, f_limit);
            else
                % 【未打滑】锁死
                feet_pos_next(1:2, i) = feet_pos_prev(1:2, i); 
            end
            
            % Z轴依然贴地 (假设没滑出悬崖)
            z_ground = get_terrain_z(feet_pos_next(1, i), feet_pos_next(2, i), p);
            feet_pos_next(3, i) = z_ground;
        end
    end
    
    % ... (第4部分 动力学 Dynamics 代码完全不变，因为力臂计算依赖 feet_pos_next) ...
    %% 4. 物理对象仿真 (Dynamics)
    dt = p.dt_sim;
    vel = x_curr(10:12);
    omega = x_curr(7:9);
    psi = x_curr(3);
    R_z = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    
    F_total = zeros(3, 1);
    Tau_total = zeros(3, 1);
    pos_com = x_curr(4:6);
    
    for i = 1:4
        if contact_curr(i) == 1
            f_leg = f_cmd((i-1)*3+1 : i*3);
            % SRBD 力臂 = 足端位置 - 质心位置
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
    
    % 地面碰撞保护（改为简单的最低高度限制，防止数值发散）
    ground_height_com = get_terrain_z(x_next(4), x_next(5), p);
    if x_next(6) < ground_height_com + 0.05
        x_next(6) = ground_height_com + 0.05;
        x_next(12) = 0;
    end
end

% (内部函数 get_gait_contact 和 get_swing_pos_cycloid 不需要改动，保留即可)
function contact = get_gait_contact(t, p)
    T = p.T_gait;
    phase = mod(t, T) / T;
    contact = zeros(4, 1);
    if phase < 0.5, contact(1) = 1; contact(4) = 1; else, contact(2) = 1; contact(3) = 1; end
end

function p_foot_cmd = get_swing_pos_cycloid(t, i, contact_state, x_curr, p, p_start)
    if contact_state == 1, p_foot_cmd = p_start; return; end 
    
    com_pos = x_curr(4:6); com_vel = x_curr(10:12); yaw = x_curr(3);
    R_z = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
    hip_pos_world = com_pos + R_z * p.hip_offset(:, i);
    
    T = p.T_gait; phase_global = mod(t, T) / T; s = 0;
    if i == 1 || i == 4, s = (phase_global - 0.5)/0.5; else, s = (phase_global - 0.0)/0.5; end
    s = max(0, min(1, s));
    
    T_stance = T * 0.5; capture_point = com_vel * (T_stance / 2);
    p_end_xy = hip_pos_world(1:2) + capture_point(1:2);
    
    % =========== 【V4 修正：盲视摆动】Start ===========
    % 原代码（作弊）：z_end = get_terrain_z(p_end_xy(1), p_end_xy(2), p);
    
    % 新代码（真实盲走）：
    % 机器人看不见地形，只能假设落点高度和起步点高度一样（假设前面是平地）
    z_end = p_start(3); 
    % =========== 【V4 修正：盲视摆动】End =============
    
    p_end = [p_end_xy; z_end];
    
    s_map = s - sin(2*pi*s)/(2*pi);
    xy_curr = p_start(1:2) + (p_end(1:2) - p_start(1:2)) * s_map;
    
    z_start = p_start(3);
    
    % 为了盲走时稍微安全点，通常建议把抬腿高度稍微调高（例如 init_params 设为 0.15）
    h_lift = p.h_swing * (1 - cos(2*pi*s)) / 2; 
    z_base = z_start + (z_end - z_start) * s_map; 
    
    p_foot_cmd = [xy_curr; z_base + h_lift];
end