function [x_next, feet_pos_next, swing_start_next, contact_curr, contact_seq_mpc] = low_level_control(t, x_curr, f_cmd, p, feet_pos_prev, swing_start_prev, contact_prev)
    % low_level_control: 包含步态逻辑、运动学更新和动力学演化的完整底层
    % 
    % 输入:
    %   feet_pos_prev: 上一刻的足端位置
    %   swing_start_prev: 上一次记录的起脚点
    %   contact_prev: 上一刻的接触状态
    %
    % 输出:
    %   x_next: 下一刻机器人的状态
    %   feet_pos_next: 更新后的足端位置 (用于画图和下一步计算)
    %   swing_start_next: 更新后的起脚点记忆
    %   contact_curr: 当前接触状态
    
    %% 1. 计算步态与接触状态 (Gait Logic)
    contact_curr = get_gait_contact(t, p);
    
    %% 2. 预测未来 N 步接触 (给 MPC 用)
    contact_seq_mpc = zeros(4, p.N);
    for k = 1:p.N
        t_pred = t + (k-1) * p.dt_mpc; 
        contact_seq_mpc(:, k) = get_gait_contact(t_pred, p);
    end
    
    %% 3. 更新足端位置 (Kinematics)
    feet_pos_next = feet_pos_prev;       % 默认继承上一帧
    swing_start_next = swing_start_prev; % 默认继承上一帧
    
    for i = 1:4
        if contact_curr(i) == 0 
            % === 摆动相 (Swing) ===
            
            % 检测起脚时刻 (Stance -> Swing)
            if contact_prev(i) == 1
                swing_start_next(:, i) = feet_pos_prev(:, i);
            end
            
            % 计算摆动轨迹
            % 直接调用内部函数，计算当前时刻 t 的位置
            feet_pos_next(:, i) = get_swing_pos_cycloid(t, i, 0, x_curr, p, swing_start_next(:, i));
            
        else
            % === 支撑相 (Stance) ===
            % 锁死在地面 (模拟摩擦)
            feet_pos_next(3, i) = 0; 
            % X, Y 保持 feet_pos_prev 的值不变
        end
    end
    
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
            
            % 【核心】SRBD 力臂 = 足端位置 - 质心位置
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
    
    if x_next(6) < 0, x_next(6) = 0; x_next(12) = 0; end
end

%% ==========================================
%  内部函数 (原 get_gait_contact.m)
% ==========================================
function contact = get_gait_contact(t, p)
    T = p.T_gait;
    phase = mod(t, T) / T;
    contact = zeros(4, 1);
    
    % Trot: 0~0.5 (1,4支撑); 0.5~1.0 (2,3支撑)
    if phase < 0.5
        contact(1) = 1; contact(4) = 1;
    else
        contact(2) = 1; contact(3) = 1;
    end
end

%% ==========================================
%  内部函数 (原 get_swing_pos_cycloid.m)
% ==========================================
function p_foot_cmd = get_swing_pos_cycloid(t, i, contact_state, x_curr, p, p_start)
    if contact_state == 1
        p_foot_cmd = p_start; p_foot_cmd(3) = 0; return;
    end
    
    com_pos = x_curr(4:6);    
    com_vel = x_curr(10:12);  
    yaw = x_curr(3);
    R_z = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
    
    hip_pos_world = com_pos + R_z * p.hip_offset(:, i);
    
    T = p.T_gait;
    phase_global = mod(t, T) / T;
    s = 0;
    
    if i == 1 || i == 4, s = (phase_global - 0.5) / 0.5;
    else,                s = (phase_global - 0.0) / 0.5;
    end
    s = max(0, min(1, s));
    
    T_stance = T * 0.5; 
    capture_point = com_vel * (T_stance / 2); 
    p_end_xy = hip_pos_world(1:2) + capture_point(1:2);
    p_end = [p_end_xy; 0]; 
    
    s_map = s - sin(2*pi*s) / (2*pi); 
    xy_curr = p_start(1:2) + (p_end(1:2) - p_start(1:2)) * s_map;
    z_arch = p.h_swing * (1 - cos(2*pi*s)) / 2;
    p_foot_cmd = [xy_curr; 0 + z_arch];
end