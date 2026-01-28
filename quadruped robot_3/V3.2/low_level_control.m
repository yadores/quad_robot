function [contact_state, x_next, contact_seq_mpc] = low_level_control(t, x_curr, f_cmd, p)
    % low_level_control: 包含步态调度和物理动力学演化
    
    %% 1. 计算当前接触状态
    contact_state = get_gait_contact(t, p);
    
    %% 2. 预测未来 N 步的接触状态 (供 MPC 使用)
    contact_seq_mpc = zeros(4, p.N);
    for k = 1:p.N
        t_pred = t + (k-1) * p.dt_mpc; 
        contact_seq_mpc(:, k) = get_gait_contact(t_pred, p);
    end
    
    %% 3. 物理对象仿真 (Robot Plant Dynamics)
    dt = p.dt_sim;
    
    % 提取状态
    vel = x_curr(10:12);
    omega = x_curr(7:9);
    psi = x_curr(3);
    pos = x_curr(4:6); % 当前位置
    
    % 旋转矩阵
    R_z = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    
    % 计算合力与合力矩
    F_total = zeros(3, 1);
    Tau_total = zeros(3, 1);
    
    for i = 1:4
        if contact_state(i) == 1
            f_leg = f_cmd((i-1)*3+1 : i*3);
            r = R_z * p.hip_offset(:, i);
            
            F_total = F_total + f_leg;
            Tau_total = Tau_total + cross(r, f_leg);
        end
    end
    
    % 外部干扰 (保留原逻辑，也可根据需求移除)
    F_dist = zeros(3, 1);
    Tau_dist = zeros(3, 1);
    if t > 2.0 && t < 2.5
        % F_dist = [0; 80; 0]; % 侧向推力示例
    end
    
    % 动力学方程
    acc_lin = (F_total + F_dist) / p.m + [0; 0; -p.g];
    acc_ang = p.Ibody \ (Tau_total + Tau_dist);
    
    % 状态更新 (欧拉积分)
    x_next = x_curr;
    x_next(1:3)   = x_curr(1:3)   + R_z * omega * dt; 
    x_next(4:6)   = x_curr(4:6)   + vel * dt;         
    x_next(7:9)   = x_curr(7:9)   + acc_ang * dt;     
    x_next(10:12) = x_curr(10:12) + acc_lin * dt;     
    

    % 计算当前 X 位置对应的地面高度
    current_x = x_next(4);
    terrain_z = get_terrain_height(current_x, p);
    
    % 如果 Z 小于地形高度，强制落地
    if x_next(6) < terrain_z
        x_next(6) = terrain_z;
        x_next(12) = 0; % 垂直速度归零
    end
end

function contact = get_gait_contact(t, p)
    % 辅助函数: 计算 Trot 步态接触状态
    T = p.T_gait;
    phase = mod(t, T) / T;
    contact = zeros(4, 1);
    % FL(1) & HR(4)
    if phase < 0.5, contact(1) = 1; contact(4) = 1; end
    % FR(2) & HL(3)
    if phase >= 0.5, contact(2) = 1; contact(3) = 1; end
end

function z = get_terrain_height(x, p)
    % 计算地形高度：平地 -> 斜坡
    if x < p.slope_start_x
        z = 0;
    else
        % z = (x - start) * tan(alpha)
        z = (x - p.slope_start_x) * tan(p.slope_rad);
    end
end