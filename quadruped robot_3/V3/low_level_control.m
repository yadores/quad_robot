function [contact_state, x_next, contact_seq_mpc] = low_level_control(t, x_curr, f_cmd, p)
    % low_level_control: 包含步态调度和物理动力学演化
    % 输入: 时间, 当前状态, MPC指令力, 参数
    % 输出: 接触状态(当前), 下一刻状态, 接触序列(未来N步供MPC用)
    
    %% 1. 计算当前接触状态
    contact_state = get_gait_contact(t, p);
    
    %% 2. 预测未来 N 步的接触状态 (供 MPC 使用)
    % 这一步非常重要：MPC 需要知道未来什么时候抬腿，什么时候落腿
    contact_seq_mpc = zeros(4, p.N);
    for k = 1:p.N
        % 预测时间点: 当前时间 + k * MPC步长
        t_pred = t + (k-1) * p.dt_mpc; 
        contact_seq_mpc(:, k) = get_gait_contact(t_pred, p);
    end
    
    %% 3. 物理对象仿真 (Robot Plant Dynamics)
    dt = p.dt_sim;
    
    % 提取状态
    vel = x_curr(10:12);
    omega = x_curr(7:9);
    psi = x_curr(3);
    
    % 旋转矩阵
    R_z = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    
    % 计算合力与合力矩
    F_total = zeros(3, 1);
    Tau_total = zeros(3, 1);
    
    for i = 1:4
        if contact_state(i) == 1
            % 只有在支撑相，力才能作用于机身
            f_leg = f_cmd((i-1)*3+1 : i*3);
            
            % 力臂 (这里使用 hip_offset 简化计算)
            r = R_z * p.hip_offset(:, i);
            
            F_total = F_total + f_leg;
            Tau_total = Tau_total + cross(r, f_leg);
        end
    end
    
    % ==========================================
    % 【新增】 添加外部干扰 (External Disturbance)
    % ==========================================
    F_dist = zeros(3, 1);
    Tau_dist = zeros(3, 1);
    
    % 设定干扰触发时间：在 2.0s 到 2.5s 之间
    %if t > 2.0 && t < 2.5
        % 场景1：侧向推力 (模拟被人从侧面踢了一脚)
        % 给 Y 轴施加 80N 的力
    %    F_dist = [0; 80; 0]; 
        
        % 场景2：(可选) 同时施加一点旋转力矩，让它转起来
        % Tau_dist = [0; 0; 5]; 
    %end
    
    % 将干扰加入动力学方程
    % F_total 是足端力，F_dist 是外力
    acc_lin = (F_total + F_dist) / p.m + [0; 0; -p.g];
    
    % 力矩同理
    acc_ang = p.Ibody \ (Tau_total + Tau_dist);
    % ==========================================

    
    % 状态更新 (欧拉积分)
    x_next = x_curr;
    x_next(1:3)   = x_curr(1:3)   + R_z * omega * dt; 
    x_next(4:6)   = x_curr(4:6)   + vel * dt;         
    x_next(7:9)   = x_curr(7:9)   + acc_ang * dt;     
    x_next(10:12) = x_curr(10:12) + acc_lin * dt;     
    
    % 地面碰撞约束
    if x_next(6) < 0
        x_next(6) = 0;
        x_next(12) = 0;
    end
end

function contact = get_gait_contact(t, p)
    % 辅助函数: 计算 Trot 步态接触状态
    T = p.T_gait;
    phase = mod(t, T) / T;
    
    contact = zeros(4, 1);
    
    % FL(1) & HR(4)
    if phase < 0.5
        contact(1) = 1;
        contact(4) = 1;
    end
    
    % FR(2) & HL(3)
    if phase >= 0.5
        contact(2) = 1;
        contact(3) = 1;
    end
end