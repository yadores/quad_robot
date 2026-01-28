function [f_opt, x_ref_preview] = mpc_controller(x_curr, contact_sequence, feet_pos_curr, p)
    % MPC_CONTROLLER_V4: 针对崎岖地形优化的 LTV-MPC
    % 包含：盲走适应策略 + 全姿态动力学修正 + 真实力臂计算
    
    %% 0. 初始化
    N = p.N; 
    dt = p.dt_mpc; 
    nx = 12; 
    nu = 12;
    
    %% 1. 生成参考轨迹 (盲走适应策略 - 保持不变)
    x_ref = zeros(nx, N);
    
    % --- 盲走核心：基于足端高度的自适应参考 ---
    contact_now = contact_sequence(:, 1);
    if sum(contact_now) > 0
        z_feet_contact = feet_pos_curr(3, contact_now == 1);
        current_ground_Z = mean(z_feet_contact);
    else
        current_ground_Z = x_curr(6) - p.h_des;
    end
    
    current_yaw = x_curr(3); 
    
    for k = 1:N
        t_pred = k * dt;
        
        % 位置参考
        x_ref(4, k) = x_curr(4) + p.v_x_des * t_pred; % X轴匀速前进
        x_ref(5, k) = x_curr(5);                      % Y轴保持
        x_ref(6, k) = current_ground_Z + p.h_des;     % Z轴：足端基准 + 期望身高
        
        % 姿态参考
        x_ref(1, k) = 0;            % Roll 归零
        x_ref(2, k) = 0;            % Pitch 归零 (爬坡时 MPC 会产生抗重力力矩)
        x_ref(3, k) = current_yaw;  % Yaw 锁定
        
        % 速度参考
        x_ref(7:9, k) = 0; 
        x_ref(10, k) = p.v_x_des;
        x_ref(11, k) = 0; 
        x_ref(12, k) = 0; 
    end
    x_ref_preview = x_ref(:, 1);

    %% 2. 离散动力学 (【关键修改：方案A & B】)
    
    % --- 修改 1：计算真实的力臂 r ---
    % 原代码使用的是 hip_offset 近似，这里改用真实足端位置
    % r = p_foot - p_com
    r_feet_real = zeros(3, 4);
    com_pos = x_curr(4:6);
    
    for i = 1:4
        % 使用传入的 feet_pos_curr (世界坐标)
        r_feet_real(:, i) = feet_pos_curr(:, i) - com_pos;
    end
    
    % 获取线性化动力学矩阵 (传入真实力臂)
    [Ad, Bd, gd] = get_linear_dynamics(x_curr, r_feet_real, p);
    
    %% 3. 构建 QP 矩阵 (标准 MPC 构建，无变化)
    Phi = zeros(nx*N, nx); Gamma = zeros(nx*N, nu*N); G_vec = zeros(nx*N, 1);
    A_pow = eye(nx);
    for i = 1:N
        A_pow = Ad * A_pow; 
        Phi((i-1)*nx+1 : i*nx, :) = A_pow;
        for j = 1:i
            if i == j, term = Bd; else, term = (Ad^(i-j)) * Bd; end
            Gamma((i-1)*nx+1 : i*nx, (j-1)*nu+1 : j*nu) = term;
        end
        g_term = zeros(nx, 1);
        for k_step = 0:i-1, g_term = g_term + (Ad^k_step) * gd; end
        G_vec((i-1)*nx+1 : i*nx) = g_term;
    end
    
    %% 4. 目标函数 (Cost Function)
    Q_bar = kron(eye(N), p.Q); 
    R_bar = kron(eye(N), p.R);
    X_ref_vec = reshape(x_ref, [], 1);
    Delta = Phi * x_curr + G_vec - X_ref_vec;
    H = Gamma' * Q_bar * Gamma + R_bar; 
    H = (H + H') / 2; 
    f = Gamma' * Q_bar * Delta;
    
    %% 5. 约束 (Constraints)
    n_constr_per_step = 20; 
    A_ineq = zeros(n_constr_per_step * N, nu*N); 
    b_ineq = zeros(n_constr_per_step * N, 1);
    lb = -inf(nu*N, 1); ub = inf(nu*N, 1); mu = p.mu;
    
    for k = 1:N
        contact_k = contact_sequence(:, k); 
        for leg = 1:4
            u_idx = (k-1)*nu + (leg-1)*3; cols = u_idx + (1:3); row = (k-1)*20 + (leg-1)*5;
            if contact_k(leg) == 1
                % 摩擦锥
                A_ineq(row+1, cols) = [1, 0, -mu]; A_ineq(row+2, cols) = [-1, 0, -mu];
                A_ineq(row+3, cols) = [0, 1, -mu]; A_ineq(row+4, cols) = [0, -1, -mu];
                % 最大力
                A_ineq(row+5, cols) = [0, 0, 1]; b_ineq(row+5) = p.f_max;
                % 最小力
                lb(cols(3)) = 0; ub(cols(3)) = p.f_max;
            else
                lb(cols) = 0; ub(cols) = 0;
            end
        end
    end
    
    %% 6. 求解
    options = optimoptions('quadprog', 'Display', 'off');
    try
        [U_opt, ~, exitflag] = quadprog(H, f, A_ineq, b_ineq, [], [], lb, ub, [], options);
    catch, exitflag = -1; end
    
    if exitflag == 1
        f_opt = U_opt(1:12); 
    else
        f_opt = zeros(12, 1); 
        f_opt(3:3:12) = p.m * 9.81 / 4; 
    end
end

%% 辅助函数：获取离散动力学矩阵 (【关键修改：方案A】)
function [Ad, Bd, gd] = get_linear_dynamics(x, r_feet, p)
    % 提取欧拉角
    phi = x(1); theta = x(2); psi = x(3);
    
    % --- 修改 2：使用全姿态旋转矩阵 (Full Rotation Matrix) ---
    % 解决 V3 中“小角度假设”在爬坡时的失效问题
    % R = Rz(psi) * Ry(theta) * Rx(phi)
    Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    R_body_to_world = Rz * Ry * Rx;
    
    % 更新世界坐标系下的惯性张量 (Scheme A: Real-time Inertia Update)
    I_world = R_body_to_world * p.Ibody * R_body_to_world';
    I_inv = inv(I_world);
    
    % 连续时间矩阵 Ac
    Ac = zeros(12); 
    
    % 角速度映射：在小角度下，欧拉角变化率 ≈ 角速度
    % 为了保持线性 MPC 的简洁性，这里通常只保留 Rz 或者 I
    % 但为了更精确，我们可以用当前的 R_body_to_world 近似
    Ac(1:3, 7:9) = R_body_to_world;       
    
    Ac(4:6, 10:12) = eye(3);  % Velocity
    
    % 连续时间矩阵 Bc
    Bc = zeros(12);
    for i = 1:4
        r = r_feet(:, i); % 这里使用的是传入的真实力臂 r_feet_real
        r_skew = [0 -r(3) r(2); r(3) 0 -r(1); -r(2) r(1) 0];
        
        % I_inv * (r x f)
        Bc(7:9, (i-1)*3+1:i*3) = I_inv * r_skew; 
        % f / m
        Bc(10:12, (i-1)*3+1:i*3) = eye(3)/p.m;
    end
    
    % 离散化
    Ad = eye(12) + Ac * p.dt_mpc; 
    Bd = Bc * p.dt_mpc; 
    gd = zeros(12, 1); 
    gd(10:12) = [0;0;-9.81] * p.dt_mpc;
end