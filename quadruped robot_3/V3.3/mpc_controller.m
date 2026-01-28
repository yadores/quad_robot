function [f_opt, x_ref_preview] = mpc_controller(x_curr, contact_sequence, p)
    % mpc_controller: 基于 SRBD 模型的 MPC 求解器
    % 输入: 
    %   x_curr: 12x1 当前状态
    %   contact_sequence: 4xN 未来N步的接触状态矩阵 (0或1) [关键修改]
    %   p: 参数结构体
    
    N = p.N; dt = p.dt_mpc; 
    nx = 12; nu = 12;
    
    %% 1. 生成参考轨迹 (Reference Trajectory)
    %  机器人应贴合参考轨迹
    x_ref = zeros(nx, N);
    
    % 获取当前偏航角，作为参考基准，防止 MPC 强行把头扭回 0 度
    current_yaw = x_curr(3); 
    
    for k = 1:N
        % 姿态: 保持当前 Yaw，Roll/Pitch 归零
        x_ref(1, k) = 0; 
        x_ref(2, k) = 0;
        x_ref(3, k) = current_yaw; 
        
        % 位置: X 随速度积分，Y 保持，Z 恒定
        x_ref(4, k)   = x_curr(4) + p.v_x_des * k * dt; 
        x_ref(5, k)   = 0;          % 强制归零 (由 MPC 产生侧向力把它拉回来)         
        x_ref(6, k)   = p.h_des;            
        
        % 速度
        x_ref(7:9, k) = 0;                  
        x_ref(10, k)  = p.v_x_des;          
        x_ref(11:12,k)= 0;                  
    end
    x_ref_preview = x_ref(:, 1);

    %% 2. 离散动力学 (Ad, Bd)
    % 欧拉离散化 SRBD 模型
    % 这里的 r_feet 使用简单的 hip_offset 近似，理想情况应使用真实足端位置
    psi = x_curr(3); 
    R_yaw = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0, 0, 1];
    r_feet_approx = zeros(3, 4);
    for i = 1:4
        % 假设足端在当前机身下的标称位置
        hip_pos = R_yaw * p.hip_offset(:, i); 
        r_feet_approx(:, i) = hip_pos; % 相对于 CoM 的向量
    end
    
    [Ad, Bd, gd] = get_linear_dynamics(x_curr, r_feet_approx, p);
    
    %% 3. 构建 QP 预测矩阵 (Y = Phi*x0 + Gamma*U + G)
    % 状态预测方程
    Phi = zeros(nx*N, nx);
    Gamma = zeros(nx*N, nu*N);
    G_vec = zeros(nx*N, 1);
    
    A_pow = eye(nx);
    for i = 1:N
        A_pow = Ad * A_pow;
        Phi((i-1)*nx+1 : i*nx, :) = A_pow;
        
        for j = 1:i
            if i == j, term = Bd; else, term = (Ad^(i-j)) * Bd; end
            Gamma((i-1)*nx+1 : i*nx, (j-1)*nu+1 : j*nu) = term;
        end
        
        % 重力项
        g_term = zeros(nx, 1);
        for k_step = 0:i-1, g_term = g_term + (Ad^k_step) * gd; end
        G_vec((i-1)*nx+1 : i*nx) = g_term;
    end
    
    %% 4. 构造 QP 标准型 (min 1/2 U'HU + f'U)
    Q_bar = kron(eye(N), p.Q);
    R_bar = kron(eye(N), p.R);
    X_ref_vec = reshape(x_ref, [], 1);
    
    % 误差向量 Delta = Phi*x0 + G - X_ref
    Delta = Phi * x_curr + G_vec - X_ref_vec;
    
    H = Gamma' * Q_bar * Gamma + R_bar;
    H = (H + H') / 2; % 保证对称正定
    f = Gamma' * Q_bar * Delta;
    
    %% 5. 约束条件 (摩擦锥 + 动态接触逻辑)
    % [cite: 131-135] 摩擦锥约束
    
    % 不等式约束 A*U <= b
    % 每个时间步 k, 每条腿 4 个摩擦锥平面 + 1 个最大力限制 = 5 个约束
    n_constr_per_step = 5 * 4; 
    A_ineq = zeros(n_constr_per_step * N, nu*N);
    b_ineq = zeros(n_constr_per_step * N, 1);
    
    % 上下界约束 (用于强制摆动腿力为 0)
    lb = -inf(nu*N, 1); 
    ub = inf(nu*N, 1);
    
    mu = p.mu;
    
    for k = 1:N
        % [关键] 使用预测视域内的接触状态，而不是当前的
        contact_k = contact_sequence(:, k); 
        
        for leg = 1:4
            u_idx_base = (k-1)*nu + (leg-1)*3; % 当前变量在 U 中的起始索引
            cols = u_idx_base + (1:3);         % fx, fy, fz 的列索引
            
            row_base = (k-1)*n_constr_per_step + (leg-1)*5;
            
            if contact_k(leg) == 1
                % --- 支撑相约束 ---
                % 1. 摩擦锥 (金字塔近似)
                % fx - mu*fz <= 0
                A_ineq(row_base+1, cols) = [1, 0, -mu];
                % -fx - mu*fz <= 0
                A_ineq(row_base+2, cols) = [-1, 0, -mu];
                % fy - mu*fz <= 0
                A_ineq(row_base+3, cols) = [0, 1, -mu];
                % -fy - mu*fz <= 0
                A_ineq(row_base+4, cols) = [0, -1, -mu];
                
                % 2. 最大力约束 fz <= f_max
                A_ineq(row_base+5, cols) = [0, 0, 1];
                b_ineq(row_base+5) = p.f_max;
                
                % 3. 最小力约束 fz >= 0 (通过 lb 实现)
                lb(cols(3)) = 0; 
                ub(cols(3)) = p.f_max;
                
            else
                % --- 摆动相约束 ---
                % 强制力为 0
                lb(cols) = 0;
                ub(cols) = 0;
            end
        end
    end
    
    %% 6. 求解 QP
    options = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
    
    % 注意：Matlab quadprog 如果 H 不正定可能会报错，这里加个 try-catch 或正则化
    try
        [U_opt, ~, exitflag] = quadprog(H, f, A_ineq, b_ineq, [], [], lb, ub, [], options);
    catch
        exitflag = -1;
    end
    
    if exitflag == 1
        f_opt = U_opt(1:12); % 取第一步控制量 Receding Horizon
    else
        % 求解失败时的安全策略：维持重力补偿或归零
        % warning('MPC QP Failed, using zero force');
        f_opt = zeros(12, 1); 
    end
end

function [Ad, Bd, gd] = get_linear_dynamics(x, r_feet, p)
    % 辅助函数：SRBD 动力学线性化 [cite: 92]
    psi = x(3);
    R_z = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    
    % 世界坐标系下的惯性张量 I_world approx R * I_body * R'
    I_world = R_z * p.Ibody * R_z';
    I_inv = inv(I_world);
    
    % 连续时间 Ac
    Ac = zeros(12, 12);
    Ac(1:3, 7:9) = R_z;       % dTheta/dt approx R * omega (小角度假设下其实近似 I)
    Ac(4:6, 10:12) = eye(3);  % dPos/dt = v
    
    % 连续时间 Bc
    Bc = zeros(12, 12);
    for i = 1:4
        r = r_feet(:, i); % 足端相对于 CoM 的位置
        % r x f -> [r]x * f
        r_skew = [0 -r(3) r(2); r(3) 0 -r(1); -r(2) r(1) 0];
        
        Bc(7:9, (i-1)*3+1:i*3) = I_inv * r_skew;  % 角加速度贡献
        Bc(10:12, (i-1)*3+1:i*3) = eye(3) / p.m; % 线加速度贡献
    end
    
    % 欧拉离散化 [cite: 98-100]
    dt = p.dt_mpc;
    Ad = eye(12) + Ac * dt;
    Bd = Bc * dt;
    gd = zeros(12, 1);
    gd(10:12) = [0;0;-9.81] * dt; % 重力只影响线性速度
end