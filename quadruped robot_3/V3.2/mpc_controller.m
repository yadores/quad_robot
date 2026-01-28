function [f_opt, x_ref_preview] = mpc_controller(x_curr, contact_sequence, p)
    % mpc_controller: 基于 SRBD 模型的 MPC 求解器
    
    N = p.N; dt = p.dt_mpc; 
    nx = 12; nu = 12;
    
    %% 1. 生成参考轨迹 (Reference Trajectory)
    % 机器人应贴合地形变化
    x_ref = zeros(nx, N);
    current_yaw = x_curr(3); 
    
    for k = 1:N
        % 姿态: 保持当前 Yaw
        x_ref(1, k) = 0; 
        x_ref(2, k) = 0;
        x_ref(3, k) = current_yaw; 
        
        % 位置预测 X
        pred_x = x_curr(4) + p.v_x_des * k * dt;
        x_ref(4, k) = pred_x; 
        x_ref(5, k) = 0;                  
        
        % Z 轴高度 = 期望机身高度 + 地形高度
        terrain_z = 0;
        if pred_x > p.slope_start_x
            terrain_z = (pred_x - p.slope_start_x) * tan(p.slope_rad);
        end
        x_ref(6, k) = p.h_des + terrain_z;            
        
        % 速度
        x_ref(7:9, k) = 0;                  
        x_ref(10, k)  = p.v_x_des;          
        x_ref(11:12,k)= 0;                  
    end
    x_ref_preview = x_ref(:, 1);

    %% 2. 离散动力学 (Ad, Bd)
    % [注] 文档提到 "小角度假设下 T approx I"，因此保持标准 SRBD 模型
    % 不在 Ad/Bd 中显式加入斜坡重力分量，而是依靠反馈控制(Feedback)来补偿误差
    
    psi = x_curr(3); 
    R_yaw = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0, 0, 1];
    r_feet_approx = zeros(3, 4);
    for i = 1:4
        hip_pos = R_yaw * p.hip_offset(:, i); 
        r_feet_approx(:, i) = hip_pos; 
    end
    
    [Ad, Bd, gd] = get_linear_dynamics(x_curr, r_feet_approx, p);
    
    %% 3. 构建 QP 预测矩阵 (Y = Phi*x0 + Gamma*U + G)
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
        g_term = zeros(nx, 1);
        for k_step = 0:i-1, g_term = g_term + (Ad^k_step) * gd; end
        G_vec((i-1)*nx+1 : i*nx) = g_term;
    end
    
    %% 4. 构造 QP 标准型
    Q_bar = kron(eye(N), p.Q);
    R_bar = kron(eye(N), p.R);
    X_ref_vec = reshape(x_ref, [], 1);
    
    Delta = Phi * x_curr + G_vec - X_ref_vec;
    
    H = Gamma' * Q_bar * Gamma + R_bar;
    H = (H + H') / 2; 
    f = Gamma' * Q_bar * Delta;
    
    %% 5. 约束条件
    n_constr_per_step = 5 * 4; 
    A_ineq = zeros(n_constr_per_step * N, nu*N);
    b_ineq = zeros(n_constr_per_step * N, 1);
    lb = -inf(nu*N, 1); 
    ub = inf(nu*N, 1);
    mu = p.mu;
    
    for k = 1:N
        contact_k = contact_sequence(:, k); 
        for leg = 1:4
            u_idx_base = (k-1)*nu + (leg-1)*3; 
            cols = u_idx_base + (1:3);         
            row_base = (k-1)*n_constr_per_step + (leg-1)*5;
            
            if contact_k(leg) == 1
                % 摩擦锥约束 (为保持线性化简单，此处仍沿用 Z 轴对齐的锥)
                % 若坡度很大，需旋转锥体；对于 8 度，此近似通常可行
                A_ineq(row_base+1, cols) = [1, 0, -mu];
                A_ineq(row_base+2, cols) = [-1, 0, -mu];
                A_ineq(row_base+3, cols) = [0, 1, -mu];
                A_ineq(row_base+4, cols) = [0, -1, -mu];
                A_ineq(row_base+5, cols) = [0, 0, 1];
                b_ineq(row_base+5) = p.f_max;
                lb(cols(3)) = 0; 
                ub(cols(3)) = p.f_max;
            else
                lb(cols) = 0;
                ub(cols) = 0;
            end
        end
    end
    
    %% 6. 求解 QP
    options = optimoptions('quadprog', 'Display', 'off', 'Algorithm', 'interior-point-convex');
    try
        [U_opt, ~, exitflag] = quadprog(H, f, A_ineq, b_ineq, [], [], lb, ub, [], options);
    catch
        exitflag = -1;
    end
    
    if exitflag == 1
        f_opt = U_opt(1:12); 
    else
        f_opt = zeros(12, 1); 
    end
end

function [Ad, Bd, gd] = get_linear_dynamics(x, r_feet, p)
    psi = x(3);
    R_z = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    I_world = R_z * p.Ibody * R_z';
    I_inv = inv(I_world);
    
    Ac = zeros(12, 12);
    Ac(1:3, 7:9) = R_z;       
    Ac(4:6, 10:12) = eye(3); 
    
    Bc = zeros(12, 12);
    for i = 1:4
        r = r_feet(:, i); 
        r_skew = [0 -r(3) r(2); r(3) 0 -r(1); -r(2) r(1) 0];
        Bc(7:9, (i-1)*3+1:i*3) = I_inv * r_skew;  
        Bc(10:12, (i-1)*3+1:i*3) = eye(3) / p.m; 
    end
    
    dt = p.dt_mpc;
    Ad = eye(12) + Ac * dt;
    Bd = Bc * dt;
    gd = zeros(12, 1);
    gd(10:12) = [0;0;-9.81] * dt; 
end