function optimize_perception_step1()
    % OPTIMIZE_PERCEPTION_STEP1
    % 解耦优化 Step 1: 感知层专项训练 (TV正则化 + Base R 动态调整版)
    % 核心逻辑: 
    %   1. 引入 kf_base_R_scale: 让滤波器在平地"装迟钝"(高R)，消除抖动。
    %   2. 配合 kf_scale_step: 让滤波器在台阶"变激进"(低R)，消除滞后。
    %   3. 物理隔离阈值: 保证逻辑不短路。
    
    clc; close all;
    
    %% 1. 定义搜索空间 (5维优化)
    vars = [
        % 允许将基础测量噪声方差放大 1~20 倍
        % 作用: 在平坦地面显著增加平滑度 (Smoothness)
        optimizableVariable('kf_base_R_scale', [1.0, 20.0], 'Type','real')

        % === 原有的4个自适应参数 (带物理隔离) ===
        % 1. 台阶阈值: 限制在 4cm ~ 12cm (物理台阶范围)
        optimizableVariable('kf_thresh_step',  [0.04, 0.12], 'Type','real')   
        
        % 2. 噪声阈值: 强制 > 15cm (物理隔离，防止吞掉台阶)
        optimizableVariable('kf_thresh_noise', [0.15, 0.60], 'Type','real')   
        
        % 3. 噪声惩罚倍率
        optimizableVariable('kf_scale_noise',  [1.0, 500.0], 'Type','real')   
        
        % 4. 台阶响应倍率 (越小越激进)
        optimizableVariable('kf_scale_step',   [0.001, 1.0], 'Type','real')   
    ];

    %% 2. 启动贝叶斯优化
    fprintf('=== Step 1: 感知层独立优化 (Final Version) ===\n');
    fprintf('目标: 寻找最佳参数\n');
    
    results = bayesopt(@run_objective_tv, vars, ...
        'Verbose', 1, ...
        'MaxObjectiveEvaluations', 100, ... % 建议跑 100 次以充分收敛
        'PlotFcn', {@plotObjectiveModel, @plotMinObjective}, ...
        'AcquisitionFunctionName', 'expected-improvement-plus');

    %% 3. 保存结果
    best_params_kf = results.XAtMinObjective;
    min_objective = results.MinObjective;

    save_dir = 'C:\Users\渊域\Desktop\HS\quadruped robot_4\V_4.1_experiment_group _decouple';
    file_name = 'results_decoupled_step1_kf.mat';
    
    if ~exist(save_dir, 'dir'), mkdir(save_dir); end
    full_save_path = fullfile(save_dir, file_name);

    fprintf('\n=== 优化完成 ===\n');
    fprintf('最优总 Cost: %.4f\n', min_objective);
    fprintf('最优参数组合:\n');
    disp(best_params_kf);

    save(full_save_path, 'best_params_kf', 'min_objective');
    fprintf('结果已保存至: %s\n', full_save_path);

    %% 4. 自动验证绘图
    fprintf('正在生成验证图...\n');
    verify_result_tv(best_params_kf);
end

%% ============================================================
%  局部函数: 目标函数 (应用 Base R Scale)
% =============================================================
function cost = run_objective_tv(opt_vars)
    % 1. 固定随机种子
    rng(42); 
    
    % 2. 解析参数
    p = init_params(); 
    
    % === [核心修改] 应用基础 R 缩放 ===
    % 原始 R 是 0.03^2 (9e-4)，现在乘上优化器算出来的倍率
    % 如果 opt_vars.kf_base_R_scale = 10，则 R 变为 0.009 (相当于告诉KF噪声有近10cm)
    p.kf_R = (0.03^2) * opt_vars.kf_base_R_scale; 
    
    % 其他自适应参数
    p.kf_thresh_noise = opt_vars.kf_thresh_noise;
    p.kf_scale_noise  = opt_vars.kf_scale_noise;
    p.kf_thresh_step  = opt_vars.kf_thresh_step;
    p.kf_scale_step   = opt_vars.kf_scale_step;
    
    % 3. 准备考题
    x_test = 0:0.01:6.0; 
    n_points = length(x_test);
    step_stride = 0.3; 
    idx_stride = floor(step_stride / 0.01);
    touchdown_indices = idx_stride : idx_stride : n_points;
    
    % 4. 考试过程
    z_est = 0; P_est = 1.0;
    squared_error_sum = 0;
    valid_checks = 0;
    jitter_sum = 0;      
    z_est_prev = 0;      
    
    for i = 1:n_points
        x_curr = x_test(i);
        [z_meas, q_flag] = get_sensor_reading(x_curr, 0, p);
        
        % 运行 KF
        [z_new, P_new] = adaptive_kalman_filter(z_est, P_est, z_meas, q_flag, p);
        
        % 计算抖动 (TV)
        if i > 1
            step_change = abs(z_new - z_est_prev);
            jitter_sum = jitter_sum + step_change;
        end
        
        % 更新状态
        z_est_prev = z_new; 
        z_est = z_new; 
        P_est = P_new;
        
        % 计算 RMSE
        if ismember(i, touchdown_indices)
            z_true = get_terrain_z(x_curr, 0, p); 
            error = z_est - z_true;
            squared_error_sum = squared_error_sum + error^2;
            valid_checks = valid_checks + 1;
        end
    end
    
    % 5. 计算得分
    if valid_checks > 0
        rmse = sqrt(squared_error_sum / valid_checks);
    else
        rmse = 10.0;
    end
    
    avg_jitter = jitter_sum / n_points;
    
    % 融合 Cost: RMSE + 0.2 * TV
    lambda_tv = 0.2; 
    cost = rmse + lambda_tv * avg_jitter;
    
    if isnan(cost) || cost > 20.0, cost = 20.0; end
    % 在 run_objective_tv 函数末尾添加：
    violation = max(0, opt_vars.kf_thresh_step - 0.08); % 超过8cm就开始罚
    cost = cost + violation * 100.0; % 重罚
end

function verify_result_tv(best_params_table)
    % 验证绘图
    p = init_params();
    
    % === [必须同步修改] ===
    p.kf_R = (0.03^2) * best_params_table.kf_base_R_scale; 
    
    p.kf_thresh_noise = best_params_table.kf_thresh_noise;
    p.kf_scale_noise  = best_params_table.kf_scale_noise;
    p.kf_thresh_step  = best_params_table.kf_thresh_step;
    p.kf_scale_step   = best_params_table.kf_scale_step;
    
    x_test = 0:0.01:6.0;
    n = length(x_test);
    z_true_trace = zeros(1,n);
    z_meas_trace = zeros(1,n);
    z_est_trace  = zeros(1,n);
    z_est = 0; P_est = 1.0;
    
    rng(42); 
    
    for i = 1:n
        x = x_test(i);
        z_true_trace(i) = get_terrain_z(x, 0, p);
        [z_meas, q] = get_sensor_reading(x, 0, p);
        z_meas_trace(i) = z_meas;
        [z_new, P_new] = adaptive_kalman_filter(z_est, P_est, z_meas, q, p);
        z_est = z_new; P_est = P_new;
        z_est_trace(i) = z_est;
    end
    
    figure('Name','Step 1 Verification (Final)', 'Color', 'w');
    plot(x_test, z_true_trace, 'k-', 'LineWidth', 2, 'DisplayName','Ground Truth'); hold on;
    plot(x_test, z_meas_trace, 'g.', 'MarkerSize', 5, 'DisplayName','Noisy Sensor');
    plot(x_test, z_est_trace, 'r-', 'LineWidth', 2, 'DisplayName','Optimized KF');
    
    title('最终优化结果: Base R Scale + TV Regularization');
    xlabel('Distance (m)'); ylabel('Height (m)');
    legend; grid on;
end