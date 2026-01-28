function optimize_planning_step2()
    % OPTIMIZE_PLANNING_STEP2
    % 解耦优化 Step 2: 规划层专项训练 (最终修复版)   
    
    clc; close all;
    
    %% 1. 定义搜索空间 (3维规划参数)
    vars = [
        % 1. 安全余量衰减速率 (遗忘因子)
        optimizableVariable('sm_decay',   [1e-6, 1e-4], 'Type','real', 'Transform','log') 
        
        % 2. 痛觉惩罚强度 (踩坑后增加多少余量)
        optimizableVariable('sm_penalty', [0.01, 0.10], 'Type','real')  
        
        % 3. 最大安全余量 (兜底上限)
        optimizableVariable('sm_max',     [0.02, 0.15], 'Type','real')   
    ];

    %% 2. 启动贝叶斯优化
    fprintf('=== Step 2: 规划层独立优化 (防弹版) ===\n');
    fprintf('锁定感知参数: Thresh=0.049m, BaseR=19.97\n');
    fprintf('目标: Scrape=0 + Energy=Min + Margin=Minimal\n');
    
    % 移除了非法的 ErrorHandling 参数
    results = bayesopt(@run_objective_planning_safe, vars, ...
        'Verbose', 1, ...
        'MaxObjectiveEvaluations', 100, ... 
        'PlotFcn', {@plotObjectiveModel, @plotMinObjective}, ...
        'AcquisitionFunctionName', 'expected-improvement-plus'); 

    %% 3. 保存结果
    best_params_plan = results.XAtMinObjective;
    min_objective = results.MinObjective;

    save_dir = 'C:\Users\渊域\Desktop\HS\quadruped robot_4\V_4.1_experiment_group_decouple';
    file_name = 'results_decoupled_step2_plan.mat';
    
    if ~exist(save_dir, 'dir'), mkdir(save_dir); end
    full_save_path = fullfile(save_dir, file_name);

    fprintf('\n=== Step 2 优化完成 ===\n');
    fprintf('最优总 Cost: %.4f\n', min_objective);
    fprintf('最优规划参数:\n');
    disp(best_params_plan);

    save(full_save_path, 'best_params_plan', 'min_objective');
    fprintf('结果已保存至: %s\n', full_save_path);
end

%% ============================================================
%  局部函数: 安全目标函数包装器 (防弹层)
% =============================================================
function total_cost = run_objective_planning_safe(opt_vars)
    try
        total_cost = run_objective_planning_impl(opt_vars);
    catch ME
        % 如果出错，打印错误信息，但不终止优化
        fprintf(2, '\n[警告] 仿真在参数 sm_decay=%e, sm_penalty=%.4f, sm_max=%.4f 处失败。\n', ...
            opt_vars.sm_decay, opt_vars.sm_penalty, opt_vars.sm_max);
        fprintf(2, '错误原因: %s\n', ME.message);
        fprintf(2, '出错位置: %s (行 %d)\n', ME.stack(1).name, ME.stack(1).line);
        
        % 返回一个巨大的惩罚值，告诉优化器"这组参数不行"
        total_cost = 10000.0; 
    end
end

%% ============================================================
%  局部函数: 实际的业务逻辑实现
% =============================================================
function total_cost = run_objective_planning_impl(opt_vars)
    % 1. 初始化参数
    p = init_params(); 
    
    % === A. 【关键】锁定感知层参数 (来自 Step 1 的最优结果) ===
    p.kf_base_R_scale = 19.974;   
    p.kf_thresh_step  = 0.049032; 
    p.kf_scale_step   = 0.99975;  
    p.kf_thresh_noise = 0.44052;
    p.kf_scale_noise  = 25.918;
    
    % 应用 Base R
    p.kf_R = (0.03^2) * p.kf_base_R_scale;
    
    % === B. 应用当前优化的规划参数 ===
    p.sm_decay   = opt_vars.sm_decay;
    p.sm_penalty = opt_vars.sm_penalty;
    p.sm_max     = opt_vars.sm_max;
    
    % 2. 运行仿真 
    x_curr = zeros(12, 1);
    x_curr(4) = 0; x_curr(6) = get_terrain_z(x_curr(4), 0, p) + 0.2; 
    t_span = 0:p.dt_sim:7.0;
    n_steps = length(t_span);
    
    log_scrape = 0;
    log_energy = 0;
    log_stability = 0;
    
    f_cmd = zeros(12, 1);
    feet_pos_planned = zeros(3, 4);
    for i=1:4, feet_pos_planned(:,i) = x_curr(4:6) + p.hip_offset(:,i); feet_pos_planned(3,i)=0; end
    swing_start_pos = feet_pos_planned;
    contact_prev = [1;1;1;1];
    vel_filtered = zeros(3, 1);
    mpc_counter = 0; ratio_mpc = round(p.dt_mpc / p.dt_sim);
    
    % --- 仿真循环 ---
    for k = 1:n_steps
        t = t_span(k);
        
        noise_vel = 0.05 * randn(6,1);
        vel_raw = x_curr(10:12) + noise_vel(4:6);
        vel_filtered = 0.15 * vel_raw + 0.85 * vel_filtered;
        x_sensed = x_curr; x_sensed(10:12) = vel_filtered;
        
        % 控制层 (这里是 6 个输出，正确)
        [~, feet_pos_new, swing_new, contact_curr, contact_seq, scrape_flag] = ...
            low_level_control(t, x_sensed, f_cmd, p, feet_pos_planned, swing_start_pos, contact_prev);
            
        if mod(mpc_counter, ratio_mpc) == 0
            [f_cmd, ~] = mpc_controller(x_sensed, contact_seq, feet_pos_new, p);
        end
        mpc_counter = mpc_counter + 1;
        
        % 物理层 (【修正完毕】这里删除了第7个 ~，现在是 6 个输出)
        % [x_next, feet_true, ~, ~, ~, ~]  <-- 对应 low_level_control 的 6 个输出
        [x_next, feet_true, ~, ~, ~, ~] = ...
            low_level_control(t, x_curr, f_cmd, p, feet_pos_new, swing_new, contact_prev);
            
        % 累加 Cost 数据
        log_scrape = log_scrape + sum(scrape_flag);
        log_energy = log_energy + sum(f_cmd.^2, 'all');
        log_stability = log_stability + x_curr(1)^2 + x_curr(2)^2; 
        
        % 状态更新
        x_curr = x_next; feet_pos_planned = feet_pos_new; swing_start_pos = swing_new; contact_prev = contact_curr; 
        
        % 摔倒提前终止
        if x_curr(6) < 0.1
            total_cost = 10000;
            return;
        end
    end
    
    % 3. 计算总 Cost
    cost_safety = log_scrape * 3.5; 
    cost_energy = log_energy / n_steps * 0.001;
    cost_stability = (log_stability / n_steps) * 500;
    
    % 极简主义惩罚 (引导 sm_max 变小)
    cost_minimalism = opt_vars.sm_max * 5.0; 
    
    total_cost = cost_safety + cost_energy + cost_stability + cost_minimalism;
end