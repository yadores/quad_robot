% START_SCRIPT.m
% 启动 8 维全物理蒸馏联合贝叶斯优化
clc; clear; close all;

%% 1. 定义优化变量 (8维全参数空间)
vars = [
    % === 感知层 (5维) ===
    % 1. 噪声阈值
    optimizableVariable('kf_thresh_noise', [0.0, 1.0], 'Type','real')   
    % 2. 噪声放大倍数
    optimizableVariable('kf_scale_noise',  [1.0, 500.0], 'Type','real')   
    % 3. 台阶阈值
    optimizableVariable('kf_thresh_step',  [0.0, 0.20], 'Type','real')   
    % 4. 台阶缩小倍数
    optimizableVariable('kf_scale_step',   [0.001, 1.0], 'Type','real')
    % 5. 基准R缩放因子 
    optimizableVariable('kf_base_R_scale', [1.0, 100.0], 'Type','real')

    % === 规划层 (3维) ===
    % 6. 遗忘速率
    optimizableVariable('sm_decay',   [1e-6, 1e-1], 'Type','real', 'Transform','log') 
    % 7. 痛觉惩罚
    optimizableVariable('sm_penalty', [0.0, 0.1], 'Type','real')  
    % 8. 最大余量
    optimizableVariable('sm_max',     [0.0, 0.30], 'Type','real')   
];

%% 2. 开始优化
fprintf('=== 开始 8 维全物理蒸馏联合优化 ===\n');
fprintf('目标函数: Task(Scrape+Energy+Stab) + Distill(TV+Param)\n');
fprintf('注意: 不包含 RMSE (模拟无真值监督)\n');

results = bayesopt(@run_main_joint, vars, ...
    'Verbose', 1, ...
    'MaxObjectiveEvaluations', 100, ... 
    'PlotFcn', {@plotObjectiveModel, @plotMinObjective}, ...
    'AcquisitionFunctionName', 'expected-improvement-plus');

%% 3. 保存结果
% 请根据实际路径修改
save_dir = 'C:\Users\渊域\Desktop\HS\quadruped robot_4\V_4.1_experiment_group_joint\result';
if ~exist(save_dir, 'dir'), mkdir(save_dir); end

filename = 'results_joint_8D_distilled.mat';
full_path = fullfile(save_dir, filename);
save(full_path, 'results');

fprintf('结果已保存至: %s\n', full_path);