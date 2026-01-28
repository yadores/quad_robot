function [z_est, P_new] = adaptive_kalman_filter(z_pred, P_prev, z_meas, quality_flag, p)
    % ADAPTIVE_KALMAN_FILTER_V2: 动态自适应版 (修正版)
    
    %% 1. 预测阶段 (Predict)
    P_pred = P_prev + p.kf_Q;
    
    %% 2. 动态权重计算
    if quality_flag == 1
        % 【场景 1】: 传感器丢包/盲区 -> 只信预测
        z_est = z_pred;
        P_new = P_pred;
        
    else
        % 计算残差
        innovation = abs(z_meas - z_pred);
        
        % === 智能动态 R 策略 (修正后) ===
        % 使用优化器传入的 p 参数
        
        % 1. 检查是否为"离谱噪声"
        if innovation > p.kf_thresh_noise 
            % 【场景 2】: 认为是噪声 -> R 变大 (拒绝更新)
            R_dynamic = p.kf_R * p.kf_scale_noise;
            
        % 2. 检查是否为"台阶"
        elseif innovation > p.kf_thresh_step
            % 【场景 3】: 认为是台阶 -> R 变小 (激进跟进)
            R_dynamic = p.kf_R * p.kf_scale_step;
            
        else
            % 【场景 4】: 普通波动 -> 标准 R
            R_dynamic = p.kf_R;
        end
        
        % === 标准更新公式 ===
        K = P_pred / (P_pred + R_dynamic);
        z_est = z_pred + K * (z_meas - z_pred);
        P_new = (1 - K) * P_pred;
    end
end