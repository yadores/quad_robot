function [z_est, P_new] = adaptive_kalman_filter(z_pred, P_prev, z_meas, quality_flag, p)
    % ADAPTIVE_KALMAN_FILTER_V2: 动态自适应版
    % 结合了“置信度”与“残差驱动”的智能融合算法
    
    %% 1. 预测阶段 (Predict)
    % 预测不确定性增加 (过程噪声)
    P_pred = P_prev + p.kf_Q;
    
    %% 2. 动态权重计算 (The "Smart" Part)
    if quality_flag == 1
        % 【场景 1】: 传感器丢包/盲区
        % 策略：完全不信观测，只信预测 (盲走模式)
        z_est = z_pred;
        P_new = P_pred;
        
    else
        % 计算"残差"(Innovation)：观测值和预测值差了多少？
        innovation = abs(z_meas - z_pred);
        
        % === 智能动态 R 策略 ===
        % R 代表我们对传感器的"怀疑程度"。R 越小，权重 K 越大。
        
        if innovation > 0.30 
            % 【场景 2】: 离谱的跳变 (>30cm)
            % 策略：这极大概率是传感器故障或噪声。
            % 动作：将 R 放大 100 倍，拒绝更新，保持平滑。
            R_dynamic = p.kf_R * 100;
            
        elseif innovation > 0.04
            % 【场景 3】: 中等幅度的突变 (4cm - 30cm)
            % 策略：这很可能是真实的台阶！V4 项目的重点！
            % 动作：将 R 缩小为 1/10，强迫滤波器"激进"地跟上变化，不要磨磨蹭蹭。
            R_dynamic = p.kf_R * 0.1;
            
        else
            % 【场景 4】: 微小波动 (<4cm)
            % 策略：认为是平地上的传感器抖动。
            % 动作：使用标准 R，进行平滑滤波。
            R_dynamic = p.kf_R;
        end
        
        % === 标准更新公式 (代入动态 R) ===
        K = P_pred / (P_pred + R_dynamic);
        
        z_est = z_pred + K * (z_meas - z_pred);
        
        % 更新协方差
        P_new = (1 - K) * P_pred;
    end
end