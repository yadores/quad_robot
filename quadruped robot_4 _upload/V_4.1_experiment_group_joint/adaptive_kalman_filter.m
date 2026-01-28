function [z_est, P_new] = adaptive_kalman_filter(z_pred, P_prev, z_meas, quality_flag, p)
    % 预测
    P_pred = P_prev + p.kf_Q;
    
    if quality_flag == 1
        z_est = z_pred; P_new = P_pred;
    else
        innovation = abs(z_meas - z_pred);
        
        % === 【修改】使用参数变量代替硬编码 ===
        if innovation > p.kf_thresh_noise 
            % 场景: 巨大跳变 (噪声)
            R_dynamic = p.kf_R * p.kf_scale_noise; 
            
        elseif innovation > p.kf_thresh_step
            % 场景: 台阶突变
            R_dynamic = p.kf_R * p.kf_scale_step;
            
        else
            % 场景: 平稳
            R_dynamic = p.kf_R;
        end
        
        K = P_pred / (P_pred + R_dynamic);
        z_est = z_pred + K * (z_meas - z_pred);
        P_new = (1 - K) * P_pred;
    end
end