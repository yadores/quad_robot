function visualize_2d(t, q, params)
    % visualize_2d_side: 2D侧视图动画，清晰展示步态相位和高度变化
    % 视觉风格：
    %   - 左侧腿 (LF, LH): 实线 (粗) -> 代表近端
    %   - 右侧腿 (RF, RH): 虚线 (细) -> 代表远端
    %   - 机身: 黑色粗线
    
    if isempty(t) || isempty(q)
        warning('无数据，无法绘图');
        return;
    end

    fprintf('正在生成 2D 侧视动画...\n');
    
    % --- 1. 场景设置 ---
    h_fig = figure('Name', '2D Side View Analysis', 'Color', 'w', 'Position', [100, 100, 1000, 500]);
    axes_handle = axes('Parent', h_fig);
    axis equal; grid on; hold on;
    xlabel('X Position (m)'); ylabel('Z Height (m)');
    
    % 定义机身长度 (随便定义)
    body_length = 0.4; 
    half_len = body_length / 2;
    
    % --- 2. 初始化绘图句柄 ---
    % 地面
    plot([-10, 50], [0, 0], 'k-', 'LineWidth', 2); 
    
    % 机身 (一条直线代表躯干)
    h_body = plot([0, 0], [0, 0], 'k-', 'LineWidth', 4);
    
    % 腿部线条: [Hip_X, Knee_X, Foot_X], [Hip_Z, Knee_Z, Foot_Z]
    % 样式约定: 
    % LF (左前): 红色实线
    % RF (右前): 红色虚线
    % LH (左后): 蓝色实线
    % RH (右后): 蓝色虚线
    
    h_LF = plot([0,0,0], [0,0,0], 'r-', 'LineWidth', 2.5, 'DisplayName', 'Left Front');
    h_RF = plot([0,0,0], [0,0,0], 'r--', 'LineWidth', 1.5, 'DisplayName', 'Right Front');
    h_LH = plot([0,0,0], [0,0,0], 'b-', 'LineWidth', 2.5, 'DisplayName', 'Left Hind');
    h_RH = plot([0,0,0], [0,0,0], 'b--', 'LineWidth', 1.5, 'DisplayName', 'Right Hind');
    
    legend([h_LF, h_RF, h_LH, h_RH, h_body], {'LF (左前)', 'RF (右前)', 'LH (左后)', 'RH (右后)', 'Body'}, 'Location', 'northeast');
    
    title_handle = title('Time: 0.00 s');
    
    % --- 【新增 1】定义 GIF 文件名，如果已存在则删除（防止追加到旧文件） ---
    %filename = 'Quadruped_Trot_2D.gif';
    %if exist(filename, 'file')
    %    delete(filename);
    %end

    % --- 3. 动画循环 ---
    play_step = 10; % 跳帧加速
    
    for k = 1:play_step:length(t)
        if ~isvalid(h_fig), break; end
        
        % 获取当前状态
        X_com = q(k, 1);
        Z_com = q(k, 2);
        
        % 计算前后髋关节位置 (世界坐标)
        % 假设机身保持水平 (Pitch=0)
        hip_front_x = X_com + half_len;
        hip_rear_x  = X_com - half_len;
        hip_z       = Z_com;
        
        % 更新机身绘图
        set(h_body, 'XData', [hip_rear_x, hip_front_x], 'YData', [hip_z, hip_z]);
        
        % --- 计算各腿运动学 (Kinematics) ---
        % 提取角度 (注意 q 的索引顺序: 5,6=LF | 7,8=RF | 9,10=LH | 11,12=RH)
        % 角度单位转弧度
        q_rad = deg2rad(q(k, 5:12)); 
        
        % 定义一个匿名函数计算单腿坐标
        % 输入: 髋坐标(x,z), 髋角h, 膝角k
        % 输出: [x_knee, x_foot], [z_knee, z_foot]
        calc_leg = @(hx, hz, th_h, th_k) deal( ...
            [hx + params.L1*sin(th_h), hx + params.L1*sin(th_h) + params.L2*sin(th_h+th_k)], ...
            [hz - params.L1*cos(th_h), hz - params.L1*cos(th_h) - params.L2*cos(th_h+th_k)] ...
        );
        
        % 1. 左前腿 (LF) - Front Hip
        [lf_x, lf_z] = calc_leg(hip_front_x, hip_z, q_rad(1), q_rad(2));
        set(h_LF, 'XData', [hip_front_x, lf_x], 'YData', [hip_z, lf_z]);
        
        % 2. 右前腿 (RF) - Front Hip
        [rf_x, rf_z] = calc_leg(hip_front_x, hip_z, q_rad(3), q_rad(4));
        set(h_RF, 'XData', [hip_front_x, rf_x], 'YData', [hip_z, rf_z]);
        
        % 3. 左后腿 (LH) - Rear Hip
        [lh_x, lh_z] = calc_leg(hip_rear_x, hip_z, q_rad(5), q_rad(6));
        set(h_LH, 'XData', [hip_rear_x, lh_x], 'YData', [hip_z, lh_z]);
        
        % 4. 右后腿 (RH) - Rear Hip
        [rh_x, rh_z] = calc_leg(hip_rear_x, hip_z, q_rad(7), q_rad(8));
        set(h_RH, 'XData', [hip_rear_x, rh_x], 'YData', [hip_z, rh_z]);
        
        % --- 相机跟随 ---
        % 让相机中心始终对准机身
        xlim([X_com - 0.5, X_com + 0.5]);
        % 【修改】极致压缩 Y 轴，并精确刻度
        % 范围 [-0.01, 0.16]: 刚好包住地面(0)和机身震荡区(0.13±0.01)
        current_ylim = [-0.01, 0.16]; 
        ylim(current_ylim); 
        
        % 【新增】强制设定 Y 轴刻度间隔为 0.01m
        % 这样你能清晰读出 0.13, 0.14, 0.12 等数值
        yticks([0, 0.12:0.01:0.15]);
        
        % 开启从属网格 (Minor Grid) 进一步辅助观察 (可选)
        grid on; grid minor; 
        
        % 更新标题信息
        set(title_handle, 'String', sprintf('Time: %.2f s | V_x: %.2f m/s | Height: %.4f m', ...
            t(k), q(k,3), q(k,2)));
        
        drawnow;
        % --- 【新增 2】将当前这一帧写入 GIF 文件 ---
        % Append=true 表示把新的一帧加到后面，形成动画
        %try
        %    exportgraphics(h_fig, filename, 'Append', true); 
        %catch
            % 防止绘图过快或窗口关闭报错
        %end
    end
    %fprintf('GIF 导出完成！文件名为: %s\n', filename);
end