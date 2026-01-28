function [total_scrapes, scrape_rate] = evaluate_scrapes(log, p, experiment_name)
    % EVALUATE_SCRAPES 离线刮擦检测分析工具
    % 【修改版】：横坐标改为"空间位置(X)"，而非"时间(t)"，以直观显示地形形状
    
    if ~isfield(log, 'feet')
        error('日志中未找到足端轨迹数据 (log.feet)。请先修改 run_main 进行记录。');
    end

    n_steps = size(log.feet, 3);
    scrape_mask = zeros(4, n_steps);
    scrape_depth = zeros(4, n_steps);
    
    % 容差 (穿模深度超过 2mm 算刮擦)
    tolerance = 0.002; 
    
    %% 1. 逐点检测
    for k = 1:n_steps
        for i = 1:4
            x = log.feet(1, i, k);
            y = log.feet(2, i, k);
            z_foot = log.feet(3, i, k);
            
            % 获取该位置的【真实地形高度】(Ground Truth)
            z_terrain = get_terrain_z(x, y, p);
            
            % 判定刮擦
            if z_foot < z_terrain - tolerance
                scrape_mask(i, k) = 1;
                scrape_depth(i, k) = z_terrain - z_foot;
            end
        end
    end
    
    %% 2. 统计指标
    is_step_scraping = sum(scrape_mask, 1) > 0;
    total_scrapes = sum(is_step_scraping);
    scrape_rate = total_scrapes / n_steps * 100;
    
    fprintf('------------------------------------------------\n');
    fprintf('[%s] 刮擦统计 (地形: %s):\n', experiment_name, p.terrain_type);
    fprintf('  - 总步数: %d\n', n_steps);
    fprintf('  - 刮擦时刻数: %d\n', total_scrapes);
    fprintf('  - 刮擦占比: %.2f%%\n', scrape_rate);
    fprintf('  - 最大穿模深度: %.2f cm\n', max(scrape_depth(:)) * 100);
    fprintf('------------------------------------------------\n');
    
    %% 3. 可视化 (空间域)
    h_fig = figure('Color', 'w', 'Name', ['Scrape Analysis: ' experiment_name]);
    clf(h_fig); 
    
    % --- 子图 1: 地形 vs 足端轨迹 (以右前腿 FR 为例) ---
    subplot(2,1,1); hold on; grid on; box on;
    idx_leg = 2; % 选一条腿 (FR)
    
    % 提取该腿轨迹 (X 和 Z)
    traj_x = squeeze(log.feet(1, idx_leg, :));
    traj_z = squeeze(log.feet(3, idx_leg, :));
    
    % 重新计算沿路径的地形高度用于画图
    terrain_profile = zeros(size(traj_x));
    for k = 1:n_steps
        terrain_profile(k) = get_terrain_z(traj_x(k), log.feet(2, idx_leg, k), p);
    end
    
    % 绘图使用 traj_x 作为横坐标
    % 画地形轮廓 (加粗黑色)
    plot(traj_x, terrain_profile, 'k-', 'LineWidth', 2, 'DisplayName', 'Terrain Profile');
    % 画足端轨迹 (蓝色虚线)
    plot(traj_x, traj_z, 'b--', 'LineWidth', 1, 'DisplayName', 'Foot Trajectory (FR)');
    
    % 标记刮擦点 (红色叉号)
    scrape_idx_leg = find(scrape_mask(idx_leg, :) == 1);
    if ~isempty(scrape_idx_leg)
        % 这里横坐标也必须用 traj_x
        plot(traj_x(scrape_idx_leg), traj_z(scrape_idx_leg), 'rx', ...
             'MarkerSize', 8, 'LineWidth', 1.5, 'DisplayName', 'Collision');
    end
    
    ylabel('Height Z (m)');
    xlabel('Foot Position X (m)'); % 标签改为位置
    title(sprintf('Spatial Trajectory Analysis (Leg %d)', idx_leg));
    legend('Location', 'best');
    
    % --- 子图 2: 全局刮擦事件分布 (随位置分布) ---
    subplot(2,1,2); hold on; grid on; box on;
    
    % 使用机器人质心位置 (COM X) 作为横坐标
    % 这样可以看出刮擦是发生在第几米处，而不是第几秒
    if isfield(log, 'x')
        com_x = log.x(4, :); % 获取质心 X 坐标
        xlabel_text = 'Robot COM Position X (m)';
        x_data = com_x;
    else
        % 兼容性回退：如果没有 log.x，还是用时间
        x_data = log.t;
        xlabel_text = 'Time (s)';
    end

    % 画出刮擦事件
    area(x_data, double(is_step_scraping), 'FaceColor', 'r', 'EdgeColor', 'none', 'FaceAlpha', 0.3);
    
    ylim([0, 1.2]);
    yticks([0 1]); yticklabels({'Safe', 'Collision'});
    xlabel(xlabel_text);
    title(sprintf('Collision Distribution along Path (Total: %d)', total_scrapes));
end