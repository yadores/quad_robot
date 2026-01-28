function draw_robot_S(t, x, f, contact, p, h_line_h, h_line_v, h_line_f1, h_line_f2, is_record, filename, step_idx, feet_pos_cmd)
    % DRAW_ROBOT_S: 可视化机器人状态、地形及足端力 (V6 布局优化版)
    
    h_fig = ancestor(h_line_h, 'figure');
    set(0, 'CurrentFigure', h_fig); 
    
    % ==========================================
    % 左侧：3D 机器人与地形 (占两行)
    % ==========================================
    subplot(2,2,[1 3]); 
    cla; hold on; axis equal; grid on;
    view(30, 20); 
    
    % 跟随视角
    xlim([x(4)-0.8, x(4)+1.8]); 
    ylim([-0.8, 0.8]); 
    zlim([-0.2, 0.8]);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    
    % --- 地形绘制 (矩阵化优化) ---
    x_terrain = linspace(x(4)-1.5, x(4)+2.5, 30); 
    y_terrain = linspace(-0.8, 0.8, 10);
    [X_grid, Y_grid] = meshgrid(x_terrain, y_terrain);
    Z_grid = get_terrain_z(X_grid, Y_grid, p);
    
    surf(X_grid, Y_grid, Z_grid, 'FaceColor', [0.9 0.9 0.9], ...
         'EdgeColor', 'none', 'FaceLighting', 'none', ...
         'FaceAlpha', 0.6); 
    
    % --- 机器人本体 ---
    com = x(4:6);
    psi = x(3);
    R = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    plot3(com(1), com(2), com(3), 'k.', 'MarkerSize', 20);
    
    for i = 1:4
        p_foot_world = feet_pos_cmd(:, i); 
        color = 'b'; if contact(i), color = 'r'; linewidth = 2; else, linewidth = 1; end 
        
        r_hip = p.hip_offset(:, i);
        p_hip_world = com + R * r_hip;
        
        plot3([p_hip_world(1), p_foot_world(1)], ...
              [p_hip_world(2), p_foot_world(2)], ...
              [p_hip_world(3), p_foot_world(3)], ...
              '-', 'Color', color, 'LineWidth', linewidth);
          
        if contact(i)
            f_i = f((i-1)*3+1:i*3);
            scale = 0.002; 
            quiver3(p_foot_world(1), p_foot_world(2), p_foot_world(3), ...
                    f_i(1)*scale, f_i(2)*scale, f_i(3)*scale, ...
                    'm', 'LineWidth', 1.5, 'MaxHeadSize', 0.5, 'AutoScale', 'off');
        end
    end
    
    title(sprintf('Time: %.2fs | Terrain: %s', t, p.terrain_type));
    
    % ==========================================
    % 右上：高度与速度
    % ==========================================
    subplot(2,2,2); 
    ylim([0, 0.8]); 
    xlim([max(0, t-3), max(1, t)]); % 动态滚屏
    addpoints(h_line_h, t, x(6));
    addpoints(h_line_v, t, x(10));
    % title('Height & Vel'); % 也可以加个标题
    
    % ==========================================
    % 右下：足端力 (对角线分组)
    % ==========================================
    subplot(2,2,4);
    ylim([0, 250]); % 根据你的 f_max=300 调整，250通常够看
    xlim([max(0, t-3), max(1, t)]); 
    
    % 计算对角线力 (只看 Z 轴支撑力)
    % 索引: 1:FL, 2:FR, 3:HL, 4:HR
    % Group 1: FL(1) + HR(4)
    fz_grp1 = f(3) + f(12); 
    
    % Group 2: FR(2) + HL(3)
    fz_grp2 = f(6) + f(9);
    
    addpoints(h_line_f1, t, fz_grp1);
    addpoints(h_line_f2, t, fz_grp2);
    
    drawnow limitrate; 
    
    % ==========================================
    % GIF 录制
    % ==========================================
    if is_record
        frame = getframe(h_fig);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);
        draw_interval = 30; 
        delay_time = p.dt_sim * draw_interval;
        
        if step_idx == 30 || ~isfile(filename)
            imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', delay_time);
        else
            imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', delay_time);
        end
    end
end