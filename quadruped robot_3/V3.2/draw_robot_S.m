function draw_robot_S(t, x, f, contact, p, h_line_h, h_line_v, is_record, filename, step_idx)
    % draw_robot_S: 可视化 + 可选的 GIF 录制
    
    h_fig = ancestor(h_line_h, 'figure');
    set(0, 'CurrentFigure', h_fig); 
    
    %% 1. 绘图逻辑
    subplot(1,2,1); 
    cla; hold on; axis equal; grid on;
    view(30, 20); 
    
    % 调整显示范围，以便看清后面的斜坡
    xlim([-0.3, 4.0]); ylim([-1, 1]); zlim([0, 1.5]);

    xlabel('X'); ylabel('Y'); zlabel('Z');
    
    com = x(4:6);
    psi = x(3);
    R = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    
    % 绘制地形 (平地 + 斜坡) ===
    % 平地部分
    patch([p.slope_start_x, -1, -1, p.slope_start_x], [-2 -2 2 2], [0 0 0 0], ...
          [0.9 0.9 0.9], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
    % 斜坡部分 (绘制长 4m 的斜坡)
    slope_len = 4.0;
    x_end = p.slope_start_x + slope_len;
    z_end = slope_len * tan(p.slope_rad);
    
    X_slope = [p.slope_start_x, x_end, x_end, p.slope_start_x];
    Y_slope = [-2, -2, 2, 2];
    Z_slope = [0, z_end, z_end, 0];
    
    patch(X_slope, Y_slope, Z_slope, ...
          [0.8 0.8 0.8], 'EdgeColor', '#aaaaaa', 'FaceAlpha', 0.5);
    % ===================================
    
    plot3(com(1), com(2), com(3), 'k.', 'MarkerSize', 30);
    
    for i = 1:4
        r_hip = p.hip_offset(:, i);
        p_foot_world = com + R * r_hip; 
        
        %  足端可视化需要贴合地形，而非简单的 z=0
        if p_foot_world(1) < p.slope_start_x
            p_foot_world(3) = 0;
        else
            p_foot_world(3) = (p_foot_world(1) - p.slope_start_x) * tan(p.slope_rad);
        end
        
        color = 'b'; if contact(i), color = 'r'; linewidth = 3; else, linewidth = 1; end
        plot3([com(1), p_foot_world(1)], [com(2), p_foot_world(2)], [com(3), p_foot_world(3)], ...
              '-', 'Color', color, 'LineWidth', linewidth);
          
        if contact(i)
            f_i = f((i-1)*3+1:i*3);
            scale = 0.002; 
            quiver3(p_foot_world(1), p_foot_world(2), p_foot_world(3), ...
                    f_i(1)*scale, f_i(2)*scale, f_i(3)*scale, ...
                    'm', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale', 'off');
        end
    end
    
    title(sprintf('Time: %.4fs | Height: %.3fm | Vx: %.2fm/s', t, x(6), x(10)));
    
    addpoints(h_line_h, t, x(6));
    addpoints(h_line_v, t, x(10));
    
    subplot(1,2,2); 
    ylim([0, 0.6]); % 调整Y轴范围适应爬坡后的高度增加
    xlim([0, max(1, t)]);
    
    drawnow limitrate; 
    
    %% 2. GIF 录制逻辑 (保持不变)
    if is_record
        frame = getframe(h_fig);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);
        
        draw_interval_steps = 25; 
        delay_time = draw_interval_steps * p.dt_sim;
        
        if step_idx == draw_interval_steps 
            imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', delay_time);
        else
            imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', delay_time);
        end
    end
end