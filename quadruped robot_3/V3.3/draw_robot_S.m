function draw_robot_S(t, x, f, contact, p, h_line_h, h_line_v, is_record, filename, step_idx, feet_pos_cmd)
    % draw_robot_S: 可视化
    % 新增输入: feet_pos_cmd [3x4] -> 外部计算好的足端位置

    h_fig = ancestor(h_line_h, 'figure');
    set(0, 'CurrentFigure', h_fig); 
    
    subplot(1,2,1); 
    cla; hold on; axis equal; grid on;
    view(30, 20); 
    
    % 调整视角范围
    xlim([-0.3, 3.0]); ylim([-1, 1]); zlim([0, 0.8]);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    
    % 绘制地形 (如果有斜坡参数)
    if isfield(p, 'slope_degree') && p.slope_degree > 0
         patch([p.slope_start_x, 5, 5, p.slope_start_x], [-2 -2 2 2], ...
               [0, (5-p.slope_start_x)*tan(p.slope_rad), (5-p.slope_start_x)*tan(p.slope_rad), 0], ...
               [0.8 0.8 0.8], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    end
    
    com = x(4:6);
    psi = x(3);
    R = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    
    % 画质心
    plot3(com(1), com(2), com(3), 'k.', 'MarkerSize', 30);
    
    for i = 1:4
        % =================================================
        % 【关键修改】 直接使用传入的轨迹位置，不再自己计算
        % =================================================
        p_foot_world = feet_pos_cmd(:, i); 
        
        color = 'b'; if contact(i), color = 'r'; linewidth = 3; else, linewidth = 1; end
        
        % 为了画腿，我们需要髋关节位置
        r_hip = p.hip_offset(:, i);
        p_hip_world = com + R * r_hip;
        
        % 画腿连杆 (Hip -> Foot)
        plot3([p_hip_world(1), p_foot_world(1)], ...
              [p_hip_world(2), p_foot_world(2)], ...
              [p_hip_world(3), p_foot_world(3)], ...
              '-', 'Color', color, 'LineWidth', linewidth);
          
        % 画力矢量
        if contact(i)
            f_i = f((i-1)*3+1:i*3);
            scale = 0.002; 
            quiver3(p_foot_world(1), p_foot_world(2), p_foot_world(3), ...
                    f_i(1)*scale, f_i(2)*scale, f_i(3)*scale, ...
                    'm', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale', 'off');
        end
    end
    
    % 画个淡淡的地面阴影
    patch([com(1)-2, com(1)+2, com(1)+2, com(1)-2], [-2 -2 2 2], [0 0 0 0], ...
          [0.9 0.9 0.9], 'EdgeColor', '#aaaaaa', 'FaceAlpha', 0.1);
          
    title(sprintf('Time: %.4fs | Height: %.3fm | Vx: %.2fm/s', t, x(6), x(10)));
    
    addpoints(h_line_h, t, x(6));
    addpoints(h_line_v, t, x(10));
    
    subplot(1,2,2); 
    ylim([0, 0.6]); xlim([0, max(1, t)]);
    drawnow limitrate; 
    
    % GIF 录制部分保持不变...
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