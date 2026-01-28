function draw_robot(t, x, f, contact, p, h_line_h, h_line_v, is_record, filename, step_idx)
    % draw_robot: 可视化 + 可选的 GIF 录制
    % 新增输入:
    %   is_record: bool, 是否录制 GIF
    %   filename:  char, GIF 文件名
    %   step_idx:  int, 当前仿真步数 k (用于判断是否是第一帧)

    % 获取当前 Figure 句柄
    h_fig = ancestor(h_line_h, 'figure');
    set(0, 'CurrentFigure', h_fig); 
    
    %% 1. 绘图逻辑 (与之前完全相同)
    subplot(1,2,1); 
    cla; hold on; axis equal; grid on;
    view(30, 20); 
    
    xlim([-0.3, 3.0]); ylim([-1, 1]); zlim([0, 0.8]);

    xlabel('X'); ylabel('Y'); zlabel('Z');
    
    com = x(4:6);
    psi = x(3);
    R = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    
    plot3(com(1), com(2), com(3), 'k.', 'MarkerSize', 30);
    
    for i = 1:4
        r_hip = p.hip_offset(:, i);
        p_foot_world = com + R * r_hip; p_foot_world(3) = 0; 
        
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
    
    patch([com(1)-2, com(1)+2, com(1)+2, com(1)-2], [-2 -2 2 2], [0 0 0 0], ...
          [0.9 0.9 0.9], 'EdgeColor', '#aaaaaa', 'FaceAlpha', 0.2);
          
    title(sprintf('Time: %.4fs | Height: %.3fm | Vx: %.2fm/s', t, x(6), x(10)));
    
    addpoints(h_line_h, t, x(6));
    addpoints(h_line_v, t, x(10));
    
    subplot(1,2,2); 
    ylim([0, 0.6]); xlim([0, max(1, t)]);
    
    drawnow limitrate; 
    
    %% 2. GIF 录制逻辑 (新增封装)
    if is_record
        % 捕获图像
        frame = getframe(h_fig);
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256);
        
        % 自动计算帧延迟 (根据 draw_robot 的调用频率)
        % 假设每 25 个物理步长调用一次本函数
        draw_interval_steps = 25; 
        delay_time = draw_interval_steps * p.dt_sim;
        
        % 写入文件
        if step_idx == draw_interval_steps 
            % 如果是第一次调用(k=25)，创建新文件
            imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', delay_time);
        else
            % 后续追加
            imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', delay_time);
        end
    end
end