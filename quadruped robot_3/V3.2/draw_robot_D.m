function draw_robot(t, x, f, contact, p, h_line_h, h_line_v, is_record, filename, step_idx)
    % draw_robot: 可视化 + 动态跟随相机
    
    h_fig = ancestor(h_line_h, 'figure');
    set(0, 'CurrentFigure', h_fig); 
    
    %% 1. 绘图逻辑
    subplot(1,2,1); 
    cla; hold on; axis equal; grid on;
    
    com = x(4:6);
    
    % === 关键修改1: 动态相机 (Camera Follow) ===
    % 始终以机器人当前 X 位置为中心，前后各看 0.8 米
    % 这样机器人永远在屏幕正中间，视觉上会被“放大”
    xlim([com(1)-0.8, com(1)+0.8]); 
    
    % Y 轴稍微给宽一点，防止踢歪了跑出屏幕
    ylim([com(2)-0.8, com(2)+0.8]); 
    
    zlim([0, 0.8]); % 高度固定
    xlabel('X'); ylabel('Y'); zlabel('Z');
    
    % 视角固定
    view(30, 20); 
    
    psi = x(3);
    R = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    
    % === 关键修改2: 静态地面跑道 (Static Ground) ===
    % 画一个固定的长跑道 (-1m 到 6m)，而不是跟着机器人动的小方块
    % 这样相机移动时，地面纹理会向后流动，产生速度感
    patch([-1, 6, 6, -1], [-2, -2, 2, 2], [0 0 0 0], ...
          [0.9 0.9 0.9], 'EdgeColor', '#aaaaaa', 'FaceAlpha', 0.2);
    
    % 画机器人身体
    plot3(com(1), com(2), com(3), 'k.', 'MarkerSize', 40); 
    
    for i = 1:4
        r_hip = p.hip_offset(:, i);
        p_foot_world = com + R * r_hip; p_foot_world(3) = 0; 
        
        if contact(i)
             color = 'r'; linewidth = 3;
        else
             color = 'b'; linewidth = 1; % 摆动腿细一点
        end
        
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
    ylim([0, 0.6]); xlim([0, max(1, t)]);
    
    drawnow limitrate; 
    
    %% 2. GIF 录制逻辑 (不变)
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