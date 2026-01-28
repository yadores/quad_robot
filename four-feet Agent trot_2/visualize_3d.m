%% --- 3D 动画播放函数 ---
function visualize_3d(t, q, params)
    fprintf('正在生成 3D 动画...\n');
    
    % 创建窗口
    figure('Name', '3D Trot Animation', 'Color', 'w', 'Position', [100, 100, 800, 600]);
    axis equal; grid on; hold on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    view(30, 30); % 设置 3D 视角
    
    % 机器人几何参数
    body_L = 0.4;  % 机身长
    body_W = 0.2;  % 机身宽
    body_H = 0.1;  % 机身高
    
    % 绘图句柄初始化
    h_body = patch('Vertices', [], 'Faces', [], 'FaceColor', 'c', 'FaceAlpha', 0.5);
    h_legs = plot3(0,0,0, 'k-', 'LineWidth', 3);
    h_feet = plot3(0,0,0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    h_ground = patch([ -1 2 2 -1], [-1 -1 1 1], [0 0 0 0], [0.9 0.9 0.9], 'EdgeColor', 'none');
    
    % 动画循环 (每隔 5 帧画一次，加速播放)
    for i = 1:5:length(t)
        % 1. 获取当前状态
        X_com = q(i, 1);
        Z_com = q(i, 2);
        % 你的模型是 2D 的，我们假设 Y = 0，Pitch = 0
        % 如果未来你加了 Pitch，可以在这里引入
        
        % 2. 绘制机身 (长方体)
        % 定义机身 8 个顶点 (相对于 CoM)
        dx = body_L/2; dy = body_W/2; dz = body_H/2;
        V_local = [ -dx -dy -dz;  dx -dy -dz;  dx  dy -dz; -dx  dy -dz; ...
                    -dx -dy  dz;  dx -dy  dz;  dx  dy  dz; -dx  dy  dz ];
        % 平移到当前位置
        V_world = V_local + [X_com, 0, Z_com];
        
        % 定义面 (Simulink Brick Solid 风格)
        Faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
        set(h_body, 'Vertices', V_world, 'Faces', Faces);
        
        % 3. 计算四条腿的关节位置 (FK)
        % 腿的安装位置 (相对于 CoM)
        hip_offsets = [ dx,  dy;   % LF
                        dx, -dy;   % RF
                       -dx,  dy;   % LH
                       -dx, -dy];  % RH
                       
        % 提取关节角度
        joint_idxs = [5 6; 7 8; 9 10; 11 12]; % 对应 q 中的列
        
        leg_X = []; leg_Y = []; leg_Z = [];
        foot_X = []; foot_Y = []; foot_Z = [];
        
        for leg = 1:4
            % 髋关节世界坐标
            hip_pos = [X_com, 0, Z_com] + [hip_offsets(leg,1), hip_offsets(leg,2), 0];
            
            % 读取角度
            q_hip = q(i, joint_idxs(leg,1));
            q_knee = q(i, joint_idxs(leg,2));
            
            % 正运动学 (2D 平面)
            % 膝关节位置
            knee_pos = hip_pos + [0, 0, -params.L1]; % 简化：假设髋关节垂直向下
            % 这里我们需要用三角函数算得更准一点：
            % 注意：你的动力学方程里 theta 是相对于竖直方向的
            th1 = deg2rad(q_hip);
            th2 = deg2rad(q_knee);
            
            % 膝关节相对于髋关节
            p_knee_rel = [params.L1 * sin(th1), 0, -params.L1 * cos(th1)];
            p_knee = hip_pos + p_knee_rel;
            
            % 脚尖相对于膝关节
            p_foot_rel = [params.L2 * sin(th1 + th2), 0, -params.L2 * cos(th1 + th2)];
            p_foot = p_knee + p_foot_rel;
            
            % 收集绘图数据 (画线：髋->膝->脚)
            leg_X = [leg_X, NaN, hip_pos(1), p_knee(1), p_foot(1)];
            leg_Y = [leg_Y, NaN, hip_pos(2), p_knee(2), p_foot(2)];
            leg_Z = [leg_Z, NaN, hip_pos(3), p_knee(3), p_foot(3)];
            
            foot_X(end+1) = p_foot(1);
            foot_Y(end+1) = p_foot(2);
            foot_Z(end+1) = p_foot(3);
        end
        
        % 更新腿和脚
        set(h_legs, 'XData', leg_X, 'YData', leg_Y, 'ZData', leg_Z);
        set(h_feet, 'XData', foot_X, 'YData', foot_Y, 'ZData', foot_Z);
        
        % 4. 跟随相机
        xlim([X_com-0.5, X_com+0.5]);
        ylim([-0.5, 0.5]);
        zlim([0, 0.5]);
        title(sprintf('Time: %.2f s', t(i)));
        
        drawnow;
    end
end