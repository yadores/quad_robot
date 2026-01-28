function z = get_terrain_z(x, y, p)
    % get_terrain_z: 计算地形高度 (支持波浪阶梯)
    
    z = zeros(size(x)); % 初始化
    
    if strcmp(p.terrain_type, 'wave')
        % 平滑波浪
        mask = x > 0.5;
        z(mask) = p.wave_amp * sin(p.wave_freq * (x(mask) - 0.5));
        
    elseif strcmp(p.terrain_type, 'wave_stairs')
        % 阶梯状波浪 (Zero-Order Hold)
        mask = x > 0.5;
        
        % 1. 计算相对距离
        x_rel = x(mask) - 0.5;
        
        % 2. 离散化 X (核心逻辑)
        % floor(x / width) * width 会把 x 变成台阶状
        x_quantized = floor(x_rel / p.stair_width) * p.stair_width;
        
        % 3. 代入正弦公式
        z(mask) = p.wave_amp * sin(p.wave_freq * x_quantized);
        
    elseif strcmp(p.terrain_type, 'slope')
        % 斜坡
        mask = x > p.slope_start_x;
        z(mask) = (x(mask) - p.slope_start_x) * tan(p.slope_rad);
    end
    % 如果是 flat，z 保持全 0 即可
end