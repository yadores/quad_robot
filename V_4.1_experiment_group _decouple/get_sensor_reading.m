function [z_meas, quality_flag] = get_sensor_reading(x, y, p)
    % GET_SENSOR_READING 模拟低精度 2.5D 地形传感器
    % 
    % 模拟特性：
    % 1. 离散化误差 (2.5D Grid): 模拟有限的分辨率
    % 2. 测量噪声 (Sensor Noise): 模拟传感器抖动
    % 3. 数据丢失 (Blind Spots/Dropouts): 模拟空白建模
    
    % 1. 获取真实真值 (Ground Truth) - 仅用于生成传感器数据
    z_true = get_terrain_z(x, y, p);
    
    % 2. 模拟 2.5D 栅格化 (降低分辨率)
    % 假设地图分辨率是 10cm (0.1m)，这意味着在这个格子内读数是一样的
    grid_res = 0.1; 
    x_grid = round(x / grid_res) * grid_res;
    y_grid = round(y / grid_res) * grid_res;
    
    % 重新采样，引入离散误差 (就像麦块Minecraft里的地形)
    z_coarse = get_terrain_z(x_grid, y_grid, p); 
    
    % 3. 模拟随机测量噪声 (高斯白噪声)
    noise = p.sensor_noise * randn(); 
    z_meas = z_coarse + noise;
    
    % 4. 模拟数据丢失 (空白建模)
    % 设定一定概率读不到数据 (例如雷达散射、遮挡)
    if rand() < p.sensor_drop_prob
        quality_flag = 1; % 【信号差/丢失】
        z_meas = 0;       % 读数无效 (通常传感器会给 NaN 或 0)
    else
        quality_flag = 0; % 【信号好】
    end
end