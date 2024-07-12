
% 代价函数
function cost = costFunction(trajectory, obstacle_center, obstacle_radius)
    x = trajectory(:, 1);
    y = trajectory(:, 2);
    z = trajectory(:, 3);
    
    % 计算路径长度
    path_length = sum(sqrt(diff(x).^2 + diff(y).^2 + diff(z).^2));
    
    % 计算加速度平方和近似能耗
    velocity_x = diff(x);
    velocity_y = diff(y);
    velocity_z = diff(z);
    acceleration_x = diff(velocity_x);
    acceleration_y = diff(velocity_y);
    acceleration_z = diff(velocity_z);
    energy = sum(acceleration_x.^2 + acceleration_y.^2 + acceleration_z.^2);
    
    % 计算避障软约束
    distance_to_obstacle = sqrt((x - obstacle_center(1)).^2 + (y - obstacle_center(2)).^2 + (z - obstacle_center(3)).^2);
    obstacle_penalty = sum(max(0, obstacle_radius - distance_to_obstacle).^2);
    
    % 代价函数（综合目标函数）
    alpha1 = 1;  % 路径长度权重
    alpha2 = 0.1;  % 能耗权重
    beta = 1000;  % 避障软约束权重
    cost = alpha1 * path_length + alpha2 * energy + beta * obstacle_penalty;
end