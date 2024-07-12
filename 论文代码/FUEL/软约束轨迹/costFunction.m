% 代价函数
function cost = costFunction(trajectory, obstacle_center, obstacle_radius)
    x = trajectory(:, 1);
    y = trajectory(:, 2);
    % 速度平滑度约束
    velocity_smoothness = sum(diff(x).^2 + diff(y).^2);
    % 避障软约束
    distance_to_obstacle = sqrt((x - obstacle_center(1)).^2 + (y - obstacle_center(2)).^2);
    obstacle_penalty = sum(max(0, obstacle_radius - distance_to_obstacle).^2);
    % 增加平滑性和避障软约束的权重
    cost = 20 * velocity_smoothness + 10 * obstacle_penalty;
end


