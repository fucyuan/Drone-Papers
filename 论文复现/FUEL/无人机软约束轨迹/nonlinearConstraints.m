% 非线性约束（硬约束）
function [c, ceq] = nonlinearConstraints(trajectory, obstacle_center, obstacle_radius)
    x = trajectory(:, 1);
    y = trajectory(:, 2);
    z = trajectory(:, 3);
    % 避障硬约束
    distance_to_obstacle = sqrt((x - obstacle_center(1)).^2 + (y - obstacle_center(2)).^2 + (z - obstacle_center(3)).^2);
    c = obstacle_radius - distance_to_obstacle;  % 确保距离大于障碍物半径
    ceq = [];
end