% 非线性约束（硬约束）
function [c, ceq] = nonlinearConstraints(trajectory, obstacle_center, obstacle_radius)
    x = trajectory(:, 1);
    y = trajectory(:, 2);
    % 避障硬约束
    distance_to_obstacle = sqrt((x - obstacle_center(1)).^2 + (y - obstacle_center(2)).^2);
    c = obstacle_radius - distance_to_obstacle;
    c = max(c, 0);  % 确保硬约束被严格应用
    ceq = [];
end