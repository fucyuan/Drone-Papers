% 清除工作区和命令窗口
clear;
clc;

% 定义起点和终点
start_point = [0, 0];
end_point = [10, 10];

% 定义障碍物位置和半径
obstacle_center = [5, 5];
obstacle_radius =0.5;

% 定义路径点的数量
num_points = 200;  % 增加路径点数量

% 生成初始轨迹（直线）
x = linspace(start_point(1), end_point(1), num_points);
y = linspace(start_point(2), end_point(2), num_points);

% 优化轨迹
options = optimset('Display', 'iter', 'Algorithm', 'sqp');
initial_guess = [x', y'];
optimized_trajectory = fmincon(@(trajectory)costFunction(trajectory, obstacle_center, obstacle_radius), ...
                               initial_guess, [], [], [], [], [], [], ...
                               @(trajectory)nonlinearConstraints(trajectory, obstacle_center, obstacle_radius), options);

% 绘制结果
figure;
hold on;
plot(x, y, 'r--', 'DisplayName', 'Initial Trajectory');
plot(optimized_trajectory(:, 1), optimized_trajectory(:, 2), 'b-', 'LineWidth', 2, 'DisplayName', 'Optimized Trajectory');
viscircles(obstacle_center, obstacle_radius, 'EdgeColor', 'k');
plot(start_point(1), start_point(2), 'go', 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
plot(end_point(1), end_point(2), 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'End');
legend show;
axis equal;
grid on;
xlabel('X');
ylabel('Y');
title('Trajectory Planning with Soft Constraints');


