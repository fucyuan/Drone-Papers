clear;
clc;

% 定义起点和终点
start_point = [0, 0, 0];
end_point = [20, 20, 20];

% 定义障碍物位置和半径
obstacle_center = [10, 10, 10];
obstacle_radius = 3;

% 定义路径点的数量
num_points = 200;  % 增加路径点数量

% 生成初始轨迹（稍微偏移直线）
theta = linspace(0, pi/3, num_points);
x = linspace(start_point(1), end_point(1), num_points);
y = 10* sin(theta) + linspace(start_point(2), end_point(2), num_points);
z = 10* cos(theta) + linspace(start_point(3), end_point(3), num_points);

% 初始猜测轨迹
initial_guess = [x', y', z'];

% 优化轨迹
options = optimset('Display', 'iter', 'Algorithm', 'sqp', 'MaxIter', 500, 'TolFun', 1e-6);
optimized_trajectory = fmincon(@(trajectory)costFunction(trajectory, obstacle_center, obstacle_radius), ...
                               initial_guess, [], [], [], [], [], [], ...
                               @(trajectory)nonlinearConstraints(trajectory, obstacle_center, obstacle_radius), options);

% 绘制结果
figure;
hold on;
plot3(x, y, z, 'r--', 'DisplayName', 'Initial Trajectory');
plot3(optimized_trajectory(:, 1), optimized_trajectory(:, 2), optimized_trajectory(:, 3), 'b-', 'LineWidth', 2, 'DisplayName', 'Optimized Trajectory');
% 绘制障碍物作为一个球体
[xs, ys, zs] = sphere(20);
surf(xs * obstacle_radius + obstacle_center(1), ys * obstacle_radius + obstacle_center(2), zs * obstacle_radius + obstacle_center(3), 'FaceAlpha', 0.3, 'EdgeColor', 'none', 'DisplayName', 'Obstacle');
plot3(start_point(1), start_point(2), start_point(3), 'go', 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
plot3(end_point(1), end_point(2), end_point(3), 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'End');
legend show;
axis equal;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Trajectory Planning with Soft Constraints');
view(3); % 设置视角为3D视图







