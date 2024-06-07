clc;
clear;

% 定义参数
N_b = 10; % B样条控制点数
p_b = 3; % B样条阶数
Delta_t_b = 0.1; % 节点间隔初始值
w_t = 1; % 总轨迹时间权重
lambda_c = 0; % 安全性权重
lambda_d = 1; % 动态可行性权重
lambda_bs = 0.5; % 边界状态权重

x_c = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10; % X 坐标
       0, 2, 1, 3, 5, 4, 6, 8, 7, 9, 11; % Y 坐标
       1, 0, 3, 2, 4, 6, 5, 7, 9, 8, 10]; % Z 坐标

% 原始轨迹，只连接控制点
original_trajectory = x_c;

% 优化函数
objectiveFunction = @(x) trajectoryCost(x, N_b, p_b, Delta_t_b, w_t, lambda_c, lambda_d, lambda_bs);

% 优化选项
options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp', 'MaxIterations', 1000, 'MaxFunctionEvaluations', 10000);

% 执行优化
x_opt = fmincon(objectiveFunction, x_c, [], [], [], [], [], [], [], options);

% 计算优化后的轨迹
[~, T_opt, optimized_trajectory] = trajectoryCost(x_opt, N_b, p_b, Delta_t_b, w_t, lambda_c, lambda_d, lambda_bs);

% 绘制轨迹
figure;
plot3(original_trajectory(1,:), original_trajectory(2,:), original_trajectory(3,:), '-o', 'DisplayName', '原始轨迹');
hold on;
plot3(optimized_trajectory(1,:), optimized_trajectory(2,:), optimized_trajectory(3,:), '-x', 'DisplayName', '优化后的轨迹');
xlabel('x'); ylabel('y'); zlabel('z');
title('原始轨迹与优化后轨迹比较');
legend;
grid on;










