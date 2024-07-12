clc;
clear;

% 定义控制点
control_points = [1, 25; 2, 4; 3, 68; 4, 82; 5, 10]; % 每行是一个控制点 (x, y)

% 提取 x 和 y 坐标
x = control_points(:, 1);
y = control_points(:, 2);

% 定义不同的参数 t 数量
t_values = {linspace(0, 1, 10), linspace(0, 1, 50), linspace(0, 1, 100), linspace(0, 1, 500), linspace(0, 1, 1000)};
% 生成不同参数 t 对应的贝塞尔曲线并绘制结果
figure;
plot(control_points(:, 1), control_points(:, 2), 'ko-', 'MarkerSize', 10, 'DisplayName', '控制点');
hold on;
colors = ['r', 'g', 'b', 'c', 'm'];
for k = 1:length(t_values)
    t = t_values{k};
    B = bezier_curve(t, control_points);
    plot(B(:, 1), B(:, 2), 'Color', colors(k), 'LineWidth', 2, 'DisplayName', ['t = ', num2str(length(t))]);
end
legend;
title('贝塞尔曲线插值（不同参数 t）');
xlabel('x');
ylabel('y');
grid on;
hold off;
