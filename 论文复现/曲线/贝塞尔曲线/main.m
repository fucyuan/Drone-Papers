clc,clear;
% 定义控制点
control_points = [1, 25; 2, 4; 3, 68; 4, 82; 5, 10]; % 每行是一个控制点 (x, y)

% 提取 x 和 y 坐标
x = control_points(:, 1);
y = control_points(:, 2);

% 生成参数 t
t = linspace(0, 1, 1000);

% 计算贝塞尔曲线
B = bezier_curve(t, control_points);

% 绘制结果
figure;
plot(control_points(:, 1), control_points(:, 2), 'ro-', 'MarkerSize', 10, 'DisplayName', '控制点');
hold on;
plot(B(:, 1), B(:, 2), 'b-', 'LineWidth', 2, 'DisplayName', '贝塞尔曲线');
legend;
title('贝塞尔曲线插值');
xlabel('x');
ylabel('y');
grid on;
hold off;

