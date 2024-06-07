

clc;
clear;
% 定义控制点
control_points = [1, 25; 2, 4; 3, 68; 4, 82; 5, 10]; % 每行是一个控制点 (x, y)

% 定义不同次数的B样条
degrees = [2, 3, 4]; % 二次、三次和四次B样条

% 生成并绘制不同次数的B样条曲线
figure;
plot(control_points(:, 1), control_points(:, 2), 'ko-', 'MarkerSize', 10, 'DisplayName', '控制点');
hold on;
colors = ['r', 'g', 'b'];
for k = 1:length(degrees)
    degree = degrees(k);
    % 生成节点向量，注意这里的节点向量长度为 控制点数量 + B样条次数 + 1
    % 生成节点向量，节点向量长度为控制点数量 + B样条次数 + 1
    internal_knots = linspace(0, 1, length(control_points) -degree + 1);
    knots = [zeros(1, degree), internal_knots, ones(1, degree)];
    % knots = [zeros(1, degree), linspace(0, 1, length(control_points) - degree + 1), ones(1, degree)];
    u = linspace(knots(degree+1), knots(end-degree), 1000); % 生成参数 u
    curve = bspline_curve(control_points, degree, knots, u);
    plot(curve(:, 1), curve(:, 2), 'Color', colors(k), 'LineWidth', 2, 'DisplayName', ['Degree = ', num2str(degree)]);
end
legend;
title('B样条曲线插值（不同次数）');
xlabel('x');
ylabel('y');
grid on;
hold off;

