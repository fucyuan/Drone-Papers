clear;
clc;
x = [1, 2, 3, 4, 5]; % 插值点的x坐标
y = [20, 4, 68, 8, 10]; % 插值点的y坐标
% 插值点之间的细分点
x_val = linspace(min(x), max(x), 100);
% 计算插值多项式在细分点处的值
Pn_val = lagrange_interpolation(x, y, x_val);
% 绘制结果
figure;
plot(x, y, 'ro', 'MarkerSize', 10, 'DisplayName', '插值点');
hold on;
plot(x_val, Pn_val, 'b-', 'LineWidth', 2, 'DisplayName', '拉格朗日插值多项式');
legend;
title('拉格朗日插值法演示');
xlabel('x');
ylabel('y');
grid on;
hold off;
