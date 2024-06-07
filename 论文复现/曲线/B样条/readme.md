# B样条曲线插值（不同次数）

本文档解释了如何使用MATLAB生成和绘制不同次数的B样条曲线。代码实现了B样条基函数的递推公式，并生成了相应的B样条曲线。

## 代码结构

### 1. 清理工作空间

首先，清理工作空间并定义控制点：

```matlab
clc;
clear;

% 定义控制点
control_points = [1, 25; 2, 4; 3, 68; 4, 82; 5, 10]; % 每行是一个控制点 (x, y)
```

### 2. B样条基函数递推公式

实现B样条基函数的递推公式。根据递推公式，基函数 \( N_{i,k}(u) \) 定义如下：

- 基本情况（k=0）：
  $$ N_{i,0}(u) = 
  \begin{cases} 
  1, & u_i \le u < u_{i+1} \\
  0, & \text{其他}
  \end{cases}
  $$
  
- 递归情况（k > 0）：
  $$ N_{i,k}(u) = \frac{u - u_i}{u_{i+k} - u_i} N_{i,k-1}(u) + \frac{u_{i+k+1} - u}{u_{i+k+1} - u_{i+1}} N_{i+1,k-1}(u) $$

在代码中实现如下：

```matlab
function B = basis_function(i, k, u, knots)
    if k == 0
        B = double(u >= knots(i) & u < knots(i+1));
        if knots(i+1) == max(knots) % 特殊处理最后一个节点
            B(u == knots(i+1)) = 1;
        end
    else
        if knots(i+k) == knots(i)
            c1 = zeros(size(u));
        else
            c1 = (u - knots(i)) ./ (knots(i+k) - knots(i)) .* basis_function(i, k-1, u, knots);
        end
        if knots(i+k+1) == knots(i+1)
            c2 = zeros(size(u));
        else
            c2 = (knots(i+k+1) - u) ./ (knots(i+k+1) - knots(i+1)) .* basis_function(i+1, k-1, u, knots);
        end
        B = c1 + c2;
    end
end
```

### 3. B样条曲线计算函数

使用B样条基函数和控制点来计算B样条曲线：

```matlab
function curve = bspline_curve(control_points, degree, knots, u)
    n = size(control_points, 1) - 1;
    B = zeros(length(u), n+1);
    
    for i = 1:n+1
        B(:, i) = basis_function(i, degree, u, knots);
    end
    
    curve = B * control_points;
end
```

### 4. 定义不同次数的B样条

定义要比较的不同次数的B样条，并生成和绘制相应的曲线：

```matlab
% 定义不同次数的B样条
degrees = [2, 3, 4]; % 二次、三次和四次B样条

% 生成并绘制不同次数的B样条曲线
figure;
plot(control_points(:, 1), control_points(:, 2), 'ko-', 'MarkerSize', 10, 'DisplayName', '控制点');
hold on;
colors = ['r', 'g', 'b'];
for k = 1:length(degrees)
    degree = degrees(k);
    % 生成节点向量，节点向量长度为控制点数量 + B样条次数 + 1
    internal_knots = linspace(0, 1, length(control_points) - degree + 1);
    knots = [zeros(1, degree), internal_knots, ones(1, degree)];
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
```

## 详细解释

### 节点向量生成

节点向量用于定义参数 \( u \) 的分布，使得B样条基函数在这些节点上有特定的属性。节点向量的生成方法如下：

```matlab
internal_knots = linspace(0, 1, length(control_points) - degree + 1);
knots = [zeros(1, degree), internal_knots, ones(1, degree)];
```

- **前面的 `zeros(1, degree)`**：在节点向量的开头插入 `degree` 个零，以确保在曲线的起点处具有特定的属性。
- **中间的 `linspace(0, 1, length(control_points) - degree + 1)`**：生成从 0 到 1 的均匀分布的节点，使得控制点在这些节点上均匀分布。
- **最后的 `ones(1, degree)`**：在节点向量的末尾插入 `degree` 个一，以确保在曲线的终点处具有特定的属性。

### 参数 \( u \) 的生成

参数 \( u \) 的生成方法确保其在有效的节点区间内变化：

```matlab
u = linspace(knots(degree+1), knots(end-degree), 1000); % 生成参数 u
```

- **`knots(degree+1)`**：有效区间的起点，对应于第 `degree+1` 个节点。
- **`knots(end-degree)`**：有效区间的终点，对应于倒数第 `degree` 个节点。
- **`linspace(knots(degree+1), knots(end-degree), 1000)`**：在起点和终点之间生成1000个等间距的点，确保参数 \( u \) 在有效区间内均匀分布。

### 数学解释

为了确保参数 \( u \) 在有效区间内，我们选择 \( u \) 的范围为 `knots(degree+1)` 到 `knots(end-degree)`。这些点表示了B样条曲线的实际参数范围，使得曲线在这一区间内平滑过渡。

## 结果展示

运行上述代码，MATLAB 会生成一个图形，显示不同次数下的B样条曲线，并比较它们在同一图中的差异。通过这些比较，可以直观地看到在不同次数下B样条曲线的平滑度和形状。


