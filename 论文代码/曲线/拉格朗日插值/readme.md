
## 拉格朗日插值法

拉格朗日插值法是通过给定的插值点来构造一个插值多项式，使得多项式在这些点上精确地通过。其核心思想是利用拉格朗日基函数来构建插值多项式。

#### 拉格朗日基函数

对于给定的插值点 $((x_1, y_1), (x_2, y_2), \ldots, (x_n, y_n))$，第 $(k)$ 个拉格朗日基函数 $(L_k(x))$ 定义如下：

$$
L_k(x) = \prod_{\substack{1 \leq i \leq n \\ i \neq k}} \frac{x - x_i}{x_k - x_i}
$$

该基函数的特点是在 $(x = x_k)$ 时 $(L_k(x_k) = 1)$，在 $(x = x_i)$（$(i \neq k)$）时 $(L_k(x_i) = 0)$。这使得拉格朗日基函数在构造插值多项式时具有选择性，只对一个插值点有贡献。

#### 拉格朗日插值多项式

利用拉格朗日基函数，可以构造拉格朗日插值多项式 $(P_n(x))$：

$$
P_n(x) = \sum_{k=1}^n y_k L_k(x)
$$

其中 $(y_k)$ 是第 $(k)$ 个插值点的值。插值多项式 $(P_n(x))$ 在每个插值点 $(x_k)$ 处都等于 $(y_k)$。

### 代码中的数学公式解释

#### 拉格朗日基函数计算

在代码中，`lagrange_basis` 函数实现了拉格朗日基函数的计算：

```matlab
function Lk = lagrange_basis(x, k, x_val)
    n = length(x);
    Lk = ones(size(x_val));
    for i = 1:n
        if i ~= k
            Lk = Lk .* (x_val - x(i)) / (x(k) - x(i));
        end
    end
end
```

- **输入：** `x` 是插值点的横坐标，`k` 是第 $(k)$ 个基函数的索引，`x_val` 是计算基函数的点。
- **过程：** 循环计算拉格朗日基函数的乘积部分，跳过 $(i = k)$ 的情况。
- **公式：** 计算的公式为 $(L_k(x) = \prod_{\substack{1 \leq i \leq n \\ i \neq k}} \frac{x - x_i}{x_k - x_i})$。

#### 拉格朗日插值多项式计算

在代码中，`lagrange_interpolation` 函数实现了拉格朗日插值多项式的计算：

```matlab
function Pn = lagrange_interpolation(x, y, x_val)
    n = length(x);
    Pn = zeros(size(x_val));
    for k = 1:n
        Lk = lagrange_basis(x, k, x_val);
        Pn = Pn + y(k) * Lk;
    end
end
```

- **输入：** `x` 是插值点的横坐标，`y` 是插值点的纵坐标，`x_val` 是计算插值多项式的点。
- **过程：** 对每个插值点计算对应的拉格朗日基函数，并累加各插值点的贡献。
- **公式：** 计算的公式为 $(P_n(x) = \sum_{k=1}^n y_k L_k(x))$。

#### 示例说明

假设有以下插值点：

```matlab
x = [1, 2, 3, 4, 5]; % 插值点的x坐标
y = [2, 4, 6, 8, 10]; % 插值点的y坐标
```

然后定义用于计算插值多项式的点 `x_val`：

```matlab
x_val = linspace(min(x), max(x), 100); % 生成100个点用于绘制插值曲线
```

计算插值多项式在这些点处的值：

```matlab
Pn_val = lagrange_interpolation(x, y, x_val);
```

绘制结果：

```matlab
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
```