function trajectory = bspline_deboor(p_b, x)
    % B样条插值函数
    n = size(x, 2) - 1;
    internal_knots = linspace(0, 1, n - p_b + 1);
    knots = [zeros(1, p_b), internal_knots, ones(1, p_b)];
    u = linspace(knots(p_b+1), knots(end-p_b), 1000); % 生成参数 u
    trajectory = bspline_curve(x', p_b, knots, u);
    trajectory = trajectory';
end