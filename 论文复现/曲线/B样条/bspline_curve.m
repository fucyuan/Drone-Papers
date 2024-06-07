% B样条曲线计算函数
function curve = bspline_curve(control_points, degree, knots, u)
    n = size(control_points, 1) - 1;
    B = zeros(length(u), n+1);
    
    for i = 1:n+1
        B(:, i) = basis_function(i, degree, u, knots);
    end
    
    curve = B * control_points;
end