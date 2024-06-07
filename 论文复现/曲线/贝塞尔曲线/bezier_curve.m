
% 贝塞尔曲线函数
function B = bezier_curve(t, control_points)
    n = size(control_points, 1) - 1;
    B = zeros(length(t), 2);
    for i = 0:n
        % 计算每个控制点对曲线的贡献
        B = B + nchoosek(n, i) * (1-t).^(n-i)' .* t.^i' * control_points(i+1, :);
    end
end