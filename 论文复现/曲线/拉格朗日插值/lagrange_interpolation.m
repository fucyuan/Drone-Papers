% 计算拉格朗日插值多项式
function Pn = lagrange_interpolation(x, y, x_val)
    n = length(x);
    Pn = zeros(size(x_val));
    for k = 1:n
        Lk = lagrange_basis(x, k, x_val);
        Pn = Pn + y(k) * Lk;
    end
end