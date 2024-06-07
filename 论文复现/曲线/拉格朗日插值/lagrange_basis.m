% 计算拉格朗日基函数
function Lk = lagrange_basis(x, k, x_val)
    n = length(x);
    Lk = ones(size(x_val));
    for i = 1:n
        if i ~= k
            Lk = Lk .* (x_val - x(i)) / (x(k) - x(i));
        end
    end
end