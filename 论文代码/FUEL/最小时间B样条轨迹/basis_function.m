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
        if i+k+1 > length(knots) || knots(i+k+1) == knots(i+1)
            c2 = zeros(size(u));
        else
            c2 = (knots(i+k+1) - u) ./ (knots(i+k+1) - knots(i+1)) .* basis_function(i+1, k-1, u, knots);
        end
        B = c1 + c2;
    end
end