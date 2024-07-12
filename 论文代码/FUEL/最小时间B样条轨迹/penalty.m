function p = penalty(value, threshold)
    if value <= threshold
        p = (value - threshold).^2;
    else
        p = 0;
    end
end