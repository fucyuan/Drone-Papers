function [cost, T, trajectory] = trajectoryCost(x, N_b, p_b, Delta_t_b, w_t, lambda_c, lambda_d, lambda_bs)
    % 平滑成本 f_s
    f_s = 0;
    for i = 1:N_b-2
        s_i = x(:,i+2) - 2*x(:,i+1) + x(:,i);
        f_s = f_s + s_i' * s_i;
    end
    
    % 总轨迹时间 T
    T = (N_b + 1 - p_b) * Delta_t_b;
    
    % 安全性惩罚 f_c
    f_c = 0;
    d_min = 0.5;
    for i = 1:N_b
        d = norm(x(:,i)); % 这里假设距离为原点的距离，可以替换为实际障碍物距离
        f_c = f_c + penalty(d, d_min);
    end
    
    % 动态可行性惩罚 f_v 和 f_a
    f_v = 0;
    f_a = 0;
    v_max = 1; % 最大速度
    a_max = 1; % 最大加速度
    for i = 1:N_b-1
        v = (x(:,i+1) - x(:,i)) / Delta_t_b;
        f_v = f_v + sum(penalty(v, v_max));
    end
    for i = 1:N_b-2
        a = (x(:,i+2) - 2*x(:,i+1) + x(:,i)) / (Delta_t_b^2);
        f_a = f_a + sum(penalty(a, a_max));
    end
    % 边界状态约束成本 f_bs
    x0 = x(:,1); % 初始状态为控制点的第一个点
    if N_b+1 > 4 % 确保有足够的控制点
        x_next = x(:,2); % 下一个目标点为第四个控制点
    else
        x_next = x(:,end); % 如果控制点不够多，取最后一个点
    end
    f_bs = norm((x(:,1) + 4*x(:,2) + x(:,3))/6 - x0)^2 + ...
            norm((x(:,end-2) + 4*x(:,end-1) + x(:,end))/6 - x_next)^2;
    
    % 总成本
    cost = f_s + w_t * T + lambda_c * f_c + lambda_d * (f_v + f_a) + lambda_bs * f_bs;
    
    % 计算轨迹
    trajectory = bspline_deboor(p_b, x);
end