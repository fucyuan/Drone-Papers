function visualizeMapAndFrontiers(map, FIS, updatedRegion)
    figure;
    hold on;
    
    % 可视化地图中的已知区域
    [x, y] = find(map == 1);
    scatter(y, x, 'filled', 'w'); % 空闲区域
    [x, y] = find(map == 2);
    scatter(y, x, 'filled', 'k'); % 未知区域
    
    % 可视化前沿簇
    colors = lines(length(FIS.Clusters));
    for i = 1:length(FIS.Clusters)
        cluster = FIS.Clusters{i};
        cluster = cell2mat(cluster');
        scatter(cluster(:,2), cluster(:,1), 50, 'filled', 'MarkerFaceColor', colors(i,:)); % 前沿簇
    end
    
    % 添加B_m标记
    B_m = updatedRegion;
    rectangle('Position', [B_m(3)-1, B_m(1)-1, B_m(4)-B_m(3), B_m(2)-B_m(1)], 'EdgeColor', 'r', 'LineStyle', '--', 'LineWidth', 2);
    
    % 添加B_i标记（假设已知B_i的位置）
    % 这里我们假设B_i为FIS.Clusters中的前两个簇的边界框
    for i = 1:min(2, length(FIS.Clusters))
        cluster = cell2mat(FIS.Clusters{i}');
        x_min = min(cluster(:,1));
        x_max = max(cluster(:,1));
        y_min = min(cluster(:,2));
        y_max = max(cluster(:,2));
        rectangle('Position', [y_min-1, x_min-1, y_max-y_min, x_max-x_min], 'EdgeColor', 'g', 'LineStyle', '--', 'LineWidth', 2);
    end
    
    
    % 设置图形属性
    axis equal;
    axis([0 size(map, 2) 0 size(map, 1)]);
    set(gca, 'YDir', 'reverse');
    grid on;
    hold off;
end
