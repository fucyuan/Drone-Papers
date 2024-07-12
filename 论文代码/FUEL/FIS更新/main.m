
    % 初始化前沿信息结构
    FIS = struct('Clusters', [], 'AveragePositions', [], 'AABB', [], 'Viewpoints', [], 'ConnectionCosts', []);
    
    % 创建一个简单的2D地图 (20x20)
    mapSize = [20, 20];
    map = ones(mapSize); % 1 表示空闲区域
    
    % 随机在地图上放置未知区域 (2 表示未知区域)
    numUnknownCells =100; % 可以根据需要调整未知区域的数量
    unknownCells = randperm(numel(map), numUnknownCells);
    map(unknownCells) = 2;
    
    % 定义已知区域 (例如 [x_min, x_max, y_min, y_max])
    updatedRegion = [5, 15, 5, 15];
    
    % 将更新区域标记为已知区域 (例如，将值设为1)
    map(updatedRegion(1):updatedRegion(2), updatedRegion(3):updatedRegion(4)) = 1; % 1 表示空闲区域
    
    % 定义阈值
    clusterSizeThreshold = 5;  % 示例阈值
    pcaThreshold = 50;         % 示例PCA阈值
    
    % 更新FIS
    FIS = updateMap(FIS, updatedRegion, map, clusterSizeThreshold, pcaThreshold);
    
    % 生成候选视点和更新连接成本
    FIS = generateViewpointsAndUpdateCosts(FIS, map);
    
    % 显示结果
    disp(FIS);
    
    % 可视化地图和前沿簇
    visualizeMapAndFrontiers(map, FIS, updatedRegion);











