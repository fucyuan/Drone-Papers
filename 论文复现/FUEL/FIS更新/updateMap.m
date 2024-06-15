% function FIS = updateMap(FIS, updatedRegion, map, clusterSizeThreshold, pcaThreshold)
%     % 检测受影响的前沿簇
%     affectedClusters = detectAffectedClusters(FIS, updatedRegion);
    
%     % 移除过时的前沿簇
%     for i = length(affectedClusters):-1:1
%         if ~isFrontier(FIS.Clusters{affectedClusters(i)}, map)
%             FIS = removeCluster(FIS, affectedClusters(i));
%         end
%     end
    
%     % 搜索新的前沿簇
%     newFrontiers = searchNewFrontiers(updatedRegion, map);
%     for i = 1:length(newFrontiers)
%         if length(newFrontiers{i}) > clusterSizeThreshold
%             newClusters = splitLargeClusters(newFrontiers{i}, clusterSizeThreshold, pcaThreshold);
%             FIS.Clusters = [FIS.Clusters, newClusters];
%         end
%     end
% end

% function affectedClusters = detectAffectedClusters(FIS, updatedRegion)
%     affectedClusters = [];
%     for i = 1:length(FIS.Clusters)
%         if intersects(FIS.AABB{i}, updatedRegion)
%             affectedClusters = [affectedClusters, i];
%         end
%     end
% end

% function result = intersects(AABB1, AABB2)
%     result = ~(AABB1(1) > AABB2(2) || AABB1(2) < AABB2(1) || ...
%                AABB1(3) > AABB2(4) || AABB1(4) < AABB2(3));
% end

% function isFront = isFrontier(cluster, map)
%     isFront = false;
%     for i = 1:length(cluster)
%         if isFrontierCell(cluster{i}, map)
%             isFront = true;
%             break;
%         end
%     end
% end

% function FIS = removeCluster(FIS, clusterIndex)
%     FIS.Clusters(clusterIndex) = [];
%     FIS.AveragePositions(clusterIndex) = [];
%     FIS.AABB(clusterIndex) = [];
%     FIS.Viewpoints(clusterIndex) = [];
%     FIS.ConnectionCosts(clusterIndex) = [];
% end

% function newClusters = splitLargeClusters(cluster, clusterSizeThreshold, pcaThreshold)
%     newClusters = {cluster};
%     % 使用PCA分割大的前沿簇
%     while true
%         largestCluster = newClusters{end};
%         if length(largestCluster) <= clusterSizeThreshold
%             break;
%         end
%         [coeff, score, ~, ~, explained] = pca(cell2mat(largestCluster'));
%         if explained(1) > pcaThreshold
%             splitIndex = score(:,1) > 0;
%             newClusters{end} = largestCluster(splitIndex);
%             newClusters{end+1} = largestCluster(~splitIndex);
%         else
%             break;
%         end
%     end
% end

% function frontiers = searchNewFrontiers(updatedRegion, map)
%     % 初始化前沿簇
%     frontiers = {};
%     visited = false(size(map));  % 访问标志数组
%     [X, Y] = ndgrid(updatedRegion(1):updatedRegion(2), ...
%                     updatedRegion(3):updatedRegion(4));
%     queue = {};  % 用于存储待处理的前沿单元格
    
%     % 遍历更新区域内的所有单元格
%     for idx = 1:numel(X)
%         x = X(idx);
%         y = Y(idx);
%         if map(x, y) == 1 && isFrontierCell([x, y], map) && ~visited(x, y)
%             % 创建一个新的前沿簇
%             frontier = {[x, y]};
%             queue{end+1} = [x, y];  % 添加到队列中
%             visited(x, y) = true;
            
%             % 执行区域增长
%             while ~isempty(queue)
%                 current = queue{1};
%                 queue(1) = [];
%                 neighbors = getNeighbors(current, size(map));
%                 for i = 1:size(neighbors, 1)
%                     nx = neighbors(i, 1);
%                     ny = neighbors(i, 2);
%                     if map(nx, ny) == 1 && isFrontierCell([nx, ny], map) && ~visited(nx, ny)
%                         frontier{end+1} = [nx, ny];
%                         queue{end+1} = [nx, ny];
%                         visited(nx, ny) = true;
%                     end
%                 end
%             end
            
%             % 将生成的前沿簇添加到前沿列表中
%             frontiers{end+1} = frontier;
%         end
%     end
% end

% function isFrontier = isFrontierCell(cell, map)
%     % 简单示例：如果单元格周围有未知区域（值为2），则认为它是前沿
%     neighbors = getNeighbors(cell, size(map));
%     isFrontier = any(map(sub2ind(size(map), neighbors(:, 1), neighbors(:, 2))) == 2);
% end

% function neighbors = getNeighbors(cell, mapSize)
%     x = cell(1);
%     y = cell(2);
%     neighbors = [];
%     for i = -1:1
%         for j = -1:1
%             if i == 0 && j == 0
%                 continue;
%             end
%             nx = x + i;
%             ny = y + j;
%             if nx > 0 && nx <= mapSize(1) && ny > 0 && ny <= mapSize(2)
%                 neighbors = [neighbors; nx, ny];
%             end
%         end
%     end
% end

function FIS = updateMap(FIS, updatedRegion, map, clusterSizeThreshold, pcaThreshold)
    % 检测受影响的前沿簇
    affectedClusters = detectAffectedClusters(FIS, updatedRegion);
    
    % 移除过时的前沿簇
    for i = length(affectedClusters):-1:1
        if ~isFrontier(FIS.Clusters{affectedClusters(i)}, map)
            FIS = removeCluster(FIS, affectedClusters(i));
        end
    end
    
    % 搜索新的前沿簇
    newFrontiers = searchNewFrontiers(updatedRegion, map);
    for i = 1:length(newFrontiers)
        if length(newFrontiers{i}) > clusterSizeThreshold
            newClusters = splitLargeClusters(newFrontiers{i}, clusterSizeThreshold, pcaThreshold);
            FIS.Clusters = [FIS.Clusters, newClusters];
        end
    end
end

function affectedClusters = detectAffectedClusters(FIS, updatedRegion)
    affectedClusters = [];
    for i = 1:length(FIS.Clusters)
        if intersects(FIS.AABB{i}, updatedRegion)
            affectedClusters = [affectedClusters, i];
        end
    end
end

function result = intersects(AABB1, AABB2)
    result = ~(AABB1(1) > AABB2(2) || AABB1(2) < AABB2(1) || ...
               AABB1(3) > AABB2(4) || AABB1(4) < AABB2(3));
end

function isFront = isFrontier(cluster, map)
    isFront = false;
    for i = 1:length(cluster)
        if isFrontierCell(cluster{i}, map)
            isFront = true;
            break;
        end
    end
end

function FIS = removeCluster(FIS, clusterIndex)
    FIS.Clusters(clusterIndex) = [];
    FIS.AveragePositions(clusterIndex) = [];
    FIS.AABB(clusterIndex) = [];
    FIS.Viewpoints(clusterIndex) = [];
    FIS.ConnectionCosts(clusterIndex) = [];
end

function newClusters = splitLargeClusters(cluster, clusterSizeThreshold, pcaThreshold)
    newClusters = {cluster};
    % 使用PCA分割大的前沿簇
    while true
        largestCluster = newClusters{end};
        if length(largestCluster) <= clusterSizeThreshold
            break;
        end
        [coeff, score, ~, ~, explained] = pca(cell2mat(largestCluster'));
        if explained(1) > pcaThreshold
            splitIndex = score(:,1) > 0;
            newClusters{end} = largestCluster(splitIndex);
            newClusters{end+1} = largestCluster(~splitIndex);
        else
            break;
        end
    end
end

function frontiers = searchNewFrontiers(updatedRegion, map)
    % 初始化前沿簇
    frontiers = {};
    visited = false(size(map));  % 访问标志数组
    [X, Y] = ndgrid(updatedRegion(1):updatedRegion(2), ...
                    updatedRegion(3):updatedRegion(4));
    queue = {};  % 用于存储待处理的前沿单元格
    
    % 遍历更新区域内的所有单元格
    for idx = 1:numel(X)
        x = X(idx);
        y = Y(idx);
        if map(x, y) == 1 && isFrontierCell([x, y], map) && ~visited(x, y)
            % 创建一个新的前沿簇
            frontier = {[x, y]};
            queue{end+1} = [x, y];  % 添加到队列中
            visited(x, y) = true;
            
            % 执行区域增长
            while ~isempty(queue)
                current = queue{1};
                queue(1) = [];
                neighbors = getNeighbors(current, size(map));
                for i = 1:size(neighbors, 1)
                    nx = neighbors(i, 1);
                    ny = neighbors(i, 2);
                    if map(nx, ny) == 1 && isFrontierCell([nx, ny], map) && ~visited(nx, ny)
                        frontier{end+1} = [nx, ny];
                        queue{end+1} = [nx, ny];
                        visited(nx, ny) = true;
                    end
                end
            end
            
            % 将生成的前沿簇添加到前沿列表中
            frontiers{end+1} = frontier;
        end
    end
end

function isFrontier = isFrontierCell(cell, map)
    % 简单示例：如果单元格周围有未知区域（值为2），则认为它是前沿
    neighbors = getNeighbors(cell, size(map));
    isFrontier = any(map(sub2ind(size(map), neighbors(:, 1), neighbors(:, 2))) == 2);
end

function neighbors = getNeighbors(cell, mapSize)
    x = cell(1);
    y = cell(2);
    neighbors = [];
    for i = -1:1
        for j = -1:1
            if i == 0 && j == 0
                continue;
            end
            nx = x + i;
            ny = y + j;
            if nx > 0 && nx <= mapSize(1) && ny > 0 && ny <= mapSize(2)
                neighbors = [neighbors; nx, ny];
            end
        end
    end
end

