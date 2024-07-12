function FIS = generateViewpointsAndUpdateCosts(FIS, map)
    % 为每个簇生成一个视点
    for i = 1:length(FIS.Clusters)
        FIS.Viewpoints{i} = generateViewpoints(FIS.Clusters{i}, map);
    end
    % 计算连接成本
    FIS.ConnectionCosts = calculateConnectionCosts(FIS.Clusters, FIS.Viewpoints, map);
end

function viewpoints = generateViewpoints(cluster, map)
    % 生成簇的中心点作为视点
    cluster_mat = cell2mat(cluster');
    center_x = mean(cluster_mat(:, 1));
    center_y = mean(cluster_mat(:, 2));
    viewpoints = {[center_x, center_y]};
end

function costs = calculateConnectionCosts(clusters, viewpoints, map)
    numClusters = length(clusters);
    costs = zeros(numClusters, numClusters);
    % 计算连接成本
    for i = 1:numClusters
        for j = 1:numClusters
            if i ~= j
                viewpoint_i = cell2mat(viewpoints{i});
                viewpoint_j = cell2mat(viewpoints{j});
                % 确保两个向量具有相同的维度
                if size(viewpoint_i, 1) == 1
                    viewpoint_i = viewpoint_i';
                end
                if size(viewpoint_j, 1) == 1
                    viewpoint_j = viewpoint_j';
                end
                costs(i, j) = norm(viewpoint_i - viewpoint_j);
            end
        end
    end
end

