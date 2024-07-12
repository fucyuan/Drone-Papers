% % % 三维空间中的快速扩展随机树（RRT）路径规划算法
% % clear all; % 清除所有变量
% % clc; % 清除命令行窗口
% % clf; % 清除图形窗口

% % % 定义起点、终点、阈值和扩展步长
% % x_I = 5; % 起点的x坐标
% % y_I = 5; % 起点的y坐标
% % z_I = 5; % 起点的z坐标
% % x_G = 95; % 终点的x坐标
% % y_G = 95; % 终点的y坐标
% % z_G = 95; % 终点的z坐标
% % threshold = 8; % 阈值，用于判断是否到达目标
% % Extended_step_size = 8; % 扩展步长
% % MaxIterations = 3000; % 最大迭代次数

% % % 初始化树
% % T.v(1).x = x_I; % 起点的x坐标
% % T.v(1).y = y_I; % 起点的y坐标
% % T.v(1).z = z_I; % 起点的z坐标
% % T.v(1).xPrev = x_I; % 起点的父节点x坐标
% % T.v(1).yPrev = y_I; % 起点的父节点y坐标
% % T.v(1).zPrev = z_I; % 起点的父节点z坐标
% % T.v(1).dist = 0; % 从父节点到当前节点的距离
% % T.v(1).indPrev = 0; % 父节点的索引

% % % 障碍物的数量和位置
% % sphere_num = 10; % 障碍物球体的数量
% % % sphere_center = round(10 + 80 * rand(3, sphere_num)); % 随机生成障碍物中心，范围在[10,90]之间
% % sphere_center = [67,35,54,81,80,77,24,40,74,50; % 障碍物球体中心的x坐标
% %                  89,83,58,17,72,71,85,73,69,50; % 障碍物球体中心的y坐标
% %                  47,84,78,68,41,79,40,74,10,50]; % 障碍物球体中心的z坐标
% % % radius = round(10 + 5 * rand(1, sphere_num)); % 随机生成障碍物的半径，范围在[10,15]之间
% % radius = ones(1, 10) * 10; % 固定障碍物的半径为10
% % [x, y, z] = sphere(15); % 生成球体的点云数据

% % % 绘制障碍物
% % for i = 1:sphere_num
% %     surf(radius(i) * x + sphere_center(1, i), ... % 绘制每个障碍物的x坐标
% %          radius(i) * y + sphere_center(2, i), ... % 绘制每个障碍物的y坐标
% %          radius(i) * z + sphere_center(3, i)); % 绘制每个障碍物的z坐标
% %     hold on; % 保持图形窗口，继续绘制下一个障碍物
% % end

% % % 设置图形参数
% % axis equal;
% % axis([0 100 0 100 0 100]);
% % xlabel('X axis(cm)');
% % ylabel('Y axis(cm)');
% % zlabel('Z axis(cm)');
% % title('RRT Algorithm');
% % sphere_RGB = [1, 0.5, 0; % 障碍物颜色RGB值
% %               1, 0.5, 0;
% %               1, 0.5, 0;
% %               1, 0.5, 0;
% %               1, 0.5, 0];
% % colormap(sphere_RGB); % 设置障碍物的颜色
% % hold on;
% % plot3(x_I, y_I, z_I, 'mo', 'MarkerSize', 5, 'MarkerFaceColor', 'm'); % 绘制起点
% % plot3(x_G, y_G, z_G, 'go', 'MarkerSize', 5, 'MarkerFaceColor', 'g'); % 绘制终点

% % % 进行路径扩展
% % count = 1; % 初始化节点计数
% % tic % 开始计时
% % for iter = 1:MaxIterations
% %     % Step 1: 在节点图中随机采样一个节点x_rand
% %     x_rand = [unifrnd(0, 100), unifrnd(0, 100), unifrnd(0, 100)]; % 生成随机节点(x, y, z)
    
% %     % Step 2: 找到树中距离x_rand最近的节点x_near
% %     minDis = sqrt((x_rand(1) - T.v(1).x)^2 + (x_rand(2) - T.v(1).y)^2 + (x_rand(3) - T.v(1).z)^2); % 计算距离
% %     minIndex = 1;
% %     for i = 2:size(T.v, 2) % 遍历树中的所有节点
% %         distance = sqrt((x_rand(1) - T.v(i).x)^2 + (x_rand(2) - T.v(i).y)^2 + (x_rand(3) - T.v(i).z)^2); % 计算距离
% %         if distance < minDis
% %             minDis = distance;
% %             minIndex = i; % 更新最近节点的索引
% %         end
% %     end
% %     x_near(1) = T.v(minIndex).x; % 最近节点的x坐标
% %     x_near(2) = T.v(minIndex).y;
% %     x_near(3) = T.v(minIndex).z;
    
% %     % Step 3: 从x_near扩展得到x_new节点
% %     orientation = [x_rand(1) - x_near(1), x_rand(2) - x_near(2), x_rand(3) - x_near(3)]; 
% %     mo = sqrt(orientation(1)^2 + orientation(2)^2 + orientation(3)^2);
% %     jiajiao = [orientation(1) / mo, orientation(2) / mo, orientation(3) / mo];
% %     x_new(1) = x_near(1) + Extended_step_size * jiajiao(1);
% %     x_new(2) = x_near(2) + Extended_step_size * jiajiao(2);
% %     x_new(3) = x_near(3) + Extended_step_size * jiajiao(3);
    
% %     % Step 4: 检查x_new是否碰撞障碍物，如果碰撞则返回Step 1，否则继续
% %     if ~CollisionChecking_3d(x_near, x_new) 
% %         continue; % 碰撞则跳过此次扩展
% %     else
% %         count = count + 1;
% %     end
    
% %     % Step 5: 将x_new添加到树中
% %     T.v(count).x = x_new(1);
% %     T.v(count).y = x_new(2);
% %     T.v(count).z = x_new(3);
% %     T.v(count).xPrev = x_near(1);
% %     T.v(count).yPrev = x_near(2);
% %     T.v(count).zPrev = x_near(3);
% %     T.v(count).dist = Extended_step_size;
% %     T.v(count).indPrev = minIndex; % 设置父节点的索引
    
% %     % Step 6: 检查是否到达目标点，距离阈值为8
% %     disToGoal = sqrt((x_new(1) - x_G)^2 + (x_new(2) - y_G)^2 + (x_new(3) - z_G)^2);
% %     if disToGoal < threshold
% %         break;
% %     end
    
% %     % Step 7: 绘制从x_near到x_new的路径
% %     plot3([x_near(1), x_new(1)], [x_near(2), x_new(2)], [x_near(3), x_new(3)], 'y', 'Linewidth', 2);
% %     plot3(x_new(1), x_new(2), x_new(3), 'go', 'MarkerSize', 2, 'MarkerFaceColor', 'g');
% %     pause(0.001); % 暂停0.001秒以便观察RRT扩展过程
% % end

% % % 找到路径后，回溯路径节点
% % path_pos(1).x = x_G;
% % path_pos(1).y = y_G;
% % path_pos(1).z = z_G;
% % path_pos(2).x = T.v(end).x;
% % path_pos(2).y = T.v(end).y;
% % path_pos(2).z = T.v(end).z;
% % pathIndex = T.v(end).indPrev; % 终点的父节点索引
% % j = 0;
% % while true
% %     path_pos(j + 3).x = T.v(pathIndex).x;
% %     path_pos(j + 3).y = T.v(pathIndex).y;
% %     path_pos(j + 3).z = T.v(pathIndex).z; % 记录路径节点的z坐标
% %     pathIndex = T.v(pathIndex).indPrev; % 更新路径索引，指向当前节点的父节点
% %     if pathIndex == 0 % 如果回溯到起点，则停止
% %         break;
% %     end
% %     j = j + 1; % 增加路径节点计数
% % end

% % % 绘制最终路径
% % for j = 2:length(path_pos)
% %     plot3([path_pos(j).x, path_pos(j-1).x], [path_pos(j).y, path_pos(j-1).y], [path_pos(j).z, path_pos(j-1).z], 'b', 'Linewidth', 3);
% % end

% % disp('The path is found!'); % 显示路径已找到
% % fprintf('The time of the RRT Algorithm is %.2f seconds.\n', toc); % 显示算法运行时间
% % fprintf('Number of sampling nodes in space is %.0f.\n', size(T.v, 2)); % 显示采样节点数量
% % fprintf('Number of effective nodes is %.0f.\n', size(path_pos, 2)); % 显示有效节点数量
% % fprintf('The path length is %.0f centimeters.\n', ... % 显示路径长度
% %     Extended_step_size * (size(path_pos, 2) - 2) + ...
% %     sqrt((path_pos(1).x - path_pos(2).x)^2 + (path_pos(1).y - path_pos(2).y)^2 + (path_pos(1).z - path_pos(2).z)^2));


% clear all;
% clc;
% clf;

% % 初始化起点、终点、阈值和网格大小
% x_I = 5; % 起点的x坐标
% y_I = 5; % 起点的y坐标
% z_I = 5; % 起点的z坐标
% x_G = 95; % 终点的x坐标
% y_G = 95; % 终点的y坐标
% z_G = 95; % 终点的z坐标
% gridSize = 100; % 网格大小

% % 障碍物初始化
% sphere_num = 10;
% sphere_center = [67,35,54,81,80,77,24,40,74,50; % 障碍物球体中心的x坐标
%                  89,83,58,17,72,71,85,73,69,50; % 障碍物球体中心的y坐标
%                  47,84,78,68,41,79,40,74,10,50]; % 障碍物球体中心的z坐标
% radius = ones(1, 10) * 10; % 障碍物球体半径
% [x, y, z] = sphere(15); % 生成球体的点云数据

% % 绘制障碍物
% figure;
% hold on;
% for i = 1:sphere_num
%     surf(radius(i) * x + sphere_center(1, i), ... % 绘制每个障碍物的x坐标
%          radius(i) * y + sphere_center(2, i), ... % 绘制每个障碍物的y坐标
%          radius(i) * z + sphere_center(3, i)); % 绘制每个障碍物的z坐标
% end

% % 设置图形参数
% axis equal;
% axis([0 gridSize 0 gridSize 0 gridSize]);
% xlabel('X axis(cm)');
% ylabel('Y axis(cm)');
% zlabel('Z axis(cm)');
% title('A* Algorithm');
% plot3(x_I, y_I, z_I, 'mo', 'MarkerSize', 5, 'MarkerFaceColor', 'm'); % 绘制起点
% plot3(x_G, y_G, z_G, 'go', 'MarkerSize', 5, 'MarkerFaceColor', 'g'); % 绘制终点

% % A*算法
% openList = PriorityQueue(); % 初始化优先队列（open list）
% closedList = []; % 初始化已处理节点列表（closed list）

% % 初始化起点节点
% startNode.x = x_I;
% startNode.y = y_I;
% startNode.z = z_I;
% startNode.g = 0; % 起点的g值（从起点到当前节点的代价）
% startNode.h = heuristic(startNode, x_G, y_G, z_G); % 启发式函数h值
% startNode.f = startNode.g + startNode.h; % f值
% startNode.parent = []; % 起点没有父节点

% openList.insert(startNode, startNode.f); % 将起点节点插入优先队列

% while ~openList.isEmpty()
%     currentNode = openList.pop(); % 从优先队列中取出f值最小的节点
    
%     % 如果当前节点是目标节点，则结束搜索
%     if isGoal(currentNode, x_G, y_G, z_G)
%         break;
%     end
    
%     % 将当前节点加入到已处理节点列表中
%     closedList = [closedList; currentNode];
    
%     % 处理当前节点的邻居节点
%     neighbors = getNeighbors(currentNode, gridSize);
%     for i = 1:length(neighbors)
%         neighbor = neighbors(i);
        
%         % 初始化邻居节点的g、h、f值和parent
%         if ~isfield(neighbor, 'g')
%             neighbor.g = inf;
%             neighbor.h = inf;
%             neighbor.f = inf;
%             neighbor.parent = [];
%         end
        
%         % 进行碰撞检测，如果发生碰撞则跳过
%         if ~CollisionChecking(currentNode, neighbor, sphere_center, radius)
%             continue;
%         end
        
%         % 计算邻居节点的g值
%         tentative_g = currentNode.g + distance(currentNode, neighbor);
        
%         % 检查邻居节点是否在已处理节点列表中
%         if isInList(closedList, neighbor)
%             continue;
%         end
        
%         % 如果邻居节点不在openList中，或者找到了一条更短的路径
%         if ~openList.contains(neighbor) || tentative_g < neighbor.g
%             neighbor.g = tentative_g;
%             neighbor.h = heuristic(neighbor, x_G, y_G, z_G); % 计算h值
%             neighbor.f = neighbor.g + neighbor.h; % 计算f值
%             neighbor.parent = currentNode; % 设置父节点
            
%             if ~openList.contains(neighbor)
%                 openList.insert(neighbor, neighbor.f); % 将邻居节点插入优先队列
%             end
%         end
%     end
% end

% % 路径回溯
% path = [];
% while ~isempty(currentNode.parent)
%     path = [currentNode; path]; % 将当前节点添加到路径中
%     currentNode = currentNode.parent; % 回溯到父节点
% end
% path = [startNode; path]; % 将起点节点添加到路径中

% % 绘制路径
% for i = 2:length(path)
%     plot3([path(i).x, path(i-1).x], [path(i).y, path(i-1).y], [path(i).z, path(i-1).z], 'b', 'Linewidth', 3);
% end

% disp('The path is found!'); % 显示路径已找到
% fprintf('The time of the A* Algorithm is %.2f seconds.\n', toc); % 显示算法运行时间
% fprintf('Number of nodes in open list is %.0f.\n', openList.size()); % 显示open list中的节点数量
% fprintf('Number of nodes in closed list is %.0f.\n', length(closedList)); % 显示closed list中的节点数量

% % 启发式函数，计算当前节点到目标节点的启发式代价
% function h = heuristic(node, x_G, y_G, z_G)
%     h = sqrt((node.x - x_G)^2 + (node.y - y_G)^2 + (node.z - z_G)^2);
% end

% % 判断节点是否为目标节点
% function goal = isGoal(node, x_G, y_G, z_G)
%     goal = (node.x == x_G && node.y == y_G && node.z == z_G);
% end

% % 获取当前节点的邻居节点
% function neighbors = getNeighbors(node, gridSize)
%     neighbors = [];
%     stepSize = 1; % 邻居节点的步长
%     for dx = -stepSize:stepSize:stepSize
%         for dy = -stepSize:stepSize:stepSize
%             for dz = -stepSize:stepSize:stepSize
%                 if dx == 0 && dy == 0 && dz == 0
%                     continue; % 跳过当前节点
%                 end
%                 neighbor.x = node.x + dx;
%                 neighbor.y = node.y + dy;
%                 neighbor.z = node.z + dz;
%                 if neighbor.x > 0 && neighbor.x <= gridSize && ...
%                    neighbor.y > 0 && neighbor.y <= gridSize && ...
%                    neighbor.z > 0 && neighbor.z <= gridSize
%                     neighbors = [neighbors; neighbor]; % 添加有效的邻居节点
%                 end
%             end
%         end
%     end
% end

% % 计算两个节点之间的距离
% function d = distance(node1, node2)
%     d = sqrt((node1.x - node2.x)^2 + (node1.y - node2.y)^2 + (node1.z - node2.z)^2);
% end

% % 判断节点是否在列表中
% function inList = isInList(list, node)
%     inList = false;
%     for i = 1:length(list)
%         if list(i).x == node.x && list(i).y == node.y && list(i).z == node.z
%             inList = true;
%             break;
%         end
%     end
% end

% % 碰撞检测函数
% function feasible = CollisionChecking(startNode, endNode, sphere_center, radius)
%     feasible = true;
%     steps = 10;
%     for i = 1:steps
%         t = i / steps;
%         x = startNode.x + t * (endNode.x - startNode.x);
%         y = startNode.y + t * (endNode.y - startNode.y);
%         z = startNode.z + t * (endNode.z - startNode.z);
%         for j = 1:size(sphere_center, 2)
%             if sqrt((x - sphere_center(1, j))^2 + (y - sphere_center(2, j))^2 + (z - sphere_center(3, j))^2) < radius(j)
%                 feasible = false;
%                 return;
%             end
%         end
%     end
% end
clear all;
clc;
clf;

% 初始化起点、终点、阈值和网格大小
x_I = 5; % 起点的x坐标
y_I = 5; % 起点的y坐标
z_I = 5; % 起点的z坐标
x_G = 95; % 终点的x坐标
y_G = 95; % 终点的y坐标
z_G = 95; % 终点的z坐标
gridSize = 100; % 网格大小

% 障碍物初始化
sphere_num = 5; % 临时减少障碍物数量
sphere_center = [67,35,54,81,80; % 障碍物球体中心的x坐标
                 89,83,58,17,72; % 障碍物球体中心的y坐标
                 47,84,78,68,41]; % 障碍物球体中心的z坐标
radius = ones(1, sphere_num) * 10; % 障碍物球体半径
[x, y, z] = sphere(15); % 生成球体的点云数据

% 绘制障碍物
figure;
hold on;
for i = 1:sphere_num
    surf(radius(i) * x + sphere_center(1, i), ... % 绘制每个障碍物的x坐标
         radius(i) * y + sphere_center(2, i), ... % 绘制每个障碍物的y坐标
         radius(i) * z + sphere_center(3, i)); % 绘制每个障碍物的z坐标
end

% 设置图形参数
axis equal;
axis([0 gridSize 0 gridSize 0 gridSize]);
xlabel('X axis(cm)');
ylabel('Y axis(cm)');
zlabel('Z axis(cm)');
title('A* Algorithm');
plot3(x_I, y_I, z_I, 'mo', 'MarkerSize', 5, 'MarkerFaceColor', 'm'); % 绘制起点
plot3(x_G, y_G, z_G, 'go', 'MarkerSize', 5, 'MarkerFaceColor', 'g'); % 绘制终点

% A*算法
openList = PriorityQueue(); % 初始化优先队列（open list）
closedList = []; % 初始化已处理节点列表（closed list）

% 初始化起点节点
startNode.x = x_I;
startNode.y = y_I;
startNode.z = z_I;
startNode.g = 0; % 起点的g值（从起点到当前节点的代价）
startNode.h = heuristic(startNode, x_G, y_G, z_G); % 启发式函数h值
startNode.f = startNode.g + startNode.h; % f值
startNode.parent = []; % 起点没有父节点

openList.insert(startNode, startNode.f); % 将起点节点插入优先队列

while ~openList.isEmpty()
    currentNode = openList.pop(); % 从优先队列中取出f值最小的节点
    
    % 如果当前节点是目标节点，则结束搜索
    if isGoal(currentNode, x_G, y_G, z_G)
        break;
    end
    
    % 将当前节点加入到已处理节点列表中
    closedList = [closedList; currentNode];
    
    % 处理当前节点的邻居节点
    neighbors = getNeighbors(currentNode, gridSize);
    for i = 1:length(neighbors)
        neighbor = neighbors(i);
        
        % 初始化邻居节点的g、h、f值和parent
        if ~isfield(neighbor, 'g')
            neighbor.g = inf;
            neighbor.h = inf;
            neighbor.f = inf;
            neighbor.parent = [];
        end
        
        % 进行碰撞检测，如果发生碰撞则跳过
        if ~CollisionChecking(currentNode, neighbor, sphere_center, radius)
            continue;
        end
        
        % 计算邻居节点的g值
        tentative_g = currentNode.g + distance(currentNode, neighbor);
        
        % 检查邻居节点是否在已处理节点列表中
        if isInList(closedList, neighbor)
            continue;
        end
        
        % 如果邻居节点不在openList中，或者找到了一条更短的路径
        if ~openList.contains(neighbor) || tentative_g < neighbor.g
            neighbor.g = tentative_g;
            neighbor.h = heuristic(neighbor, x_G, y_G, z_G); % 计算h值
            neighbor.f = neighbor.g + neighbor.h; % 计算f值
            neighbor.parent = currentNode; % 设置父节点
            
            if ~openList.contains(neighbor)
                openList.insert(neighbor, neighbor.f); % 将邻居节点插入优先队列
            end
        end
    end
end

% 路径回溯
path = [];
while ~isempty(currentNode.parent)
    path = [currentNode; path]; % 将当前节点添加到路径中
    currentNode = currentNode.parent; % 回溯到父节点
end
path = [startNode; path]; % 将起点节点添加到路径中

% 绘制路径
for i = 2:length(path)
    plot3([path(i).x, path(i-1).x], [path(i).y, path(i-1).y], [path(i).z, path(i-1).z], 'b', 'Linewidth', 3);
end

disp('The path is found!'); % 显示路径已找到
fprintf('The time of the A* Algorithm is %.2f seconds.\n', toc); % 显示算法运行时间
fprintf('Number of nodes in open list is %.0f.\n', openList.size()); % 显示open list中的节点数量
fprintf('Number of nodes in closed list is %.0f.\n', length(closedList)); % 显示closed list中的节点数量

% 启发式函数，计算当前节点到目标节点的启发式代价
function h = heuristic(node, x_G, y_G, z_G)
    h = sqrt((node.x - x_G)^2 + (node.y - y_G)^2 + (node.z - z_G)^2);
end

% 判断节点是否为目标节点
function goal = isGoal(node, x_G, y_G, z_G)
    goal = (node.x == x_G && node.y == y_G && node.z == z_G);
end

% 获取当前节点的邻居节点
function neighbors = getNeighbors(node, gridSize)
    neighbors = [];
    stepSize = 1; % 邻居节点的步长
    for dx = -stepSize:stepSize:stepSize
        for dy = -stepSize:stepSize:stepSize
            for dz = -stepSize:stepSize:stepSize
                if dx == 0 && dy == 0 && dz == 0
                    continue; % 跳过当前节点
                end
                neighbor.x = node.x + dx;
                neighbor.y = node.y + dy;
                neighbor.z = node.z + dz;
                if neighbor.x > 0 && neighbor.x <= gridSize && ...
                   neighbor.y > 0 && neighbor.y <= gridSize && ...
                   neighbor.z > 0 && neighbor.z <= gridSize
                    neighbors = [neighbors; neighbor]; % 添加有效的邻居节点
                end
            end
        end
    end
end

% 计算两个节点之间的距离
function d = distance(node1, node2)
    d = sqrt((node1.x - node2.x)^2 + (node1.y - node2.y)^2 + (node1.z - node2.z)^2);
end

% 判断节点是否在列表中
function inList = isInList(list, node)
    inList = false;
    for i = 1:length(list)
        if list(i).x == node.x && list(i).y == node.y && list(i).z == node.z
            inList = true;
            break;
        end
    end
end

% 碰撞检测函数
function feasible = CollisionChecking(startNode, endNode, sphere_center, radius)
    feasible = true;
    steps = 5; % 减少步数
    for i = 1:steps
        t = i / steps;
        x = startNode.x + t * (endNode.x - startNode.x);
        y = startNode.y + t * (endNode.y - startNode.y);
        z = startNode.z + t * (endNode.z - startNode.z);
        for j = 1:size(sphere_center, 2)
            if sqrt((x - sphere_center(1, j))^2 + (y - sphere_center(2, j))^2 + (z - sphere_center(3, j))^2) < radius(j)
                feasible = false;
                return;
            end
        end
    end
end
