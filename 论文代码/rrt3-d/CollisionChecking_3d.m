function feasible = CollisionChecking_3d(x_start, x_goal)
    % 判断从x_start到x_goal的路径是否会发生碰撞
    % 如果碰撞返回false，否则返回true
    
        feasible = true; % 初始化为可行
        sphere_center = [67,35,54,81,80,77,24,40,74,50; % 障碍物球体中心的x坐标
                         89,83,58,17,72,71,85,73,69,50; % 障碍物球体中心的y坐标
                         47,84,78,68,41,79,40,74,10,50]; % 障碍物球体中心的z坐标
        % 计算起点到终点的角度
        theta = atan2((x_goal(2) - x_start(2)), (x_goal(1) - x_start(1))); % 计算水平角度
        alpha = atan2((x_goal(3) - x_start(3)), sqrt((x_goal(2) - x_start(2))^2 + (x_goal(1) - x_start(1))^2)); % 计算垂直角度
    
        % 沿直线从起点到终点进行碰撞检测
        for r = 0:0.5:sqrt(sum((x_goal - x_start).^2)) % 以0.5为步长，从起点沿直线检查
            % 计算当前检测点的坐标
            posCheck(1) = x_start(1) + cos(theta) * cos(alpha) * r;
            posCheck(2) = x_start(2) + sin(theta) * cos(alpha) * r;
            posCheck(3) = x_start(3) + sin(alpha) * r; % 计算当前检测点的z坐标
    
            % 检查当前点是否在边界内且不与任何障碍物碰撞
            if ~(posCheck(1) > 0 && posCheck(1) < 100 ... % 检查x坐标是否在边界内
               && posCheck(2) > 0 && posCheck(2) < 100 ... % 检查y坐标是否在边界内
               && posCheck(3) > 0 && posCheck(3) < 100 ... % 检查z坐标是否在边界内
               && sqrt(sum((posCheck' - sphere_center(:, 1)).^2)) > 11 ... % 检查是否碰撞第一个球体
               && sqrt(sum((posCheck' - sphere_center(:, 2)).^2)) > 11 ... % 检查是否碰撞第二个球体
               && sqrt(sum((posCheck' - sphere_center(:, 3)).^2)) > 11 ... % 检查是否碰撞第三个球体
               && sqrt(sum((posCheck' - sphere_center(:, 4)).^2)) > 11 ... % 检查是否碰撞第四个球体
               && sqrt(sum((posCheck' - sphere_center(:, 5)).^2)) > 11 ... % 检查是否碰撞第五个球体
               && sqrt(sum((posCheck' - sphere_center(:, 6)).^2)) > 11 ... % 检查是否碰撞第六个球体
               && sqrt(sum((posCheck' - sphere_center(:, 7)).^2)) > 11 ... % 检查是否碰撞第七个球体
               && sqrt(sum((posCheck' - sphere_center(:, 8)).^2)) > 11 ... % 检查是否碰撞第八个球体
               && sqrt(sum((posCheck' - sphere_center(:, 9)).^2)) > 11 ... % 检查是否碰撞第九个球体
               && sqrt(sum((posCheck' - sphere_center(:, 10)).^2)) > 11) % 检查是否碰撞第十个球体
                feasible = false; % 如果任一条件不满足，则发生碰撞
                break; % 终止循环
            end
        end
    end
    