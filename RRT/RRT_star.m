function [path, explored_points] = RRT_star(shp, workspace_limits, start_point, goal_point, ball_center, ball_radius)
    % RRT*算法参数
    max_iterations = 20000; % 最大迭代次数
    step_size = 2; % 每步移动的距离
    goal_tolerance = 1; % 目标点容差
    search_radius = 10; % 搜索半径

    % 初始化RRT*树
    tree = start_point;
    costs = 0; % 记录树中节点的成本
    parents = 0; % 记录树中节点的父节点
    explored_points = start_point; % 记录所有被遍历到的点

    for iter = 1:max_iterations
        % 生成随机节点
        q_rand = samplePoint(shp, workspace_limits, goal_point);

        % 找到最近的树节点
        nearest_idx = nearest_node(tree, q_rand);
        q_nearest = tree(nearest_idx, :);

        % 生成新节点
        q_new = gen_node(q_nearest, q_rand, step_size);

        % 检查新节点是否在工作空间范围内且无碰撞
        if check_workspace_limits(q_new, workspace_limits) && ~check_collision(q_new, ball_center, ball_radius)
            % 找到搜索半径内的所有节点
            near_indices = find_near_nodes(tree, q_new, search_radius);

            % 选择最小成本节点作为父节点
            [min_cost, min_idx] = min_cost_node(tree, q_new, near_indices, costs, step_size);
            if min_idx > 0
                tree = [tree; q_new];
                parents = [parents; min_idx];
                costs = [costs; min_cost];
                explored_points = [explored_points; q_new]; % 记录新节点

                % 重连路径优化
                tree = rewire(tree, q_new, near_indices, parents, costs, step_size,workspace_limits, ball_center, ball_radius);
                
                % 检查是否到达目标
                if norm(q_new - goal_point) < goal_tolerance
                    disp('找到路径');
                    break;
                end
            end
        end
    end

    % 回溯路径
    path = goal_point;
    current_idx = size(tree, 1);
    while current_idx > 0
        path = [tree(current_idx, :); path];
        current_idx = parents(current_idx);
    end
    disp('RRT*路径生成完成');
end


function q_rand = samplePoint(shp,workspace,goalPoint)
    if rand<0.8
        q_rand = rand(1, 3) .* (workspace(:, 2) - workspace(:, 1))' + workspace(:, 1)';
        while ~inShape(shp, q_rand)
            q_rand = rand(1, 3) .* (workspace(:, 2) - workspace(:, 1))' + workspace(:, 1)';
        end
    else
        q_rand = goalPoint;  
    end
end

function nearest_idx = nearest_node(tree, q_rand)
    % 找到树中距离随机节点最近的节点
    distances = vecnorm(tree - q_rand, 2, 2);
    [~, nearest_idx] = min(distances);
end

function q_new = gen_node(q_nearest, q_rand, step_size)
    % 生成新节点，沿着q_nearest指向q_rand的方向移动step_size
    direction = (q_rand - q_nearest) / norm(q_rand - q_nearest);
    q_new = q_nearest + step_size * direction;
end

function valid = check_workspace_limits(q, workspace_limits)
    % 检查节点是否在工作空间范围内
    valid = all(q >= workspace_limits(:, 1)') && all(q <= workspace_limits(:, 2)');
end

function collision = check_collision(q, ball_center, ball_radius)
    % 检查节点是否与球体碰撞
    collision = norm(q - ball_center) <= ball_radius;
end

function near_indices = find_near_nodes(tree, q_new, search_radius)
    % 找到树中所有距离q_new在search_radius以内的节点
    distances = vecnorm(tree - q_new, 2, 2);
    near_indices = find(distances < search_radius);
end

function [min_cost, min_idx] = min_cost_node(tree, q_new, near_indices, costs, step_size)
    % 选择最小成本节点作为父节点
    min_cost = Inf;
    min_idx = -1;
    for idx = near_indices'
        q_near = tree(idx, :);
        new_cost = costs(idx) + norm(q_new - q_near);
        if new_cost < min_cost
            min_cost = new_cost;
            min_idx = idx;
        end
    end
end

function tree = rewire(tree, q_new, near_indices, parents, costs, step_size,workspace_limits, ball_center, ball_radius)
    % 路径重连，优化路径
    for idx = near_indices'
        q_near = tree(idx, :);
        new_cost = costs(end) + norm(q_new - q_near);
        if new_cost < costs(idx) && check_workspace_limits(q_new, workspace_limits) && ~check_collision(q_new, ball_center, ball_radius)
            parents(idx) = size(tree, 1);
            costs(idx) = new_cost;
        end
    end
end
