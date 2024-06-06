function [path,explored_points] = RRT(shp,workspace_limits,start_point,goal_point,ball_center,ball_radius)
% RRT算法参数
max_iterations = 10000; % 最大迭代次数
step_size = 3; % 每步移动的距离
goal_tolerance = 1; % 目标点容差

% 初始化RRT树
tree = start_point;
parents = 0; % 记录树中节点的父节点
explored_points = start_point; % 记录所有被遍历到的点

for iter = 1:max_iterations
    % 生成随机节点
    q_rand = samplePoint(shp,workspace_limits,goal_point);
    
    % 找到最近的树节点
    nearest_idx = nearest_node(tree, q_rand);
    q_nearest = tree(nearest_idx, :);
    
    % 生成新节点
    q_new = gen_node(q_nearest, q_rand, step_size);
    
    % 检查新节点是否在工作空间范围内且无碰撞
    if check_workspace_limits(q_new, workspace_limits) && ~check_collision(q_new, ball_center, ball_radius)
        % 添加新节点到树
        tree = [tree; q_new];
        parents = [parents; nearest_idx];
        explored_points = [explored_points; q_new]; % 记录新节点
%         disp(norm(q_new - goal_point))
        % 检查是否到达目标
        if norm(q_new - goal_point) < goal_tolerance
            disp('找到路径');
            break;
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
disp('RRT路径生成完成');
end

