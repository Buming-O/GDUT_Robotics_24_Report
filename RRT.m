clc;
clear all;
close all;
global Link_1
global Link_2
global Link_3
Build;
load('FK_matrix.mat', 'T0end');
global FK;
FK = matlabFunction(T0end);
load('workspace_points1.mat', 'workspace_points1');
load('workspace_points2.mat', 'workspace_points2');
load('workspace_points3.mat', 'workspace_points3');
alpha_r=40;
shp1 = alphaShape(workspace_points1,alpha_r);
shp2 = alphaShape(workspace_points2,alpha_r);
shp3 = alphaShape(workspace_points3,alpha_r);

% 定义末端执行器的工作空间范围和障碍物
workspace_limits = [0, 120; -50, 50; -90, 0;]; % 工作空间范围
ball_center = [103.74, 0, -13.57]; % 障碍物中心
ball_radius = 10; % 障碍物半径

% 定义起始和目标点
start_point = [120, 0, 0]; % 替换为实际起始点
goal_point = [31.9265646451375,3.41292071861552,-83.8546943415400]; % 替换为实际目标点

% RRT算法参数
max_iterations = 10000; % 最大迭代次数
step_size = 1; % 每步移动的距离
goal_tolerance = 1; % 目标点容差

% 初始化RRT树
tree = start_point;
parents = 0; % 记录树中节点的父节点
explored_points = start_point; % 记录所有被遍历到的点

for iter = 1:max_iterations
    % 生成随机节点
    q_rand = samplePoint(shp1,workspace_limits,goal_point);
    
    % 找到最近的树节点
    nearest_idx = nearest_node(tree, q_rand);
    q_nearest = tree(nearest_idx, :);
    
    % 生成新节点
    q_new = steer(q_nearest, q_rand, step_size);
    
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

% 平滑路径
smoothed_path = smooth_end_effector_path(path);

% 绘制末端轨迹
figure;
plot3(smoothed_path(:, 1), smoothed_path(:, 2), smoothed_path(:, 3), 'r-', 'LineWidth', 1);
hold on;
% 显示所有遍历到的点
plot3(explored_points(:, 1), explored_points(:, 2), explored_points(:, 3), 'b.', 'MarkerSize', 1, 'MarkerFaceColor', 'k');
% 显示所有规划到的点
plot3(smoothed_path(:, 1), smoothed_path(:, 2), smoothed_path(:, 3), 'k.', 'MarkerSize', 1, 'MarkerFaceColor', 'k');
% 标记起点和终点
plot3(smoothed_path(1, 1), smoothed_path(1, 2), smoothed_path(1, 3), 'go', 'MarkerSize', 3, 'MarkerFaceColor', 'g'); % 起点
plot3(smoothed_path(end, 1), smoothed_path(end, 2), smoothed_path(end, 3), 'bo', 'MarkerSize', 3, 'MarkerFaceColor', 'b'); % 终点

q_0=repmat(0,1,4);
q_1=[0,60,60,60];%0,0,-51.9615
DHFk_hand(q_0,q_0,q_0,true);
DrawSphere(ball_center,ball_radius,0);
view(-21,12);
axis([-150,150,-150,150,-150,50]);
xlabel('X');
ylabel('Y');
zlabel('Z');
legend('End Effector Trajectory', 'Explored Points','Planned Points', 'Start', 'Goal');
title('End Effector Trajectory');
grid on;
axis equal;
%%
function q_rand = samplePoint(shp,workspace,goalPoint)
    if rand<0.8
        q_rand = rand(1, 3) .* (workspace(:, 2) - workspace(:, 1))' + workspace(:, 1)';
        while(inShape(shp, q_rand))
            q_rand = rand(1, 3) .* (workspace(:, 2) - workspace(:, 1))' + workspace(:, 1)';
        end
    else
        q_rand = goalPoint;  
    end
end

% 定义检查工作空间限制的函数
function in_limits = check_workspace_limits(point, workspace_limits)
    in_limits = all(point >= workspace_limits(:, 1)') && all(point <= workspace_limits(:, 2)');
end

% 定义检查碰撞的函数
function in_collision = check_collision(point, obstacle_center, obstacle_radius)
    in_collision = norm(point - obstacle_center) < obstacle_radius+5;
end

% 定义平滑路径函数
function smoothed_path = smooth_end_effector_path(path)
    num_points = size(path, 1);
    smoothed_path = path;
    for i = 2:num_points-1
        smoothed_path(i, :) = (path(i-1, :) + path(i+1, :)) / 2;
    end
end

%定义最近节点查找函数
function nearest_idx = nearest_node(tree, q_rand)
    tempDis = inf;
    for k1 = 1:size(tree, 1)
        dis=norm(q_rand - tree(k1,:));
        if tempDis>dis
            tempDis = dis;
            index = k1;
        end    
    end
    nearest_idx = index;
end

% 定义生成新节点的函数
function q_new = steer(q_nearest, q_rand, step_size)
    direction = q_rand - q_nearest;
    distance = norm(direction);
    q_new = q_nearest + (direction / distance) * min(step_size, distance);
end

function end_point = FK_end_point(q)
    global FK;  
    end_point= FK(0,q(1),q(2),q(3),q(4));
    end_point = end_point(1:3,4)';
end


