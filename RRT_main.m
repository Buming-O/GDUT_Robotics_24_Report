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
alpha_r=20;
shp1 = alphaShape(workspace_points1,alpha_r);
shp2 = alphaShape(workspace_points2,alpha_r);
shp3 = alphaShape(workspace_points3,alpha_r);

learning_rate = 0.9;
global th2_min th2_max th3_min th3_max th4_min th4_max th5_min th5_max;
th2_min=-90; th2_max=90;   % Range for th2
th3_min=0; th3_max=60;  % Range for th3
th4_min=0; th4_max=60;   % Range for th4
th5_min=0; th5_max=80;     % Range for th5

% 定义球的半径和中心位置
catch_ball_radius = 40;
catch_ball_center = [0, 0, -60];
% 定义物体参数
mass = 0.5; % 物体质量，单位kg
g = 9.81; % 重力加速度，单位m/s^2
weight = mass * g; % 物体的重量

%% 抓取点规划
num_points = 60;
angle_threshold = 10;
distance_threshold = 3;%稳定阀值
[goal_point1, goal_point2, goal_point3] = Planning_point(catch_ball_center, catch_ball_radius, num_points, angle_threshold, distance_threshold, shp1, shp2, shp3);
%% 力矩规划
Planning_target_torque(goal_point1, goal_point2, goal_point3, catch_ball_center, weight)

%% RRT规划
q_0=zeros(1,4);
q_start=[0,10,0,0];%103.74, 0, -13.57

% 定义末端执行器的工作空间范围和障碍物
workspace_limits = [-150, 150; -150, 150; -90, 0;]; % 全局工作空间范围
ball_center = [90, 0, -35]; % 障碍物中心
ball_radius = 5; % 障碍物半径

% 定义起始和目标点
start_point1 = FK_end_point(Link_1,q_start);
% goal_point1 = [40	0	-60];
start_point2 = FK_end_point(Link_2,q_start);
% goal_point2 = [-21.2168363247897	33.9093771155868	-60];
start_point3 = FK_end_point(Link_3,q_start);
% goal_point3 = [-17.4922928183542	-35.9724852068488	-60];
% [path1,explored_points1] = RRT(shp1,workspace_limits,start_point1,goal_point1,ball_center,ball_radius);
% [path2,explored_points2] = RRT(shp2,workspace_limits,start_point2,goal_point2,ball_center,ball_radius);
% [path3,explored_points3] = RRT(shp3,workspace_limits,start_point3,goal_point3,ball_center,ball_radius);

[path1, explored_points1] = RRT_star(shp1, workspace_limits, start_point1, goal_point1, ball_center, ball_radius);
[path2, explored_points2] = RRT_star(shp2, workspace_limits, start_point2, goal_point2, ball_center, ball_radius);
[path3, explored_points3] = RRT_star(shp3, workspace_limits, start_point3, goal_point3, ball_center, ball_radius);




%% 显示路径
view(-21,12);
axis([-150,150,-150,150,-150,50]);hold on;
% 平滑路径
smoothed_path1 = smooth_end_effector_path(path1);
smoothed_path2 = smooth_end_effector_path(path2);
smoothed_path3 = smooth_end_effector_path(path3);

% 显示所有遍历到的点
plot3(explored_points1(:, 1), explored_points1(:, 2), explored_points1(:, 3), 'r.', 'MarkerSize', 1.5, 'MarkerFaceColor', 'k');
plot3(explored_points2(:, 1), explored_points2(:, 2), explored_points2(:, 3), 'g.', 'MarkerSize', 1.5, 'MarkerFaceColor', 'k');
plot3(explored_points3(:, 1), explored_points3(:, 2), explored_points3(:, 3), 'b.', 'MarkerSize', 1.5, 'MarkerFaceColor', 'k');
% 绘制末端轨迹
plot3(smoothed_path1(:, 1), smoothed_path1(:, 2), smoothed_path1(:, 3), 'r-', 'LineWidth', 1);
plot3(smoothed_path2(:, 1), smoothed_path2(:, 2), smoothed_path2(:, 3), 'r-', 'LineWidth', 1);
plot3(smoothed_path3(:, 1), smoothed_path3(:, 2), smoothed_path3(:, 3), 'r-', 'LineWidth', 1);
% 标记起点和终点
plot3(smoothed_path1(1, 1), smoothed_path1(1, 2), smoothed_path1(1, 3), 'go', 'MarkerSize', 3, 'MarkerFaceColor', 'g'); % 起点
plot3(smoothed_path2(1, 1), smoothed_path2(1, 2), smoothed_path2(1, 3), 'go', 'MarkerSize', 3, 'MarkerFaceColor', 'g'); % 起点
plot3(smoothed_path3(1, 1), smoothed_path3(1, 2), smoothed_path3(1, 3), 'go', 'MarkerSize', 3, 'MarkerFaceColor', 'g'); % 起点
plot3(smoothed_path1(end, 1), smoothed_path1(end, 2), smoothed_path1(end, 3), 'bo', 'MarkerSize', 3, 'MarkerFaceColor', 'b'); % 终点
plot3(smoothed_path2(end, 1), smoothed_path2(end, 2), smoothed_path2(end, 3), 'bo', 'MarkerSize', 3, 'MarkerFaceColor', 'b'); % 终点
plot3(smoothed_path3(end, 1), smoothed_path3(end, 2), smoothed_path3(end, 3), 'bo', 'MarkerSize', 3, 'MarkerFaceColor', 'b'); % 终点

DHFk_hand(q_start,q_start,q_start,true);
DrawSphere(ball_center,ball_radius,0);
DrawSphere(catch_ball_center,catch_ball_radius,0);
% plot(shp1)
% plot(shp2)
% plot(shp3)
xlabel('X');
ylabel('Y');
zlabel('Z');
% legend('End Effector Trajectory', 'Explored Points', 'Start', 'Goal');
title('End Effector Trajectory');
grid on;
axis equal;

%% 逆运动学转换
done_1 = false;
done_2 = false;
done_3 = false;
num=0;
load('J_matrix.mat', 'J_matrix');
global mf;
mf = matlabFunction(J_matrix);
% 通过逆运动学将轨迹转换为关节空间
q_1=[0,q_start];
q_2=[120,q_start];
q_3=[-120,q_start];
joint_angles1 = [];
joint_angles2 = [];
joint_angles3 = [];

max_size = max([size(path1, 1), size(path2, 1), size(path3, 1)]);
for i = 1:max_size
    if i<=size(path1,1)
        done_1 = false;%re_init
        num = 0;%re_init
        target_pos = path1(i, :);
        while ~done_1
            Link_1=DHfk_finger(Link_1,q_1(2:end),false);
            [q_1, done_1] = IK_Sol(Link_1, q_1, target_pos, learning_rate);
            if done_1
                joint_angles1 = [joint_angles1; q_1];
                DHfk_finger(Link_1,q_1(2:end),true);
            end
            num=num+1;
            if num > 30
                %                 fprintf('%d逆解失败\n', i);
                break;
            end
        end
    end
    if i<=size(path2,1)
        done_2 = false;%re_init
        num = 0;%re_init
        target_pos = path2(i, :);
        while ~done_2
            Link_2=DHfk_finger(Link_2,q_2(2:end),false);
            [q_2, done_2] = IK_Sol(Link_2, q_2, target_pos, learning_rate);
            if done_2
                joint_angles2 = [joint_angles2; q_2];
                DHfk_finger(Link_2,q_2(2:end),true);
            end
            num=num+1;
            if num > 30
                %                 fprintf('%d逆解失败\n', i);
                break;
            end
        end
    end
    if i<=size(path3,1)
        done_3 = false;%re_init
        num = 0;%re_init
        target_pos = path3(i, :);
        while ~done_3
            Link_3=DHfk_finger(Link_3,q_3(2:end),false);
            [q_3, done_3] = IK_Sol(Link_3, q_3, target_pos, learning_rate);
            if done_3
                joint_angles3 = [joint_angles3; q_3];
                DHfk_finger(Link_3,q_3(2:end),true);
            end
            num=num+1;
            if num > 30
                %                 fprintf('%d逆解失败\n', i);
                break;
            end
        end
    end
    drawnow;
end

%% 绘制机器人
% for i = 1:size(joint_angles1, 1)
%     DHfk_finger(Link_1,joint_angles1(i,2:end),true);
% end
% for i = 1:size(joint_angles2, 1)
%     DHfk_finger(Link_2,joint_angles2(i,2:end),true);
% end
% for i = 1:size(joint_angles3, 1)
%     DHfk_finger(Link_3,joint_angles3(i,2:end),true);
% end

%% 局部函数
function end_point = FK_end_point(Link,q)
global FK;
q = deg2rad(q);
end_point= FK(Link(1).th,q(1),q(2),q(3),q(4));
end_point = end_point(1:3,4)';
end

