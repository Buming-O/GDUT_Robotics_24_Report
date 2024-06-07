clc;
clear all;
close all;
num=0;
view(134,12);
% 定义物体参数
mass = 0.5; % 物体质量，单位kg
g = 9.81; % 重力加速度，单位m/s^2
weight = mass * g; % 物体的重量

learning_rate = 0.9;
ToDeg = 180/pi;
ToRad = pi/180;
% 加载手指的工作空间点
load('workspace_points1.mat', 'workspace_points1');
load('workspace_points2.mat', 'workspace_points2');
load('workspace_points3.mat', 'workspace_points3');
% 使用alphaShape检查点是否在多边形体内
shp1 = alphaShape(workspace_points1,60);
shp2 = alphaShape(workspace_points2,60);
shp3 = alphaShape(workspace_points3,60);
% 定义正方体的边长和中心位置
cube_side = 30;
cube_center = [0, 0, -60];
cube_color = [0, 0.5, 0]; % 绿色
half_length = cube_side / 2;

global Link_1
global Link_2
global Link_3
Build;
global th2_min th2_max th3_min th3_max th4_min th4_max th5_min th5_max;
th2_min=-90; th2_max=90;   % Range for th2
th3_min=0; th3_max=60;  % Range for th3
th4_min=0; th4_max=60;   % Range for th4
th5_min=0; th5_max=80;     % Range for th5

%% 输入起始六关节位置
q_1=[0,0,0,0,0];
q_2=[120,0,0,0,0];
q_3=[-120,0,0,0,0];
%% 规划目标末端位置
num_points = 20; % 候选接触点的数量
% 生成正方体的候选接触点
inner_ratio = 0.5; % 中间部分占整个边长的比例
% 中间部分的范围
inner_half_length = inner_ratio * half_length;
% 前后左右表面
[X, Y] = meshgrid(linspace(-inner_half_length, inner_half_length, num_points));
Z1 = half_length * ones(num_points, num_points);
Z2 = -half_length * ones(num_points, num_points);
% 前后表面
points_front = [X(:), Z2(:), Y(:)] + cube_center;
points_back = [X(:), Z1(:), Y(:)] + cube_center;
% 左右表面
points_left = [Z2(:), X(:), Y(:)] + cube_center;
points_right = [Z1(:), X(:), Y(:)] + cube_center;

% 合并所有表面点
candidate_points = [points_front; points_back; points_left; points_right];

% 计算候选点关于正方体中心绕z轴的角度 theta
theta = atan2(candidate_points(:, 2) - cube_center(2), candidate_points(:, 1) - cube_center(1));

% 将角度转换为0到2π的范围
theta = mod(theta, 2 * pi);

% 角度转换为度数
theta_deg = rad2deg(theta);
% 通过如下的方法将它们分成三个象限
angle_threshold = 10;
points_sector1 = candidate_points(theta_deg >= 0 - angle_threshold & theta_deg < 0 + angle_threshold, :);
points_sector2 = candidate_points(theta_deg >= 120 - angle_threshold & theta_deg < 120 + angle_threshold, :);
points_sector3 = candidate_points(theta_deg >= 240 - angle_threshold & theta_deg < 240 + angle_threshold, :);


% 初始化可达的接触点,检查每个象限的候选接触点是否在对应手指的工作空间内
reachable_points1 = find_reachable_points_shp(points_sector1, shp1);
reachable_points2 = find_reachable_points_shp(points_sector2, shp2);
reachable_points3 = find_reachable_points_shp(points_sector3, shp3);

% 确保形成稳定的抓取三角形 %用物体重心与三角形质心之间的欧氏距离判断
distance_threshold = 3; % 稳定阀值
if ~isempty(reachable_points1) && ~isempty(reachable_points2) && ~isempty(reachable_points3)
    % 初始化最小距离和对应的接触点
    min_distance = inf;
    best_cp1 = [];
    best_cp2 = [];
    best_cp3 = [];

    % 遍历所有可能的接触点组合
    for i = 1:size(reachable_points1, 1)
        for j = 1:size(reachable_points2, 1)
            for k = 1:size(reachable_points3, 1)
                cp1 = reachable_points1(i, :);
                cp2 = reachable_points2(j, :);
                cp3 = reachable_points3(k, :);

                % 检查三个点是否共线
                if ~iscollinear(cp1, cp2, cp3)
                    % 计算接触点的质心
                    centroid = (cp1 + cp2 + cp3) / 3;

                    % 计算质心与球心之间的距离
                    distance = norm(centroid - cube_center);

                    % 更新最小距离和最佳接触点
                    if distance < min_distance
                        min_distance = distance;
                        best_cp1 = cp1;
                        best_cp2 = cp2;
                        best_cp3 = cp3;
                    end
                end
            end
        end
    end
    % 显示结果
    if min_distance < distance_threshold
        fprintf('质心与正方体中心之间的距离:%.4f, 形成稳定的抓取三角形\n', min_distance);
    else
        disp('抓取三角形不稳定');
    end
else
    disp('没有足够的可达接触点来形成稳定的抓取');
end
T1_pos=[best_cp1(1), best_cp1(2), best_cp1(3)];
T2_pos=[best_cp2(1), best_cp2(2), best_cp2(3)];
T3_pos=[best_cp3(1), best_cp3(2), best_cp3(3)];
%% 计算目标力矩
Planning_target_torque(T1_pos, T2_pos, T3_pos, cube_center, weight)
%% 画正方体的接触点
hold on;
axis([-200,200,-200,200,-200,200]);
DHFk_hand(q_1(2:end),q_2(2:end),q_3(2:end),true);
DrawCube(cube_center, cube_side,cube_color);
plot3(candidate_points(:,1),candidate_points(:,2),candidate_points(:,3),'k.','MarkerSize',1);
plot3(points_sector1(:,1), points_sector1(:,2), points_sector1(:,3), 'r.', 'MarkerSize', 1);
plot3(points_sector2(:,1), points_sector2(:,2), points_sector2(:,3), 'g.', 'MarkerSize', 1);
plot3(points_sector3(:,1), points_sector3(:,2), points_sector3(:,3), 'b.', 'MarkerSize', 1);
plot3(reachable_points1(:, 1), reachable_points1(:, 2), reachable_points1(:, 3), 'r.', 'MarkerSize', 5, 'LineWidth', 1);
plot3(reachable_points2(:, 1), reachable_points2(:, 2), reachable_points2(:, 3), 'g.', 'MarkerSize', 5, 'LineWidth', 1);
plot3(reachable_points3(:, 1), reachable_points3(:, 2), reachable_points3(:, 3), 'b.', 'MarkerSize', 5, 'LineWidth', 1);
plot3(best_cp1(1), best_cp1(2), best_cp1(3), 'ro', 'MarkerSize', 10, 'LineWidth', 4);
plot3(best_cp2(1), best_cp2(2), best_cp2(3), 'go', 'MarkerSize', 10, 'LineWidth', 4);
plot3(best_cp3(1), best_cp3(2), best_cp3(3), 'bo', 'MarkerSize', 10, 'LineWidth', 4);
% hold off;
pause;
cla;hold on;
%% 求解逆运动学
done_1 = false;
done_2 = false;
done_3 = false;
load('J_matrix.mat', 'J_matrix');
global mf;
mf = matlabFunction(J_matrix);
% profile on; % 开启性能分析
tic
while ~(done_1 && done_2 && done_3)
    DHFk_hand(q_1(2:end),q_2(2:end),q_3(2:end),false); % FK计算并绘制机器人
    if ~done_1
        [q_1, done_1] = IK_Sol(Link_1, q_1, T1_pos, learning_rate);
    end
    if ~done_2
        [q_2, done_2] = IK_Sol(Link_2, q_2, T2_pos, learning_rate);
    end
    if ~done_3
        [q_3, done_3] = IK_Sol(Link_3, q_3, T3_pos, learning_rate);
    end
    num=num+1;
    if(num>100)
        disp("逆解失败");
        break;
    end
end
toc
% profile off; % 关闭性能分析
% profile viewer; % 显示性能分析结果
disp(['迭代次数: ', num2str(num)]);
disp(['Done_1: ', num2str(done_1),'  |  Done_2: ', num2str(done_2),'  |  Done_3: ', num2str(done_3)]);

%% 再次绘制机器人保持图像
plot3(T1_pos(1),T1_pos(2),T1_pos(3),'ro', 'MarkerSize', 5, 'LineWidth', 3); hold on;
plot3(T2_pos(1),T2_pos(2),T2_pos(3),'go', 'MarkerSize', 5, 'LineWidth', 3); hold on;
plot3(T3_pos(1),T3_pos(2),T3_pos(3),'bo', 'MarkerSize', 5, 'LineWidth', 3); hold on;
DrawCube(cube_center, cube_side,cube_color);
DHFk_hand(q_1(2:end),q_2(2:end),q_3(2:end),true);
title('机械臂末端姿态');
drawnow;

%% 
% 检查三个点是否共线
function result = iscollinear(p1, p2, p3)
    result = norm(cross(p2 - p1, p3 - p1)) < 1e-10;
end
function reachable_points = find_reachable_points_shp(points_sector, shp)
    % 使用alphaShape检查点是否在多边形体内
    reachable_points = [];
    for i = 1:size(points_sector, 1)
        contact_point = points_sector(i, :);
        if inShape(shp, contact_point)
            reachable_points = [reachable_points; contact_point];
        end
    end
end

function cost = objective_function(f_dir, cp1, cp2, cp3, cube_center,weight)
    f1_dir = f_dir(1:3);
    f2_dir = f_dir(4:6);
    f3_dir = f_dir(7:9);

    f1_dir = f1_dir / norm(f1_dir);
    f2_dir = f2_dir / norm(f2_dir);
    f3_dir = f3_dir / norm(f3_dir);


    force_magnitude =  weight / 3; % 假设每个接触力的大小相等

    f1 = force_magnitude * f1_dir;
    f2 = force_magnitude * f2_dir;
    f3 = force_magnitude * f3_dir;

    r1 = cp1 - cube_center;
    r2 = cp2 - cube_center;
    r3 = cp3 - cube_center;

    total_torque = cross(r1, f1) + cross(r2, f2) + cross(r3, f3);
    total_force = f1 + f2 + f3 - [0,0, weight];

    cost = norm(total_torque) + 10*norm(total_force); % 最小化力和力矩的不平衡量
end
