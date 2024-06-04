clc;
clear all;
close all;
num=0;
learning_rate = 0.9;
ToDeg = 180/pi;
ToRad = pi/180;
% 加载手指的工作空间点
load('workspace_points1.mat', 'workspace_points1');
load('workspace_points2.mat', 'workspace_points2');
load('workspace_points3.mat', 'workspace_points3');
% 使用Delaunay三角剖分检查点是否在多边形体内
dt1 = delaunayTriangulation(workspace_points1);
dt2 = delaunayTriangulation(workspace_points2);
dt3 = delaunayTriangulation(workspace_points3);
% 定义球的半径和中心位置
ball_radius = 40;
ball_center = [0, 0, -85];

global Link_1
global Link_2
global Link_3
Build;
% load('Link_1.mat','Link_1');load('Link_2.mat','Link_2');load('Link_3.mat','Link_3');
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
% 在球的表面上生成候选接触点
num_points = 50; % 候选接触点的数量
theta = linspace(0, 2*pi, num_points);
phi = linspace(pi/2, pi, num_points);
[Theta, Phi] = meshgrid(theta, phi);
X = ball_radius * sin(Phi) .* cos(Theta) + ball_center(1);
Y = ball_radius * sin(Phi) .* sin(Theta) + ball_center(2);
Z = ball_radius * cos(Phi) + ball_center(3);
candidate_points = [X(:), Y(:), Z(:)];
% 划分候选接触点到三个象限
theta_deg = rad2deg(Theta(:));
points_sector1 = candidate_points(theta_deg >= -20 & theta_deg < 20, :);
points_sector2 = candidate_points(theta_deg >= 110 & theta_deg < 130, :);
points_sector3 = candidate_points(theta_deg >= 230 & theta_deg < 250, :);

% 初始化可达的接触点
reachable_points1 = [];
reachable_points2 = [];
reachable_points3 = [];

% 检查每个象限的候选接触点是否在对应手指的工作空间内
for i = 1:size(points_sector1, 1)
    contact_point = points_sector1(i, :);
    is_reachable1 = is_point_in_workspace(dt1, contact_point);% 判断是否在手指的工作空间内
    if is_reachable1
        reachable_points1 = [reachable_points1; contact_point];
    end
end

for i = 1:size(points_sector2, 1)
    contact_point = points_sector2(i, :);
    is_reachable2 = is_point_in_workspace(dt2, contact_point);
    if is_reachable2
        reachable_points2 = [reachable_points2; contact_point];
    end
end

for i = 1:size(points_sector3, 1)
    contact_point = points_sector3(i, :);
    is_reachable3 = is_point_in_workspace(dt3, contact_point);
    if is_reachable3
        reachable_points3 = [reachable_points3; contact_point];
    end
end

% 确保形成稳定的抓取三角形 %可以用物体重心与三角形质心之间的欧氏距离判断
if size(reachable_points1, 1) >= 1 && size(reachable_points2, 1) >= 1 && size(reachable_points3, 1) >= 1
    % 选择每个象限中的第一个点
    cp1 = reachable_points1(1, :);
    cp2 = reachable_points2(1, :);
    cp3 = reachable_points3(1, :);  
%     while iscollinear(cp1, cp2, cp3)% 检查三个点是否共线
%         % 如果共线，则随机选择下一个点
%         cp1 = reachable_points1(randi(size(reachable_points1, 1)), :);
%         cp2 = reachable_points2(randi(size(reachable_points2, 1)), :);
%         cp3 = reachable_points3(randi(size(reachable_points3, 1)), :);
%     end
    disp('选择的接触点:');
    disp(cp1);
    disp(cp2);
    disp(cp3);
else
    disp('没有足够的可达接触点来形成稳定的抓取');
end

figure;
DrawSphere(ball_center,ball_radius,0);
hold on;
plot3(candidate_points(:,1),candidate_points(:,2),candidate_points(:,3),'k.','MarkerSize',1);
plot3(reachable_points1(:, 1), reachable_points1(:, 2), reachable_points1(:, 3), 'r.', 'MarkerSize', 5, 'LineWidth', 1);
plot3(reachable_points2(:, 1), reachable_points2(:, 2), reachable_points2(:, 3), 'g.', 'MarkerSize', 5, 'LineWidth', 1);
plot3(reachable_points3(:, 1), reachable_points3(:, 2), reachable_points3(:, 3), 'b.', 'MarkerSize', 5, 'LineWidth', 1);
plot3(cp1(1), cp1(2), cp1(3), 'ro', 'MarkerSize', 10, 'LineWidth', 4);
plot3(cp2(1), cp2(2), cp2(3), 'go', 'MarkerSize', 10, 'LineWidth', 4);
plot3(cp3(1), cp3(2), cp3(3), 'bo', 'MarkerSize', 10, 'LineWidth', 4);

% 绘制原始工作空间点（可选）
% scatter3(workspace_points1(:,1), workspace_points1(:,2), workspace_points1(:,3), 'r.');
% scatter3(workspace_points2(:,1), workspace_points2(:,2), workspace_points2(:,3), 'g.');
% scatter3(workspace_points3(:,1), workspace_points3(:,2), workspace_points3(:,3), 'b.');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('球的接触点');
grid on;
% hold off;
axis equal;
pause; 
T1_pos=[cp1(1), cp1(2), cp1(3)];
T2_pos=[cp2(1), cp2(2), cp2(3)];
T3_pos=[cp3(1), cp3(2), cp3(3)];
%% 绘制机械臂初始位姿及末端姿态
DHFk_hand(q_1(2:end),q_2(2:end),q_3(2:end),true);
plot3(T1_pos(1),T1_pos(2),T1_pos(3),'ro', 'MarkerSize', 5, 'LineWidth', 3); hold on;
plot3(T2_pos(1),T2_pos(2),T2_pos(3),'go', 'MarkerSize', 5, 'LineWidth', 3); hold on;
plot3(T3_pos(1),T3_pos(2),T3_pos(3),'bo', 'MarkerSize', 5, 'LineWidth', 3); hold on;
DrawSphere(ball_center,ball_radius,0);
drawnow;
view(-21,12);
title('机械臂初始位姿及末端点');
pause; 
cla;hold on;
%% 求解逆运动学
done_1 = false;
done_2 = false;
done_3 = false;
load('test_j_matrix.mat', 'test_j_matrix');
global mf;
mf = matlabFunction(test_j_matrix);
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
    if(num>1000)
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
DrawSphere(ball_center,ball_radius,0);
DHFk_hand(q_1(2:end),q_2(2:end),q_3(2:end),true);
title('机械臂末端姿态');
drawnow;

%%

% 辅助函数：检查三个点是否共线
function flag = iscollinear(p1, p2, p3)
    v1 = p2 - p1;
    v2 = p3 - p1;
    flag = norm(cross(v1, v2)) < 1e-6;
end

function is_in_workspace = is_point_in_workspace(dt, point)
    [~, volume] = convexHull(dt);
    % 检查点是否在工作空间的凸包内
    [~, point_inside] = tsearchn(dt.Points, dt.ConnectivityList, point);
    is_in_workspace = ~isnan(point_inside);
end



