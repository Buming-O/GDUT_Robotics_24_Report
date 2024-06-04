clear all;
Build;
% qq=[-120,30,60,60,60];
% % qq=[0,0,0,0,0];
% j1=Jacobian4DoF(Link_3,qq)
% j2=Jacobian_finger(qq)
% 
% ToRad = pi / 180;
% load('test_j_matrix.mat', 'test_j_matrix');
% disp(test_j_matrix);

% %           theta      d        a        alpha 
% L(1)=Link([  Link_1(1).th       Link_1(1).dz      Link_1(1).dx      Link_1(1).alf ],'standard');
% L(2)=Link([  Link_1(2).th       Link_1(2).dz      Link_1(2).dx      Link_1(2).alf ],'standard'); L(2).qlim=[-pi,pi];
% L(3)=Link([  Link_1(3).th       Link_1(3).dz      Link_1(3).dx      Link_1(3).alf],'standard');  L(3).qlim=[-pi,pi];
% L(4)=Link([  Link_1(4).th       Link_1(4).dz      Link_1(4).dx      Link_1(4).alf],'standard');  L(4).qlim=[-pi,pi];
% L(5)=Link([  Link_1(5).th       Link_1(5).dz      Link_1(5).dx      Link_1(5).alf],'standard');  L(5).qlim=[-pi,pi]; 
% % 把上述连杆“串起来”
% Scara=SerialLink(L,'name','thumb');
% Scara.teach();
% Scara.jacob0(qq)
% Scara.jacobe(qq)

% pinv(j1)
% pinv(j2)

% 定义球的半径和中心位置
ball_radius = 40;
ball_center = [0, -46, -65];
% 加载手指的工作空间点
load('workspace_points1.mat', 'workspace_points1');
load('workspace_points2.mat', 'workspace_points2');
load('workspace_points3.mat', 'workspace_points3');


% 检查球心是否在每个手指的工作空间内
is_in_workspace1 = is_point_in_workspace(workspace_points1, ball_center);
is_in_workspace2 = is_point_in_workspace(workspace_points2, ball_center);
is_in_workspace3 = is_point_in_workspace(workspace_points3, ball_center);
disp([is_in_workspace1,is_in_workspace2,is_in_workspace3])
% 显示结果
if all(is_in_workspace1) && all(is_in_workspace2) && all(is_in_workspace3)
    disp('球心在所有手指的工作空间内');
else
    disp('球心不在所有手指的工作空间内');
end

% 绘制球和工作空间
figure;
DrawSphere(ball_center,ball_radius,0);
hold on;
plot3(workspace_points1(:, 1), workspace_points1(:, 2), workspace_points1(:, 3), 'ro', 'MarkerSize', 2);
plot3(workspace_points2(:, 1), workspace_points2(:, 2), workspace_points2(:, 3), 'go', 'MarkerSize', 2);
plot3(workspace_points3(:, 1), workspace_points3(:, 2), workspace_points3(:, 3), 'bo', 'MarkerSize', 2);

plot3(ball_center(1), ball_center(2), ball_center(3), 'k*', 'MarkerSize', 10, 'LineWidth', 2);

xlabel('X');
ylabel('Y');
zlabel('Z');
title('球心与工作空间');
grid on;
hold off;
axis equal;

function is_in_workspace = is_point_in_workspace(workspace_points, point)
    % 使用Delaunay三角剖分检查点是否在多边形体内
    dt = delaunayTriangulation(workspace_points);
    [~, volume] = convexHull(dt);
    % 检查点是否在工作空间的凸包内
    [~, point_inside] = tsearchn(dt.Points, dt.ConnectivityList, point);
    is_in_workspace = ~isnan(point_inside);
end