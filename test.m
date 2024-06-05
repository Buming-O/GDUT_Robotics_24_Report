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
r=40;
dt1 = alphaShape(workspace_points1,r);
dt2 = alphaShape(workspace_points2,r);
dt3 = alphaShape(workspace_points3,r);
% 绘制凸包
figure;
hold on;
% trisurf(convexHull(dt1), dt1.Points(:,1), dt1.Points(:,2), dt1.Points(:,3), ...
% 'EdgeColor', 'r', 'FaceColor', [0.8, 0.8, 0.8], 'LineWidth', 2);
% trisurf(convexHull(dt2), dt2.Points(:,1), dt2.Points(:,2), dt2.Points(:,3), ...
% 'EdgeColor', 'g', 'FaceColor', [0.8, 0.8, 0.8], 'LineWidth', 2);
% trisurf(convexHull(dt3), dt3.Points(:,1), dt3.Points(:,2), dt3.Points(:,3), ...
% 'EdgeColor', 'b', 'FaceColor', [0.8, 0.8, 0.8], 'LineWidth', 2);
disp(dt1.Alpha)
plot(dt1);
% plot(dt2);
% plot(dt3);
scatter3(workspace_points1(:,1), workspace_points1(:,2), workspace_points1(:,3), 'r.');
scatter3(workspace_points2(:,1), workspace_points2(:,2), workspace_points2(:,3), 'g.');
scatter3(workspace_points3(:,1), workspace_points3(:,2), workspace_points3(:,3), 'b.');
% 添加坐标轴标签和标题
xlabel('X');
ylabel('Y');
zlabel('Z');
title('凸包和检查点');

% 其他可视化设置
grid on;
axis equal;
view(3); % 3D 视图