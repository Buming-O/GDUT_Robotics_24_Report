clc;
clear all;
close all;
num=0;
learning_rate = 0.9;
ToDeg = 180/pi;
ToRad = pi/180;
% ������ָ�Ĺ����ռ��
load('workspace_points1.mat', 'workspace_points1');
load('workspace_points2.mat', 'workspace_points2');
load('workspace_points3.mat', 'workspace_points3');
% ʹ��Delaunay�����ʷּ����Ƿ��ڶ��������
% dt1 = delaunayTriangulation(workspace_points1);
% dt2 = delaunayTriangulation(workspace_points2);
% dt3 = delaunayTriangulation(workspace_points3);
% ʹ��alphaShape�����Ƿ��ڶ��������
shp1 = alphaShape(workspace_points1,60);
shp2 = alphaShape(workspace_points2,60);
shp3 = alphaShape(workspace_points3,60);
% ������İ뾶������λ��
ball_radius = 40;
ball_center = [10, 10, -70];

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

%% ������ʼ���ؽ�λ��
q_1=[0,0,0,0,0];
q_2=[120,0,0,0,0];
q_3=[-120,0,0,0,0];

%% �滮Ŀ��ĩ��λ��
% ����ı��������ɺ�ѡ�Ӵ���
num_points = 50; % ��ѡ�Ӵ��������
theta = linspace(0, 2*pi, num_points);
phi = linspace(pi/2, pi, num_points);
[Theta, Phi] = meshgrid(theta, phi);
X = ball_radius * sin(Phi) .* cos(Theta) + ball_center(1);
Y = ball_radius * sin(Phi) .* sin(Theta) + ball_center(2);
Z = ball_radius * cos(Phi) + ball_center(3);
candidate_points = [X(:), Y(:), Z(:)];
% ���ֺ�ѡ�Ӵ��㵽��������
theta_deg = rad2deg(Theta(:));
angle_threshold = 20;
points_sector1 = candidate_points(theta_deg >= 0-angle_threshold & theta_deg < 0+angle_threshold, :);
points_sector2 = candidate_points(theta_deg >= 120-angle_threshold & theta_deg < 120+angle_threshold, :);
points_sector3 = candidate_points(theta_deg >= 240-angle_threshold & theta_deg < 240+angle_threshold, :);

% ��ʼ���ɴ�ĽӴ���,���ÿ�����޵ĺ�ѡ�Ӵ����Ƿ��ڶ�Ӧ��ָ�Ĺ����ռ���
% reachable_points1 = find_reachable_points_dt(points_sector1, dt1);
% reachable_points2 = find_reachable_points_dt(points_sector2, dt2);
% reachable_points3 = find_reachable_points_dt(points_sector3, dt3);
reachable_points1 = find_reachable_points_shp(points_sector1, shp1);
reachable_points2 = find_reachable_points_shp(points_sector2, shp2);
reachable_points3 = find_reachable_points_shp(points_sector3, shp3);

% ȷ���γ��ȶ���ץȡ������ %����������������������֮���ŷ�Ͼ����ж�
distance_threshold = 3;%�ȶ���ֵ
if ~isempty(reachable_points1) && ~isempty(reachable_points2) && ~isempty(reachable_points3)
    % ��ʼ����С����Ͷ�Ӧ�ĽӴ���
    min_distance = inf;
    best_cp1 = [];
    best_cp2 = [];
    best_cp3 = [];
    
    % �������п��ܵĽӴ������
    for i = 1:size(reachable_points1, 1)
        for j = 1:size(reachable_points2, 1)
            for k = 1:size(reachable_points3, 1)
                cp1 = reachable_points1(i, :);
                cp2 = reachable_points2(j, :);
                cp3 = reachable_points3(k, :);

                % ����������Ƿ���
                if ~iscollinear(cp1, cp2, cp3)
                    % ����Ӵ��������
                    centroid = (cp1 + cp2 + cp3) / 3;

                    % ��������������֮��ľ���
                    distance = norm(centroid - ball_center);

                    % ������С�������ѽӴ���
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
    
    % ��ʾ���
    disp('ѡ��ĽӴ���:');
    disp(best_cp1);
    disp(best_cp2);
    disp(best_cp3);
    disp('�Ӵ�������:');
    disp((best_cp1 + best_cp2 + best_cp3) / 3);
    disp('����������֮��ľ���:');
    disp(min_distance);

    if min_distance < distance_threshold
        disp('�γ��ȶ���ץȡ������');
    else
        disp('ץȡ�����β��ȶ�');
    end
else
    disp('û���㹻�Ŀɴ�Ӵ������γ��ȶ���ץȡ');
end

figure;
DrawSphere(ball_center,ball_radius,0);
hold on;
plot3(candidate_points(:,1),candidate_points(:,2),candidate_points(:,3),'k.','MarkerSize',1);
plot3(reachable_points1(:, 1), reachable_points1(:, 2), reachable_points1(:, 3), 'r.', 'MarkerSize', 5, 'LineWidth', 1);
plot3(reachable_points2(:, 1), reachable_points2(:, 2), reachable_points2(:, 3), 'g.', 'MarkerSize', 5, 'LineWidth', 1);
plot3(reachable_points3(:, 1), reachable_points3(:, 2), reachable_points3(:, 3), 'b.', 'MarkerSize', 5, 'LineWidth', 1);
plot3(best_cp1(1), best_cp1(2), best_cp1(3), 'ro', 'MarkerSize', 10, 'LineWidth', 4);
plot3(best_cp2(1), best_cp2(2), best_cp2(3), 'go', 'MarkerSize', 10, 'LineWidth', 4);
plot3(best_cp3(1), best_cp3(2), best_cp3(3), 'bo', 'MarkerSize', 10, 'LineWidth', 4);

% ����ԭʼ�����ռ�㣨��ѡ��
% scatter3(workspace_points1(:,1), workspace_points1(:,2), workspace_points1(:,3), 'r.');
% scatter3(workspace_points2(:,1), workspace_points2(:,2), workspace_points2(:,3), 'g.');
% scatter3(workspace_points3(:,1), workspace_points3(:,2), workspace_points3(:,3), 'b.');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('��ĽӴ���');
grid on;
% hold off;
axis equal;
pause; 
T1_pos=[best_cp1(1), best_cp1(2), best_cp1(3)];
T2_pos=[best_cp2(1), best_cp2(2), best_cp2(3)];
T3_pos=[best_cp3(1), best_cp3(2), best_cp3(3)];
%% ���ƻ�е�۳�ʼλ�˼�ĩ����̬
DHFk_hand(q_1(2:end),q_2(2:end),q_3(2:end),true);
% plot3(T1_pos(1),T1_pos(2),T1_pos(3),'ro', 'MarkerSize', 5, 'LineWidth', 3); hold on;
% plot3(T2_pos(1),T2_pos(2),T2_pos(3),'go', 'MarkerSize', 5, 'LineWidth', 3); hold on;
% plot3(T3_pos(1),T3_pos(2),T3_pos(3),'bo', 'MarkerSize', 5, 'LineWidth', 3); hold on;
% DrawSphere(ball_center,ball_radius,0);
drawnow;
view(-21,12);
title('��е�۳�ʼλ�˼�ĩ�˵�');
pause; 
cla;hold on;
%% ������˶�ѧ
done_1 = false;
done_2 = false;
done_3 = false;
load('J_matrix.mat', 'J_matrix');
global mf;
mf = matlabFunction(J_matrix);
% profile on; % �������ܷ���
tic
while ~(done_1 && done_2 && done_3)
    DHFk_hand(q_1(2:end),q_2(2:end),q_3(2:end),false); % FK���㲢���ƻ�����
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
% profile off; % �ر����ܷ���
% profile viewer; % ��ʾ���ܷ������
disp(['��������: ', num2str(num)]);
disp(['Done_1: ', num2str(done_1),'  |  Done_2: ', num2str(done_2),'  |  Done_3: ', num2str(done_3)]);

%% �ٴλ��ƻ����˱���ͼ��
plot3(T1_pos(1),T1_pos(2),T1_pos(3),'ro', 'MarkerSize', 5, 'LineWidth', 3); hold on;
plot3(T2_pos(1),T2_pos(2),T2_pos(3),'go', 'MarkerSize', 5, 'LineWidth', 3); hold on;
plot3(T3_pos(1),T3_pos(2),T3_pos(3),'bo', 'MarkerSize', 5, 'LineWidth', 3); hold on;
DrawSphere(ball_center,ball_radius,0);
DHFk_hand(q_1(2:end),q_2(2:end),q_3(2:end),true);
title('��е��ĩ����̬');
drawnow;

%%

% ��������������������Ƿ���
function flag = iscollinear(p1, p2, p3)
    v1 = p2 - p1;
    v2 = p3 - p1;
    flag = norm(cross(v1, v2)) < 1e-6;
end

function is_in_workspace = is_point_in_workspace(dt, point)
    [~, volume] = convexHull(dt);
    % �����Ƿ��ڹ����ռ��͹����
    [~, point_inside] = tsearchn(dt.Points, dt.ConnectivityList, point);
    is_in_workspace = ~isnan(point_inside);
end

function reachable_points = find_reachable_points_dt(points_sector, dt)
    reachable_points = [];
    for i = 1:size(points_sector, 1)
        contact_point = points_sector(i, :);
        if is_point_in_workspace(dt, contact_point)
            reachable_points = [reachable_points; contact_point];
        end
    end
end

function reachable_points = find_reachable_points_shp(points_sector, shp)
    % ʹ��alphaShape�����Ƿ��ڶ��������
    reachable_points = [];
    for i = 1:size(points_sector, 1)
        contact_point = points_sector(i, :);
        if inShape(shp, contact_point)
            reachable_points = [reachable_points; contact_point];
        end
    end
end