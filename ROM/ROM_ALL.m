close all;clear;
global Link_1
global Link_2
global Link_3
Build;
% Number of random samples
th_ranges = [
    -90, 90;   % Range for th1
    0, 60;  % Range for th2
    0, 60;   % Range for th3
    0, 80     % Range for th4
    ];
%%
grid on;hold on;

% ball_radius = 20;
% ball_center = [0,0,-50];
% DrawSphere(ball_center,ball_radius,0);

% 工作空间采样点
workspace_points1 = [];
workspace_points2 = [];
workspace_points3 = [];
num_samples = 5000;

for sample = 1:num_samples
    th1 = th_ranges(1,1) + (th_ranges(1,2) - th_ranges(1,1)) * rand* ToRad;
    th2 = th_ranges(2,1) + (th_ranges(2,2) - th_ranges(2,1)) * rand* ToRad;
    th3 = th_ranges(3,1) + (th_ranges(3,2) - th_ranges(3,1)) * rand* ToRad;
    th4 = th_ranges(4,1) + (th_ranges(4,2) - th_ranges(4,1)) * rand* ToRad;
    
    Link_1(2).th = th1 ;
    Link_1(3).th = th2 ;
    Link_1(4).th = th3 ;
    Link_1(5).th = th4 ;
    
    Link_2(2).th = th1 ;
    Link_2(3).th = th2 ;
    Link_2(4).th = th3 ;
    Link_2(5).th = th4 ;
    
    Link_3(2).th = th1 ;
    Link_3(3).th = th2 ;
    Link_3(4).th = th3 ;
    Link_3(5).th = th4 ;
    
    for i = 1:5
        Link_1 = Matrix_dh(Link_1,i);
        Link_2 = Matrix_dh(Link_2,i);
        Link_3 = Matrix_dh(Link_3,i);
    end
    for i = 2:5
        Link_1(i).A = Link_1(i-1).A * Link_1(i).A;
        Link_1(i).p = Link_1(i).A(:, 4);

        Link_2(i).A = Link_2(i-1).A * Link_2(i).A;
        Link_2(i).p = Link_2(i).A(:, 4);

        Link_3(i).A = Link_3(i-1).A * Link_3(i).A;
        Link_3(i).p = Link_3(i).A(:, 4);
    end
    % 保存工作空间点
    workspace_points1 = [workspace_points1; Link_1(5).p(1), Link_1(5).p(2), Link_1(5).p(3)];
    workspace_points2 = [workspace_points2; Link_2(5).p(1), Link_2(5).p(2), Link_2(5).p(3)];
    workspace_points3 = [workspace_points3; Link_3(5).p(1), Link_3(5).p(2), Link_3(5).p(3)];
end
scatter3(workspace_points1(:,1), workspace_points1(:,2), workspace_points1(:,3), 'r.');
scatter3(workspace_points2(:,1), workspace_points2(:,2), workspace_points2(:,3), 'g.');
scatter3(workspace_points3(:,1), workspace_points3(:,2), workspace_points3(:,3), 'b.');
%%
% save('workspace_points1.mat', 'workspace_points1');
% save('workspace_points2.mat', 'workspace_points2');
% save('workspace_points3.mat', 'workspace_points3');

%%
plot3(0, 0, 0, 'g+', 'MarkerSize', 10);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('ROM');
grid on;
hold off;
axis auto;