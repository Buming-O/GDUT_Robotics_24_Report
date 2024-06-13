clc;
clear all;
close all;
global Link_1
global Link_2
global Link_3
global Link_Arm
Build;
Build_Puma560;
load('FK_matrix.mat', 'T0end');global FK;
FK = matlabFunction(T0end);

%建立工作空间
load('workspace_points1.mat', 'workspace_points1');
load('workspace_points2.mat', 'workspace_points2');
load('workspace_points3.mat', 'workspace_points3');
alpha_r=30;
shp1 = alphaShape(workspace_points1,alpha_r);
shp2 = alphaShape(workspace_points2,alpha_r);
shp3 = alphaShape(workspace_points3,alpha_r);
global th2_min th2_max th3_min th3_max th4_min th4_max th5_min th5_max;
th2_min=-90; th2_max=90;   % Range for th2
th3_min=0; th3_max=60;  % Range for th3
th4_min=0; th4_max=60;   % Range for th4
th5_min=0; th5_max=80;     % Range for th5

% 定义球的半径和中心位置
catch_ball_radius = 40;
catch_ball_center = [457,102,92];
% 定义物体参数
mass = 0.5; % 物体质量，单位kg
g = 9.81; % 重力加速度，单位m/s^2
weight = mass * g; % 物体的重量

% q_arm = [0,90,0,0,90,180];%457,102,152
q_arm = [0,20,0,0,0,0];%457,102,152
q_0=[0,0,0,0,0];

q_1=[0,0,0,0,0];
q_2=[120,0,0,0,0];
q_3=[-120,0,0,0,0];
T_arm=[457,102,152];
TR_arm = [1 0 0;
          0 1 0;
          0 0 1];

Link_Arm=DHfk_J_Puma560(Link_Arm,q_arm,true);
DHFk_hand_with_arm(Link_Arm,q_1(2:end),q_2(2:end),q_3(2:end),true);
DrawSphere(catch_ball_center,catch_ball_radius,0);
drawnow;
pause;
%% 机械臂逆解
done_arm = false;
learning_rate = 0.8;
num=0;
while ~done_arm
    Link_Arm=DHfk_J_Puma560(Link_Arm,q_arm,true);
    [q_arm,done_arm]=Arm_IK_Sol(Link_Arm,q_arm,T_arm,TR_arm,learning_rate);
    num=num+1;
    if(num>300)
        disp("逆解失败");
        break;
    end
end
disp(num);
Link_Arm=DHfk_J_Puma560(Link_Arm,q_arm,true);
DHFk_hand_with_arm(Link_Arm,q_1(2:end),q_2(2:end),q_3(2:end),true);
DrawSphere(catch_ball_center,catch_ball_radius,0);
drawnow;
%% 抓取点规划
num_points = 60;
angle_threshold = 5 ;
distance_threshold = 3;%稳定阀值
[best_cp1, best_cp2, best_cp3] = Planning_point_with_arm(catch_ball_center, catch_ball_radius, num_points, angle_threshold, distance_threshold, shp1, shp2, shp3);
T1_pos=[best_cp1(1), best_cp1(2), best_cp1(3)];
T2_pos=[best_cp2(1), best_cp2(2), best_cp2(3)];
T3_pos=[best_cp3(1), best_cp3(2), best_cp3(3)];
pause;
hold off;
%% 求解逆运动学
learning_rate=1;
num=0;
done_1 = false;
done_2 = false;
done_3 = false;
load('J_matrix.mat', 'J_matrix');
global mf;
mf = matlabFunction(J_matrix);
% profile on; % 开启性能分析
tic
% while ~done_1
while ~(done_1 && done_2 && done_3)
    DHFk_hand_with_arm(Link_Arm,q_1(2:end),q_2(2:end),q_3(2:end),true);
    if ~done_1
        [q_1, done_1] = IK_Sol_with_arm(Link_1, q_1, T1_pos, learning_rate);
    end
    if ~done_2
        [q_2, done_2] = IK_Sol_with_arm(Link_2, q_2, T2_pos, learning_rate);
    end
    if ~done_3
        [q_3, done_3] = IK_Sol_with_arm(Link_3, q_3, T3_pos, learning_rate);
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
Link_Arm=DHfk_J_Puma560(Link_Arm,q_arm,true);
DHFk_hand_with_arm(Link_Arm,q_1(2:end),q_2(2:end),q_3(2:end),true);
DrawSphere(catch_ball_center,catch_ball_radius,0);
drawnow;

