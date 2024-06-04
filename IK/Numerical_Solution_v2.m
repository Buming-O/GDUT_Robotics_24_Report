clc;
clear all;
close all;
num=0;
learning_rate = 0.9;
ToDeg = 180/pi;
ToRad = pi/180;
% ������İ뾶������λ��
ball_radius = 20;
ball_center = [0, 0, -50];

global Link_1
global Link_2
global Link_3
Build;
% load('Link_1.mat','Link_1');
% load('Link_2.mat','Link_2');
% load('Link_3.mat','Link_3');
global th2_min th2_max th3_min th3_max th4_min th4_max th5_min th5_max;
th2_min=-90; th2_max=90;   % Range for th2
th3_min=0; th3_max=60;  % Range for th3
th4_min=0; th4_max=60;   % Range for th4
th5_min=0; th5_max=80;     % Range for th5

%% ������ʼ���ؽ�λ��
q_1=[0,0,0,0,0];
q_2=[120,0,0,0,0];
q_3=[-120,0,0,0,0];

%% ����Ŀ��ĩ��λ�ü���ת����
% T1_pos=[20,0,-51.65]';     %����λ��
% T2_pos=[-8,18.25,-51.65]';     %����λ��
% T3_pos=[-7.78,-17.75,-51.65]';     %����λ��
theta1 = 0; % �Ӵ���1�ĽǶ�
phi1 = pi/2;  % �Ӵ���1�ĽǶ�
theta2 = 2*pi/3; % �Ӵ���2�ĽǶ�
phi2 = pi/2;  % �Ӵ���2�ĽǶ�
theta3 = -2*pi/3; % �Ӵ���3�ĽǶ�
phi3 = pi/2;  % �Ӵ���3�ĽǶ�
T1_pos=ball_center + ball_radius * [cos(theta1) * sin(phi1), sin(theta1) * sin(phi1), cos(phi1)];
T2_pos=ball_center + ball_radius * [cos(theta2) * sin(phi2), sin(theta2) * sin(phi2), cos(phi2)];
T3_pos=ball_center + ball_radius * [cos(theta3) * sin(phi3), sin(theta3) * sin(phi3), cos(phi3)];

%% ���ƻ�е�۳�ʼλ�˼�ĩ����̬
DHFk_hand(q_1(2:end),q_2(2:end),q_3(2:end),true);
plot3(T1_pos(1),T1_pos(2),T1_pos(3),'ro', 'MarkerSize', 5, 'LineWidth', 3); hold on;
plot3(T2_pos(1),T2_pos(2),T2_pos(3),'go', 'MarkerSize', 5, 'LineWidth', 3); hold on;
plot3(T3_pos(1),T3_pos(2),T3_pos(3),'bo', 'MarkerSize', 5, 'LineWidth', 3); hold on;
DrawSphere(ball_center,ball_radius,0);
drawnow;
view(-21,12);
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
drawnow;






