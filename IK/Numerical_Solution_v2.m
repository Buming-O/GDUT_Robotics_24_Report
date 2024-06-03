clc;
clear all;
close all;
num=0;
learning_rate = 1;
ToDeg = 180/pi;
ToRad = pi/180;
global Link_1
global Link_2
global Link_3
Build;
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
T1_pos=[49.5,-11,-49.64]';     %����λ��
T2_pos=[-17,37.7,-53.57]';     %����λ��
T3_pos=[-2.3,-87.82,-57.91]';     %����λ��

%% ���ƻ�е�۳�ʼλ�˼�ĩ����̬
DHFk_hand(q_1(2:end),q_2(2:end),q_3(2:end),true);
pause;
plot3(T1_pos(1),T1_pos(2),T1_pos(3),'rX'); hold on;
plot3(T2_pos(1),T2_pos(2),T2_pos(3),'gX'); hold on;
plot3(T3_pos(1),T3_pos(2),T3_pos(3),'bX'); hold on;
view(-21,12);
pause;  cla;
hold on;
%%
done_1 = false;
done_2 = false;
done_3 = false;
load('test_j_matrix.mat', 'test_j_matrix');
global mf;
mf = matlabFunction(test_j_matrix);
profile on; % �������ܷ���
tic
while ~(done_1 && done_2 && done_3)
%     disp(['Done_1: ', num2str(done_1),'Done_2: ', num2str(done_2),'Done_3: ', num2str(done_3)]);
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
end
% disp(['��������: ', num2str(num)]);
toc
profile off; % �ر����ܷ���
profile viewer; % ��ʾ���ܷ������


%% �ٴλ��ƻ����˱���ͼ��
plot3(T1_pos(1),T1_pos(2),T1_pos(3),'rX'); hold on;
plot3(T2_pos(1),T2_pos(2),T2_pos(3),'gX'); hold on;
plot3(T3_pos(1),T3_pos(2),T3_pos(3),'bX'); hold on;
DHFk_hand(q_1(2:end),q_2(2:end),q_3(2:end),true);







