clc;
clear all;
close all;

num=1;
learning_rate = 0.9;
ToDeg = 180/pi;
ToRad = pi/180;
Build;
global Link_1
global Link_2
global Link_3

%������ʼ���ؽ�λ��
th1=0;
th2=0;
th3=0;
th4=0;
th5=0;
q=[th2,th3,th4,th5];
q_0=[0,0,0,0];

%% ����Ŀ��ĩ��λ�ü���ת����
Tpos=[15,0,-78]';     %����λ��
% Tpos=[120,0,0]';     %����λ��
% Tpos=[-17,37.7,-53.57]';     %����λ��
% Tpos=[-2.3,-87.82,-57.91]';     %����λ��

%% ���ƻ�е�۳�ʼλ�˼�ĩ����̬
DHFk_hand(q_0,q_0,q,true);
pause; 
plot3(Tpos(1),Tpos(2),Tpos(3),'kX'); hold on;
view(-21,12);
pause;  cla;

tic
while (1)
    q=[th2,th3,th4,th5];
    % FK���㲢���ƻ����ˣ���Ŀ���
    plot3(Tpos(1),Tpos(2),Tpos(3),'kX'); hold on;
    DHFk_hand(q,q_0,q_0,true);
    
    %��ȡ��е��ĩ�˵�ǰλ��
    ex=Link_1(5).p(1);
    ey=Link_1(5).p(2);
    ez=Link_1(5).p(3);
     
    % �������
     p_err =[Tpos(1)-ex, Tpos(2)-ey, Tpos(3)-ez]' ;%����λ�����
    w_err=[0,0,0]'; % 3DoF��������̬��������̬�������
    Loss = norm(p_err) + norm(w_err)  %�������
%     disp(Loss);
    
    % С������������������
    if Loss<1e-5
        break;
    end
    
    %��������ſɱȾ��󲢼���Ƕ�������
%     J=Jacobian_finger([th1,th2,th3,th4,th5]);
    J=Jacobian4DoF(Link_1,[th1,th2,th3,th4,th5]);
    dth = learning_rate * pinv(J) * [p_err;w_err];  %�������������˴���λΪ����

    % ����������ĹؽڽǶ� th1_new = th1 + dth(1) * ToDeg; th2_new = th2 + dth(2) * ToDeg; th3_new = th3 + dth(3) * ToDeg; th4_new = th4 + dth(4) * ToDeg; 
    % �ؽڷ�ΧԼ�� th1_new = max(min(th1_new, th1_max), th1_min); th2_new = max(min(th2_new, th2_max), th2_min); th3_new = max(min(th3_new, th3_max), th3_min); th4_new = max(min(th4_new, th4_max), th4_min);
    % ���¹ؽڽǶ� th1 = th1_new; th2 = th2_new; th3 = th3_new; th4 = th4_new;
%     disp(dth);
%      th1=th1+dth(1)*ToDeg;
    th2=th2+dth(1)*ToDeg;
    th3=th3+dth(2)*ToDeg;
    th4=th4+dth(3)*ToDeg;
    th5=th5+dth(4)*ToDeg;
%     pause;
    
    x(num)=ex;
    y(num)=ey;
    z(num)=ez;
    num=num+1;
    plot3(x,y,z,'r.');grid on;
    hold on;    
    drawnow;
end
toc
%%�ٴλ��ƻ����˱���ͼ��
plot3(x,y,z,'r.');grid on;
DHFk_hand(q_0,q_0,q,true);
plot3(Tpos(1),Tpos(2),Tpos(3),'kX'); hold on;
disp(num);






