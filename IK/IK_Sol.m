function [q_value, done] = IK_Sol(Link,q_value,Tpos,learning_rate)
ToDeg = 180/pi;
%获取机械臂末端当前位置
ex=Link(5).p(1);
ey=Link(5).p(2);
ez=Link(5).p(3);
global th2_min th2_max th3_min th3_max th4_min th4_max th5_min th5_max;
% 计算误差
p_err =[Tpos(1)-ex, Tpos(2)-ey, Tpos(3)-ez]' ;%计算位置误差
Loss = norm(p_err);  %误差评价
%     disp(Loss);

if Loss<10
    learning_rate = 1;
end

% 小于期望误差则结束迭代
if Loss<1e-4
    done = true;
    return;
else
    done = false;
end

%否则计算雅可比矩阵并计算角度修正量
J=Jacobian_finger(q_value);
dth = learning_rate * pinv(J) * p_err;  %计算修正量，此处单位为弧度

%     disp(dth);
%      th1=th1+dth(1)*ToDeg;


% 计算修正后的关节角度
q_value(2) = q_value(2) + dth(2) * ToDeg;
q_value(3) = q_value(3) + dth(3) * ToDeg;
q_value(4) = q_value(4) + dth(4) * ToDeg;
q_value(5) = q_value(5) + dth(5) * ToDeg;

% q_value(2) = q_value(2) + dth(1) * ToDeg;
% q_value(3) = q_value(3) + dth(2) * ToDeg;
% q_value(4) = q_value(4) + dth(3) * ToDeg;
% q_value(5) = q_value(5) + dth(4) * ToDeg;
% 关节空间约束
q_value(2) = max(min(q_value(2), th2_max), th2_min);
q_value(3) = max(min(q_value(3), th3_max), th3_min); 
q_value(4) = max(min(q_value(4), th4_max), th4_min);
q_value(5) = max(min(q_value(5), th5_max), th5_min);

plot3(ex,ey,ez,'r.');hold on;
drawnow;
return;
end

