function [q_value, done] = Arm_IK_Sol(Link,q_value,Tpos,TR,learning_rate)
ToDeg = 180/pi;
% global th2_min th2_max th3_min th3_max th4_min th4_max th5_min th5_max;
%获取机械臂末端当前位置
ex=Link(7).p(1);
ey=Link(7).p(2);
ez=Link(7).p(3);
% 获取末端方向
current_orientation = Link(7).R; % 旋转矩阵


% disp(norm(current_orientation-TR, 'fro'));
if norm(current_orientation-TR, 'fro') < 0.1
    w_err = [0,0,0]';
else
     % 计算姿态误差
    w_err_matrix = TR' * current_orientation;
    theta=(trace(w_err_matrix)-1)/2;
    theta=acos(theta);
    w_err = [w_err_matrix(3, 2)-w_err_matrix(2, 3),...
            w_err_matrix(1, 3)-w_err_matrix(3, 1),...
            w_err_matrix(2, 1)-w_err_matrix(1, 2)]';
    w_err = w_err*theta/2*sin(theta);
end
% disp(norm(w_err));

% 计算误差
p_err = [Tpos(1)-ex, Tpos(2)-ey, Tpos(3)-ez]' ;%计算位置误差
Loss = norm(p_err) + norm(w_err);  %误差评价
% disp(Loss);
% 小于期望误差则结束迭代
if Loss<5e-3
    done = true;
    return;
else
    done = false;
end

%否则计算雅可比矩阵并计算角度修正量
J=Jacobian6DoF(Link,q_value);
% ToDeg_Matrix=repmat(ToDeg,1,4);
dth = pinv(J) * [p_err; w_err] * ToDeg * learning_rate;  %计算修正量，此处单位为弧度

% 计算修正后的关节角度
q_value(1) = q_value(1) + dth(1);
q_value(2) = q_value(2) + dth(2);
q_value(3) = q_value(3) + dth(3);
q_value(4) = q_value(4) + dth(4);
q_value(5) = q_value(5) + dth(5);
q_value(6) = q_value(6) + dth(6);
% % 关节空间约束
% q_value(2) = max(min(q_value(2), th2_max), th2_min);
% q_value(3) = max(min(q_value(3), th3_max), th3_min); 
% q_value(4) = max(min(q_value(4), th4_max), th4_min);
% q_value(5) = max(min(q_value(5), th5_max), th5_min);
%可以记录下点，完成逆运动学后统一绘制
% plot3(ex,ey,ez,'r.');
% drawnow;
return;
end

