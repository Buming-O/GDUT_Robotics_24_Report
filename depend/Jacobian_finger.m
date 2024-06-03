function J = Jacobian_finger(q_th_value)
    global mf;
    % 定义符号变量
%     syms theta1 theta2 theta3 theta4 theta5;
    ToRad = pi/180;
%     theta1 = q_th_value(1)*ToRad;
%     theta2 = q_th_value(2)*ToRad;
%     theta3 = q_th_value(3)*ToRad;
%     theta4 = q_th_value(4)*ToRad;
%     theta5 = q_th_value(5)*ToRad;
% %     计算雅克比矩阵
%     J = mf(theta1,theta2,theta3,theta4,theta5);
    J = mf(q_th_value(1)*ToRad,q_th_value(2)*ToRad,q_th_value(3)*ToRad,q_th_value(4)*ToRad,q_th_value(5)*ToRad);
   return;
end

