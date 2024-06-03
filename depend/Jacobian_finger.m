function J = Jacobian_finger(q_th_vaule)
%    load('j_matrix.mat', 'j_matrix');
    load('test_j_matrix.mat', 'test_j_matrix');
   % 定义符号变量
    syms theta1 theta2 theta3 theta4 theta;
    ToRad = pi/180;
    theta1 = q_th_vaule(1)*ToRad;
    theta2 = q_th_vaule(2)*ToRad;
    theta3 = q_th_vaule(3)*ToRad;
    theta4 = q_th_vaule(4)*ToRad;
    theta5 = q_th_vaule(5)*ToRad;
    % 计算雅克比矩阵
    J = subs(test_j_matrix);
%     J = J(:, 2:end);
%     J=J(1:3,:);
    J = double(J);
   return;
end

