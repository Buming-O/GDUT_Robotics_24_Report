function J = Jacobian_finger(q_th_value)
    global mf;
%     ToRad = pi/180;
    q_th_value= q_th_value*pi/180;
    J = mf(q_th_value(1),q_th_value(2),q_th_value(3),q_th_value(4),q_th_value(5));
   return;
end

