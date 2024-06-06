function J = Jacobian_finger(q_th_value)
    global mf;
    q_th_value= deg2rad(q_th_value);
    J = mf(q_th_value(1),q_th_value(2),q_th_value(3),q_th_value(4),q_th_value(5));
   return;
end

