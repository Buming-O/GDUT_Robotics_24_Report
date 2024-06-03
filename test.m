clear all;
Build;
% qq=[-120,30,60,60,60];
% % qq=[0,0,0,0,0];
% j1=Jacobian4DoF(Link_3,qq)
% j2=Jacobian_finger(qq)
% 
% ToRad = pi / 180;
% load('test_j_matrix.mat', 'test_j_matrix');
% disp(test_j_matrix);

% %           theta      d        a        alpha 
% L(1)=Link([  Link_1(1).th       Link_1(1).dz      Link_1(1).dx      Link_1(1).alf ],'standard');
% L(2)=Link([  Link_1(2).th       Link_1(2).dz      Link_1(2).dx      Link_1(2).alf ],'standard'); L(2).qlim=[-pi,pi];
% L(3)=Link([  Link_1(3).th       Link_1(3).dz      Link_1(3).dx      Link_1(3).alf],'standard');  L(3).qlim=[-pi,pi];
% L(4)=Link([  Link_1(4).th       Link_1(4).dz      Link_1(4).dx      Link_1(4).alf],'standard');  L(4).qlim=[-pi,pi];
% L(5)=Link([  Link_1(5).th       Link_1(5).dz      Link_1(5).dx      Link_1(5).alf],'standard');  L(5).qlim=[-pi,pi]; 
% % 把上述连杆“串起来”
% Scara=SerialLink(L,'name','thumb');
% Scara.teach();
% Scara.jacob0(qq)
% Scara.jacobe(qq)
for i = 1:5
       Link_1= Matrix_dh(Link_1,i);
       Link_2= Matrix_dh(Link_2,i);
       Link_3= Matrix_dh(Link_3,i);
end
% pinv(j1)
% pinv(j2)