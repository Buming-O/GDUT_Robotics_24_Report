function J=Jacobian4DoF(Link,q_th_vaule)
% close all

jsize=5;
J=zeros(6,jsize);

Link(2).th=q_th_vaule(2)*pi/180;
Link(3).th=q_th_vaule(3)*pi/180;
Link(4).th=q_th_vaule(4)*pi/180;
Link(5).th=q_th_vaule(5)*pi/180;

for i=1:5
    Link=Matrix_dh(Link,i);
end
% 
Link(1).p=Link(1).p(1:3);
for i=2:5
    Link(i).A=Link(i-1).A*Link(i).A;
    Link(i).p= Link(i).A(1:3,4);
    Link(i).n= Link(i).A(:,1);
    Link(i).o= Link(i).A(:,2);
    Link(i).a= Link(i).A(:,3);
    Link(i).R=[Link(i).n(1:3),Link(i).o(1:3),Link(i).a(1:3)];
end

for n=1:jsize
    a=Link(n).R*Link(n).az;
    J(:,n)=[cross(a,Link(5).p-Link(n).p); a];
end
