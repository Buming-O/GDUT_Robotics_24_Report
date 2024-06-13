function Link=DHfk_J_Puma560(Link,q_arm,Draw)
% close all
if Draw
cla;
end
% global Link
radius    = 25;  %25
len       = 60;  %60
joint_col = 0;

plot3(0,0,0,'ro'); 

 
 Link(2).th=q_arm(1)*pi/180;
 Link(3).th=q_arm(2)*pi/180;
 Link(4).th=q_arm(3)*pi/180;
 Link(5).th=q_arm(4)*pi/180;
 Link(6).th=q_arm(5)*pi/180;
 Link(7).th=q_arm(6)*pi/180;

p0=[0,0,0]';


for i=1:9
    Link=Matrix_dh(Link,i);
end


for i=2:7
      Link(i).A=Link(i-1).A*Link(i).A;
      Link(i).p= Link(i).A(:,4);
      Link(i).n= Link(i).A(:,1);
      Link(i).o= Link(i).A(:,2);
      Link(i).a= Link(i).A(:,3);
      Link(i).R=[Link(i).n(1:3),Link(i).o(1:3),Link(i).a(1:3)];
      if Draw
          Connect3D(Link(i-1).p,Link(i).p,'b',2); hold on;
          plot3(Link(i).p(1),Link(i).p(2),Link(i).p(3),'rx');hold on;
          if i<=7
              DrawCylinder(Link(i-1).p, Link(i-1).R * Link(i).az, radius,len, joint_col); hold on;
          end 
      end
end
% view(125,52);
% set (gcf,'Position',[650,100,700,600])
axis([-900,900,-900,900,0,1200]);
% xlabel('x');
% ylabel('y'); 
% zlabel('z');
% grid on;
drawnow;
% hold off;
% if(fcla)
%     cla;
% end




