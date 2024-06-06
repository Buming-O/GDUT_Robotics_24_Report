function Link=DHfk_finger_with_arm(Link_Arm,Link,inputArg,Draw)
% close all
radius    = 5;
len       = 15;
joint_col = 0;

ToRad = pi/180;
Link(2).th=inputArg(1)*ToRad;
Link(3).th=inputArg(2)*ToRad;
Link(4).th=inputArg(3)*ToRad;
Link(5).th=inputArg(4)*ToRad;


%%
for i=1:5
    Link=Matrix_dh(Link,i); 
end
azz=[1,0,0,0;
      0,1,0,0;
      0,0,-1,0;
      0,0,0,1];
axx=[-1,0,0,0;
      0,1,0,0;
      0,0,1,0;
      0,0,0,1];
Link(1).A= Link_Arm(7).A*azz*axx*Link(1).A;%机械臂向下,X轴反转
Link(1).p = Link(1).A(:,4);
Link(1).n = Link(1).A(:,1);
Link(1).o = Link(1).A(:,2);
Link(1).a = Link(1).A(:,3);
Link(1).R=  [Link(1).n(1:3),Link(1).o(1:3),Link(1).a(1:3)];
for i=2:5
      Link(i).A= Link(i-1).A*Link(i).A;
      Link(i).p= Link(i).A(:,4);
      Link(i).n= Link(i).A(:,1);
      Link(i).o= Link(i).A(:,2);
      Link(i).a= Link(i).A(:,3);
      Link(i).R=[Link(i).n(1:3),Link(i).o(1:3),Link(i).a(1:3)];
      if Draw
          Connect3D(Link(i-1).p,Link(i).p,'b',2); hold on;
          DrawCylinder(Link(i-1).p, Link(i-1).R * Link(i).az, radius,len, joint_col); hold on;
      end
end
return;
% grid on;
% view(134,12);
% axis([-200,200,-200,200,-200,200]);
% xlabel('x');
% ylabel('y');
% zlabel('z');
% drawnow;
% pic=getframe;


