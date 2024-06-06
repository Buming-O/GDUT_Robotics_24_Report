function DHFk_hand_with_arm(Link_Arm,inputArg1,inputArg2,inputArg3,Draw)
    P=Link_Arm(7).p;
    R=Link_Arm(7).R;
    az = [0;0;1];
    global Link_1
    global Link_2
    global Link_3
%     global Link_Arm
    Link_1=DHfk_finger_with_arm(Link_Arm,Link_1,inputArg1,Draw);
    Link_2=DHfk_finger_with_arm(Link_Arm,Link_2,inputArg2,Draw);
    Link_3=DHfk_finger_with_arm(Link_Arm,Link_3,inputArg3,Draw);
    if Draw
%         cla;
%         axis([-200,200,-200,200,-200,200]);
        xlabel('x');
        ylabel('y');
        zlabel('z');
        DrawCylinder(P, R*az, 30 ,5, 0); hold on;grid on;
    end
%     hold off;
%     drawnow;
end

