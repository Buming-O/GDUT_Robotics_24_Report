function DHFk_hand(inputArg1,inputArg2,inputArg3,Draw)
    
    p_0=[0;0;0;1];
    r_0=[1,0,0;0,1,0;0,0,1];
    az = [0;0;1];
    global Link_1
    global Link_2
    global Link_3
    Link_1=DHfk_finger(Link_1,inputArg1,Draw);
    Link_2=DHfk_finger(Link_2,inputArg2,Draw);
    Link_3=DHfk_finger(Link_3,inputArg3,Draw);
    if Draw
%         cla;
        axis([-200,200,-200,200,-200,200]);
        xlabel('x');
        ylabel('y');
        zlabel('z');
        DrawCylinder(p_0, r_0*az, 30 ,5, 0); hold on;grid on;
    end
%     hold off;
%     drawnow;
end

