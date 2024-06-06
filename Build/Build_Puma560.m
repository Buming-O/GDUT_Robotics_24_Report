% %Build Robot by D_H methods


ToDeg = 180/pi;
ToRad = pi/180;
UX = [1 0 0]';
UY = [0 1 0]';
UZ = [0 0 1]';

Link_Arm= struct('name','Body' , 'th',  0, 'dz', 0, 'dx', 0, 'alf',0*ToRad,'az',UZ);     % az 
Link_Arm(1)= struct('name','Base' , 'th',  0*ToRad, 'dz', 0, 'dx', 0, 'alf',0*ToRad,'az',UZ);        %BASE to 1
Link_Arm(2) = struct('name','J1' , 'th',   90*ToRad, 'dz', 685, 'dx', 0, 'alf',-90*ToRad,'az',UZ);       %1 TO 2 
Link_Arm(3) = struct('name','J2' , 'th',  0*ToRad, 'dz', 152, 'dx', 381, 'alf',0*ToRad,'az',UZ);    %2 TO 3
Link_Arm(4) = struct('name','J3' , 'th',  90*ToRad, 'dz', -50, 'dx', 25, 'alf',90*ToRad,'az',UZ);          %3 TO 4
Link_Arm(5) = struct('name','J4' , 'th',  0*ToRad, 'dz', 457, 'dx', 0, 'alf',-90*ToRad,'az',UZ);          %4 TO 5
Link_Arm(6) = struct('name','J5' , 'th',  0*ToRad, 'dz', 0, 'dx', 0, 'alf',90*ToRad,'az',UZ);          %5 TO 6
Link_Arm(7) = struct('name','J6' , 'th',  0*ToRad, 'dz', 127, 'dx', 0, 'alf',0,'az',UZ);          %6 TO E
Link_Arm(8) = struct('name','J7' , 'th',  0*ToRad, 'dz', 0, 'dx', 100, 'alf',90*ToRad,'az',UZ);          %E TO E1
Link_Arm(9) = struct('name','J8' , 'th',  0*ToRad, 'dz', 0, 'dx', -200, 'alf',0,'az',UZ);          %E1 TO E2