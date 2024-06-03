% %Build Robot by D_H methods


ToDeg = 180/pi;
ToRad = pi/180;
UX = [1 0 0]';
UY = [0 1 0]';
UZ = [0 0 1]';

Link_1    = struct('name','Link_1' , 'th',  0, 'dz', 0, 'dx', 0, 'alf',0*ToRad,'az',UZ);     % az 
Link_1(1) = struct('name','J0' , 'th',  0*ToRad, 'dz', 0, 'dx', 30, 'alf',180*ToRad,'az',UZ);        %BASE to 1
Link_1(2) = struct('name','J1' , 'th',  0*ToRad, 'dz', 0, 'dx', 0, 'alf', 90*ToRad,'az',UZ);       %1 TO 2 
Link_1(3) = struct('name','J2' , 'th',  0*ToRad, 'dz', 0, 'dx', 30, 'alf',0*ToRad,'az',UZ);      %2 TO 3
Link_1(4) = struct('name','J3' , 'th',  0*ToRad, 'dz', 0, 'dx', 30, 'alf',0*ToRad,'az',UZ);         %3 TO 4
Link_1(5) = struct('name','J4' , 'th',  0*ToRad, 'dz', 0, 'dx', 30, 'alf',0*ToRad,'az',UZ);         %4 TO 5

Link_2    = struct('name','Link_2' , 'th',  0, 'dz', 0, 'dx', 0, 'alf',0*ToRad,'az',UZ);     % az 
Link_2(1) = struct('name','J0' , 'th',  120*ToRad, 'dz', 0, 'dx', 30, 'alf',180*ToRad,'az',UZ);        %BASE to 1
Link_2(2) = struct('name','J1' , 'th',  0*ToRad, 'dz', 0, 'dx', 0, 'alf', 90*ToRad,'az',UZ);       %1 TO 2 
Link_2(3) = struct('name','J2' , 'th',  0*ToRad, 'dz', 0, 'dx', 30, 'alf',0*ToRad,'az',UZ);      %2 TO 3
Link_2(4) = struct('name','J3' , 'th',  0*ToRad, 'dz', 0, 'dx', 30, 'alf',0*ToRad,'az',UZ);         %3 TO 4
Link_2(5) = struct('name','J4' , 'th',  0*ToRad, 'dz', 0, 'dx', 30, 'alf',0*ToRad,'az',UZ);         %4 TO 5

Link_3    = struct('name','Link_3' , 'th',  0, 'dz', 0, 'dx', 0, 'alf',0*ToRad,'az',UZ);     % az 
Link_3(1) = struct('name','J0' , 'th',  -120*ToRad, 'dz', 0, 'dx', 30, 'alf',180*ToRad,'az',UZ);        %BASE to 1
Link_3(2) = struct('name','J1' , 'th',  0*ToRad, 'dz', 0, 'dx', 0, 'alf', 90*ToRad,'az',UZ);       %1 TO 2 
Link_3(3) = struct('name','J2' , 'th',  0*ToRad, 'dz', 0, 'dx', 30, 'alf',0*ToRad,'az',UZ);      %2 TO 3
Link_3(4) = struct('name','J3' , 'th',  0*ToRad, 'dz', 0, 'dx', 30, 'alf',0*ToRad,'az',UZ);         %3 TO 4
Link_3(5) = struct('name','J4' , 'th',  0*ToRad, 'dz', 0, 'dx', 30, 'alf',0*ToRad,'az',UZ);         %4 TO 5


