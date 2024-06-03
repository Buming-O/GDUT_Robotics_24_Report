% close all;
% clear;

global Link_2

% Number of random samples


th_ranges = [
    -90, 90;   % Range for th1
    0, 60;  % Range for th2
    0, 60;   % Range for th3
    0, 80     % Range for th4
];

grid on;
hold on;
num_samples = 2000;
for sample = 1:num_samples
    th1 = th_ranges(1,1) + (th_ranges(1,2) - th_ranges(1,1)) * rand;
    th2 = th_ranges(2,1) + (th_ranges(2,2) - th_ranges(2,1)) * rand;
    th3 = th_ranges(3,1) + (th_ranges(3,2) - th_ranges(3,1)) * rand;
    th4 = th_ranges(4,1) + (th_ranges(4,2) - th_ranges(4,1)) * rand;

    Link_2(2).th =  th1 * ToRad; 
    Link_2(3).th =  th2 * ToRad;
    Link_2(4).th =  th3 * ToRad; 
    Link_2(5).th =  th4 * ToRad;

    for i = 1:5
        Link_2= Matrix_dh(Link_2,i);
    end
    for i = 2:5
        Link_2(i).A = Link_2(i-1).A * Link_2(i).A;
        Link_2(i).p = Link_2(i).A(:, 4); 
    end
    
    plot3(Link_2(5).p(1), Link_2(5).p(2), Link_2(5).p(3), 'go');
end
plot3(0, 0, 0, 'g+', 'MarkerSize', 10);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('ROM');
grid on;
hold off;
axis auto;