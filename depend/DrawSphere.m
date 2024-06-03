function h = DrawSphere(pos, radius, col)
% Draw a sphere
%
% pos: position of the center of the sphere [x, y, z]
% radius: radius of the sphere
% col: color of the sphere

% Define the number of subdivisions for the sphere
n = 20;

% Generate the sphere coordinates
[theta, phi] = meshgrid(linspace(0, pi, n), linspace(0, 2*pi, n));
x = radius * sin(theta) .* cos(phi);
y = radius * sin(theta) .* sin(phi);
z = radius * cos(theta);

% Adjust for position
x = x + pos(1);
y = y + pos(2);
z = z + pos(3);

% Define color for each face
cc = col * ones(size(x));

% Draw the sphere
h = surf(x, y, z, cc, 'EdgeColor', 'none');

% Add lighting for better visualization
camlight;
lighting gouraud;
end

