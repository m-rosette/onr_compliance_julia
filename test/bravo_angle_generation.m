% Bravo Angle Generation
% Marcus Rosette
% 01/04/2023

clear
clc

%% Test Using ndgrid

robot = importrobot('bravo7.urdf', DataFormat='column');

num_samples = 3;
j1 = linspace(0, 2*pi, num_samples);
j2 = linspace(0, pi, num_samples);
j3 = linspace(0, pi, num_samples);
j4 = linspace(0, 2*pi, num_samples);
j5 = linspace(0, pi, num_samples);
j6 = linspace(0, 2*pi, num_samples);
j7 = linspace(0, 0, num_samples);     % Jaw prismatic [mm] - URDF does not seem to update with this
j8 = linspace(0, 0.65, num_samples);  % Jaw angle - limit found via measuring on screen
j9 = linspace(0, 0.65, num_samples);  % Jaw angle - limit found via measuring on screen

[J1, J2, J3, J4, J5, J6, J7] = ndgrid(j1, j2, j3, j4, j5, j6, j7);

collision_check = zeros(size(J1));
tic
for i = 1:numel(J1) % Can iterate over either J1, J2, J3, etc (same size)
    current_config = [J1(i); J2(i); J3(i); J4(i); J5(i); J6(i); J7(i)];
%     collision_check(i) = checkCollision(robot, current_config, 'Exhaustive', 'on', 'SkippedSelfCollisions','parent');
end
toc

% tic
% collision = find(collision_check == 1);
% collision_free = find(collision_check == 0);