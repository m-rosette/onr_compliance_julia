% Bravo Angle Generation
% Marcus Rosette
% 01/04/2023

clear
clc

%% Test Using ndgrid

robot = importrobot('bravo7.urdf', DataFormat='column');

num_samples = 120;
j1 = linspace(0, 2*pi, num_samples);
j2 = linspace(0, pi, num_samples);
j3 = linspace(0, pi, num_samples);
j4 = linspace(0, 2*pi, num_samples);
j5 = linspace(0, pi, num_samples);
j6 = linspace(0, 2*pi, num_samples);
% j7 = linspace(0, 0, num_samples);     % Jaw prismatic [mm] - URDF does not seem to update with this
j8 = linspace(0, 0.65, num_samples);  % Jaw angle - limit found via measuring on screen
j7 = linspace(0, 0.65, num_samples);  % Jaw angle - limit found via measuring on screen

% reshape(j7, num_samples, num_samples)
% [J1, J2, J3, J4, J5, J6, J7, J8, J9] = ndgrid(j1, j2, j3, j4, j5, j6, j7, j8, j9);

% Adjusted line 363 in URDF from prismatic to fixed
[J1, J2, J3, J4, J5, J6, J7, J8] = ndgrid(j1, j2, j3, j4, j5, j6, j7, j8);

collision_check = zeros(size(J1));
tic
for i = 1:numel(J1) % Can iterate over either J1, J2, J3, etc (same size)
    if ~mod(i,100)
        elapsedTime = toc;
        estimatedTime = elapsedTime*numel(J1)/i;
%         disp(i/numel(J1));
        ETA = estimatedTime - elapsedTime;
        disp(['ETA: ',num2str(ETA/60),' minutes']);
    end
    
    current_config = [J1(i); J2(i); J3(i); J4(i); J5(i); J6(i); J7(i); J8(i)];

    collision_check(i) = checkCollision(robot, current_config, 'Exhaustive', 'on', 'SkippedSelfCollisions','parent');
end
toc

% collide_index = find(collision_check == 1);
% i = 200;
% show(robot, [J1(i); J2(i); J3(i); J4(i); J5(i); J6(i); J7(i); J8(i)],'visuals','on','collision','on')




