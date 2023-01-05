% Angle Generation
% Marcus Rosette
% 01/03/2023

clear
clc

%% Test Using ndgrid

robot = importrobot('three_link_arm.urdf', DataFormat='column');

num_samples = 10;
j1 = linspace(0, 2*pi, num_samples);
j2 = linspace(0, 2*pi, num_samples);
j3 = linspace(0, 2*pi, num_samples);

[J1, J2, J3] = ndgrid(j1, j2, j3);

collision_check = zeros(size(J1));
tic
for i = 1:numel(J1) % Can iterate over either J1, J2, or J3 (same size)
%     if ~mod(i,100)
%         elapsedTime = toc;
%         estimatedTime = elapsedTime*numel(J1)/i;
%         disp(i/numel(J1));
%         ETA = estimatedTime - elapsedTime;
%         disp(['ETA: ',num2str(ETA/60),' minutes']);
%     end
    current_config = [J1(i); J2(i); J3(i)];
    collision_check(i) = checkCollision(robot, current_config, 'Exhaustive', 'on', 'SkippedSelfCollisions','parent');
end
toc


% tic
% collision = find(collision_check == 1);
% collision_free = find(collision_check == 0);
% num_collision = 50;
% test_collision = collision(num_collision);
% 
% current_config = [J1(test_collision); J2(test_collision); J3(test_collision)];
% show(robot, current_config, 'visuals','on','collision','off');
% view(2)
% ax = gca;
% ax.Projection = 'orthographic';
% xlim([-3.5 3.5])
% ylim([-2 2])
% title("Collision Detected")
% toc







