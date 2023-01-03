% Three Link Arm Collision Check
% Marcus Rosette
% 01/02/2023

clear
clc

% Create robot representation
robot = importrobot('three_link_arm.urdf', DataFormat='column');

% Robot configuration collision check
initial_config = [0 0 0]';
final_config = [0 5*pi/6 5*pi/6]';

num_samples = 10;
traj = trapveltraj([initial_config final_config], num_samples, 'EndTime', 3);

collision_check = false(num_samples, 1); % Initialize array with no collisions
collision_traj = [];
collision_free_traj = [];

for i = 1:num_samples
    collision_check(i) = checkCollision(robot, traj(:, i), 'Exhaustive', 'on', 'SkippedSelfCollisions','parent');
    if collision_check(i)
        collision_traj(:, end+1) = traj(:, i);
    else
        collision_free_traj(:, end+1) = traj(:, i);

    end
end

% disp(collision_free_traj)





%         show(robot, traj(:, i), 'visuals','on','collision','on');
%         view(2)
%         ax = gca;
%         ax.Projection = 'orthographic';
%         xlim([-0.5 3.5])
%         ylim([-2 2])
%         title("Collision Detected")
