% Bravo Arm Self Collision Detection and Forward Kinematics
% Planar Bravo Arm
% No batch update
% Marcus Rosette
% 02/03/2023

clear
clc

% Setup .mat file storage method
filename = 'bravo_workspace.mat';
if isfile(filename)     % Remove the preexisting file if exists
    delete(filename)
end
file = matfile(filename, 'Writable', true); % Create new file instance

% Import robot URDF
robot = importrobot('bravo7_planar.urdf', DataFormat='column');

% Initialize calculation dimensions
num_samples = 181; % Number of samples over joint space
num_dim = 3; % Number of dimensions

% Joint Limits
j2 = linspace(0, pi, num_samples);
j3 = linspace(0, pi, num_samples);
j5 = linspace(0, pi, num_samples);

% Iteratively sample and test joint space for collisions
range = 1:num_samples; % loop range for each joint
collision_free_count = 1; % Start the counter
tic
for joint2 = range
    for joint3 = range
        for joint5 = range
            current_config = [j2(joint2); j3(joint3); j5(joint5)];
            collision_check = checkCollision(robot, current_config, 'Exhaustive', 'on', 'SkippedSelfCollisions','parent');
            if ~collision_check % If there is no collision
                file.collision_free_angles(collision_free_count, 1:3) = current_config';
        
                transform = getTransform(robot, current_config, "end_effector_tip", "world");
                file.ee_points(collision_free_count, 1:3) = transform(1:3, 4)';
        
                collision_free_count = collision_free_count + 1;
            end
        end
    end 
end
toc
file.ElapsedTime = toc;  
disp("Calculation Complete")                                                                                                                                                                                                                                                                                                                                                                                                                                