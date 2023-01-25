function file = bravo_angle_gen_3(num_samples, filename)
disp("Starting angle generation calcs")

% Bravo Arm Self Collision Detection
% Marcus Rosette
% 01/16/2023

% clear
% clc

% Setup .mat file storage method
% filename = 'collision_free_thetas.mat';
if isfile(filename)     % Remove the preexisting file if exists
    delete(filename)
end
file = matfile(filename, 'Writable', true); % Create new file instance

% Import robot URDF
robot = importrobot('bravo7.urdf', DataFormat='column');

% Initialize calculation dimensions
% num_samples = 16; % Number of samples over joint space
num_dim = 6; % Number of dimensions

% Joint Limits
j1 = linspace(0, 2*pi, num_samples);
j2 = linspace(0, pi, num_samples);
j3 = linspace(0, pi, num_samples);
j4 = linspace(0, 2*pi, num_samples);
j5 = linspace(0, pi, num_samples);
j6 = linspace(0, 2*pi, num_samples); 

% Iteratively sample and test joint space for collisions
range = 1:num_samples; % loop range for each joint
collision_free_count = 0; % Start the counter
batch_size = (num_samples^num_dim)/num_samples; % Remains the same. Used in indexing method
batch = (num_samples^num_dim)/num_samples; % To be incremented with the increasing mat file size
collision_free_angles = zeros(batch_size, num_dim); % Initialize collision free angle storage
tic
for joint1 = range
    for joint2 = range
        for joint3 = range
            for joint4 = range
                for joint5 = range
                    for joint6 = range
                        current_config = [j1(joint1); j2(joint2); j3(joint3); j4(joint4); j5(joint5); j6(joint6)];
                        collision_check = checkCollision(robot, current_config, 'Exhaustive', 'on', 'SkippedSelfCollisions','parent');
                        if ~collision_check % If there is no collision
                            collision_free_count = collision_free_count + 1; % Increase count
                            collision_free_angles(collision_free_count, :) = current_config';

                            if collision_free_count == batch_size
                                file.collision_free_angles((batch-batch_size)+1:batch, 1:num_dim) = collision_free_angles;

                                batch = batch + batch_size; % Increment the batch index by the batch size
                                collision_free_angles = zeros(batch_size, num_dim); % Reset the batch of angles
                                collision_free_count = 0;   % Reset the collision free counter
                            end
                        end
                    end
                end 
            end
        end
    end
end
% Final catch to append any lasting collision free angle combinations that didnt make it into the batch update
if collision_free_count > 0 
    % Check if there is already data in the matfile
    var_in_mat = who(file);
    if ismember("collision_free_angles", var_in_mat)
        file.collision_free_angles(length(file.collision_free_angles)+1:length(file.collision_free_angles)+collision_free_count, 1:num_dim) = collision_free_angles(1:collision_free_count, 1:num_dim);
    else
        file.collision_free_angles(1:collision_free_count, 1:num_dim) = collision_free_angles(1:collision_free_count, 1:num_dim);
    end
end
toc
file.ElapsedTime = toc;  
disp("Calculation Complete")                                                                                                                                                                                                                                                                                                                                                                                                                            