function file = bravo_angle_gen_3(num_samples, filename)
disp("Starting angle generation calcs")

% Setup .mat file storage method
% filename = 'collision_free_thetas.mat';
if isfile(filename)     % Remove the preexisting file if exists
    delete(filename)
end
file = matfile(filename, 'Writable', true); % Create new file instance

% Import robot URDF
robot = importrobot('bravo7.urdf', DataFormat='column');

% Number of samples over joint space
% num_samples = 16; 

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
                            file.current_config(collision_free_count, 1:length(current_config)) = current_config'; % Save collision free angles
                        end
                    end
                end 
            end
        end
    end
end
toc   
disp("Calculation Complete")                                                               
end