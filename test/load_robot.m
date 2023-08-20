% Bravo7 workspace calculation
% Marcus Rosette
% 12/08/2022

clear
clc

file = load("WorkspaceData\pitch_data\hardware_configs_actuated_only.mat");

data = load('WorkspaceData\pitch_data\pitch_torque_camera_vehicle_trans.mat');
configs = file.configs;

config_num = 1;
% idx = find(data.config==[configs(config_num, 2), configs(config_num, 3), configs(config_num, 5)]);
% idx = find(data.config==[3.04, 0.6, 2.64]);
% com = data.com(idx(1)) * 39.37 % meters to inches

% Create Robot Representation
bravo = importrobot('bravo7.urdf', DataFormat='column');

% % show(bravo, configs(config_num, :)');
% config1 = [3.5, 2.3, 1.5, 2.1, 1.5, 3, 5];
% config2 = [2.9, 2.3, 2.1, 5, 1.5, 3, 5];
% 
% filename = 'WorkspaceData/pitch_data/out_of_plane_config.mat';
% file = matfile(filename, 'Writable', true); % Create new file instance
% file.configs(1, 1:7) = config1;
% file.configs(2, 1:7) = config2;

zero_column = zeros(length(configs), 1);
pi_column = pi * ones(length(configs), 1);
configs_full = [pi_column, configs(:, 1), configs(:, 2), ...
    zero_column, configs(:, 3), zero_column, zero_column];

show(bravo, configs_full(4, :)');
view(0, 0)
lightangle(0, 60)
xlim([-0.5, 1])
ylim([-0.1, 0.1])
zlim([-0.6, 0.8])




% arm_vehicle = importrobot('arm_vehicle.urdf', DataFormat='column');

% Generate Trajectory and Check for Collisions
% config = file.collision_free_angles(row, 1:3);

% If the two geometries are in collision, is_self_collide is 1. Otherwise, the value is 0
% is_self_collide = checkCollision(bravo, startConfig, 'SkippedSelfCollisions', 'parent');

% Loop through a set of collision free angles
% row = 800000; 
% range = row + 100;
% for i = row:range
%     % Display the URDF
%     show(bravo, file.collision_free_angles(i, :)', 'PreservePlot', false, 'FastUpdate', true);
%     drawnow
% end
