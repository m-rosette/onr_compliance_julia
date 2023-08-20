% Comparison of harware and simulation results
% Marcus Rosette
% 6/28/2023

clc
clear

%% Load data
% % Gather ee-points (X and Z positions of the heatmap)
% num_points = 100;
% x = linspace(-0.75, 1, num_points)';
% z = linspace(-0.75, 1, num_points)';
% workspace = [x, z];
% [X, Z] = meshgrid(x, z);
% x_pos = reshape(X, [num_points^2, 1]);
% z_pos = reshape(Z, [num_points^2, 1]);

% Gather and reshape pitch and torque data
sim_data = load('WorkspaceData\pitch_data\pitch_torque_camera_vehicle_trans.mat');
pitch = sim_data.pitch;
torque = sim_data.torque;
com = sim_data.com;
config = sim_data.config;
ee_point = sim_data.ee_point;

% Reshape the column data to match the table size of heatmap
% pitch = reshape(sim_data.pitch, [num_points, num_points]);
% torque = reshape(sim_data.torque, [num_points, num_points]);
% com = reshape(sim_data.com, [num_points, num_points]);

%% Choose Discritized locations
num_test_points = 10;
rand_idx = zeros(num_test_points, 1);
pitch_test = zeros(num_test_points, 1);
torque_test = zeros(num_test_points, 1);
com_test = zeros(num_test_points, 1);
config_test = zeros(num_test_points, 3);
ee_point_test =zeros(num_test_points, 3);

i = 1;
no_nan = 1;
while no_nan
    rand_num = randi(length(pitch), 1, 1);
    if isnan(pitch(rand_num, 1))
        continue
    end
    rand_idx(i) = rand_num;
    
    % Data at rand_idx
    pitch_test(i, 1) = pitch(rand_num);
    torque_test(i, 1) = torque(rand_num);
    com_test(i, 1) = com(rand_num);
    config_test(i, :) = config(rand_num, :);
    ee_point_test(i, :) = ee_point(rand_num, :);

    % Increment counter
    i = i + 1;

    % Check while loop statement
    if i == num_test_points + 1
        no_nan = 0;
    end
end

%% Save data to .mat file
% Setup .mat file storage method
filename = 'WorkspaceData/pitch_data/hardware_test.mat';
file = matfile(filename, 'Writable', true); % Create new file instance
file.idx(1:num_test_points, 1) = rand_idx;
file.pitch(1:num_test_points, 1) = pitch_test;
file.torque(1:num_test_points, 1) = torque_test;
file.com(1:num_test_points, 1) = com_test;
file.config(1:num_test_points, 1:3) = config_test;
file.ee_point(1:num_test_points, 1:3) = ee_point_test;

