% Expected pitch between hardware and simulation
% Marcus Rosette
% 8/12/2023

clear
clc

% Enter config_num:
config_num = 3;

%% Load Config Data
% Create Robot Representation
bravo = importrobot('bravo7.urdf', DataFormat='row');

data = load('WorkspaceData\pitch_data\pitch_torque_camera_vehicle_trans.mat');
com = data.com;
configs = data.config;

roi_data = load('WorkspaceData\pitch_data\hardware_configs.mat');
roi_configs = roi_data.configs;

% Downsample roi_configs to eliminate joints that are not actuated
roi_configs(:, [1, 4, 6, 7]) = [];

%% Save actuated joint configs to its own .mat file
filename = 'WorkspaceData/pitch_data/hardware_configs_actuated_only.mat';
file = matfile(filename, 'Writable', true); % Create new file instance
file.configs(1, 1:3) = [3.04, 0.6, 2.64];
file.configs(2:11, 1:3) = roi_configs;

%% Find index of roi config from original list
[q, idx] = ismember(roi_configs(config_num, :), configs, 'rows');

%% Find COM at roi config
com_at_config = com(idx);
