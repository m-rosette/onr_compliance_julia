% Comparison of harware and simulation results
% Marcus Rosette
% 6/28/2023

clc
clear

% Load data
workspace = load('WorkspaceData\pitch_data\disc_bin2_config_space_100.mat');
sim_data = load('WorkspaceData\pitch_data\pitch_torque_camera_vehicle_trans.mat');


% Choose Discritized locations
