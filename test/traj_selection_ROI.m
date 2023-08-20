% Trajectory Selection based on Region of Interest (ROI)
% Marcus Rosette
% 7/24/2023

clear
clc

%% Load Robot and Config Data
% Create Robot Representation
bravo = importrobot('bravo7.urdf', DataFormat='row');

data = load('WorkspaceData\pitch_data\pitch_torque_camera_vehicle_trans.mat');
com = data.com;
configs = data.config;
%% Collect ROI Index
% COM at ROI: 8.5872-18.0712 (in) OR 0.218-0.459 (m)

idx = linspace(1, length(com), length(com))';
roi_idx = idx(com>=0.218 & com<=0.459);

%% Select Initial Config
initial_config = [pi, pi-0.1, 0.6, 0, pi-0.5, pi/2, 0];
show(bravo, initial_config);

%% Select Final Config
num_configs = 10;
rand_idx = randi([1 length(roi_idx)], 1, num_configs)';
configs_idx = roi_idx(rand_idx);

%% Show Robot
title('Final Arm Configurations')
zlabel('z-axis (m)')
xlabel('x-axis (m)')
view(0, 0)
lightangle(0, 60)
xlim([-0.5, 1])
ylim([-0.1, 0.1])
zlim([-0.6, 0.8])
hold on

final_configs = zeros(num_configs, 7);
for i = 1:length(configs_idx)
    % Display the URDF
    final_configs(i, 1:7) = [pi, configs(configs_idx(i), 1) configs(configs_idx(i), 2), 0, configs(configs_idx(i), 3), pi/2, 0];
    show(bravo, final_configs(i, 1:7), 'PreservePlot', false, 'FastUpdate', true);
    grid off
    drawnow
    pause(1)
end

% %% Save data to .mat file
% % Setup .mat file storage method
% filename = 'WorkspaceData/pitch_data/CHANGE_THIS_hardware_configs.mat';
% file = matfile(filename, 'Writable', true); % Create new file instance
% file.configs(1:num_configs, 1:7) = final_configs;
% 
