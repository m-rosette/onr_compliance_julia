% Manipulation Region of Interest
% Marcus Rosette
% 7/6/2023

clear
clc
clf

%% Load Robot and Config Data
% Create Robot Representation
bravo = importrobot('bravo7.urdf', DataFormat='row');

data = load('WorkspaceData\pitch_data\pitch_torque_camera_vehicle_trans.mat');

%% Find config idx to test
disc_points = 20;
com_to_test = linspace(0.0254, max(data.com), disc_points);

tol = 0.0005;
    % Good configurations starting at 9
    % Good configurations ending at 19
    % Overal region of interest (ROI): 9-19
    % COM at ROI: 8.5872-18.0712 (in)
com_idx = 12;
logic_list = (data.com >= com_to_test(com_idx) - tol) & (data.com <= com_to_test(com_idx) + tol);
idx = find(logic_list);
disp('COM Location (in):')
disp(com_to_test(com_idx) * 39.37)
disp(' ')
disp('Number of COM points within tol:')
disp(length(idx))
disp(' ')

%% Display Robot
configs = data.config;

title('Configuration at Discrete COM Locations')
zlabel('z-axis (m)')
xlabel('x-axis (m)')
view(0, 0)
lightangle(0, 60)
xlim([-0.5, 1])
ylim([-0.1, 0.1])
zlim([-0.6, 0.8])
hold on
for i = 1:length(idx)
    % Display the URDF
    config = [pi, configs(idx(i), 1) configs(idx(i), 2), 0, configs(idx(i), 3), 0, 0];
    show(bravo, config, 'PreservePlot', false, 'FastUpdate', true);
    grid off
    drawnow
    pause(1.5)
end
