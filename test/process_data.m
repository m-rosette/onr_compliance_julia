%% Process Initial Dataset
clear
clc

% Load data
vehicle_pitch_data = load('WorkspaceData/pitch_data/arm_camera_pitch_data.csv');
arm_torque_data = load('WorkspaceData/pitch_data/arm_vehicle_torque.csv');

% Unit conversion
vehicle_pitch_data = vehicle_pitch_data(:) * (180 / pi);    % Rad to deg
arm_torque_data = arm_torque_data(:) * 9.81;                % Kg-m to N-m

% Average neighbors to single point
pitch_avg = zeros(10000, 1);
torque_avg = zeros(10000, 1);
num_neighbors = 10;
idx_count = 0;
for i = 1:length(pitch_avg)
    % Maintain NaN values if outside of reachable space
    if isnan(vehicle_pitch_data(idx_count + 1))
        pitch_avg(i) = NaN;
        torque_avg(i) = NaN;
        % Increment counter
        idx_count = idx_count + 1;
    else
        % Average neighbors
        pitch_avg(i) = sum(vehicle_pitch_data(idx_count:idx_count + num_neighbors - 1)) / num_neighbors;
        torque_avg(i) = sum(arm_torque_data(idx_count:idx_count + num_neighbors - 1)) / num_neighbors;
        % Increment counter by number of neighbors
        idx_count = idx_count + num_neighbors;
    end
end

stiffness = torque_avg ./ pitch_avg;

%% Save data to .mat file
% Setup .mat file storage method
filename = 'WorkspaceData/pitch_data/pitch_torque_camera_vehicle_trans.mat';
file = matfile(filename, 'Writable', true); % Create new file instance
file.pitch(1:10000, 1) = pitch_avg;
file.torque(1:10000, 1) = torque_avg;
% file.stiffness_rot(1:10000, 1) = stiffness;