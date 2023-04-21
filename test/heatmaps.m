%% Process Initial Dataset
clear
clc

% Load data
vehicle_pitch_data = load('WorkspaceData/pitch_data/bin2_pitch_data.csv');
arm_torque_data = load('WorkspaceData/pitch_data/test_bin2_torque_data.csv');

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

%% Processes Heatmap
% Gather data for the X and Y positions of the heatmap
num_points = 100;
X = linspace(-0.75, 1, num_points)';
Z = linspace(-0.75, 1, num_points)';

% Reshape the column data to match the table size of heatmap
pitch_reshape = reshape(pitch_avg, [num_points, num_points]);
torque_reshape = reshape(torque_avg, [num_points, num_points]);
stiffness = torque_reshape ./ pitch_reshape;

% There was a lone zero in the data in the top right corner (not critical)
if torque_reshape(100, 100) == 0
    torque_reshape(100, 100) = NaN;
end

%% Plotting Heatmaps
% Initialize the heatmap axes labels
x_label_list = cell(1, num_points);
z_label_list = cell(1, num_points);
for i = 1:num_points
    if ~mod(i, 5)
        x_label_list{i} = string(round(X(i), 2));
        z_label_list{i} = string(round(Z(i), 2));
    else
        x_label_list{i} = '';
        z_label_list{i} = '';
    end
end

%% Pitch Heatmap
figure
h = heatmap(X, Z, pitch_reshape);
colormap(parula(1000))
h.NodeChildren(3).YDir = 'normal';  
h.Title = "Vehicle Pitch Intensity (deg)";
h.XLabel = "Bravo EE-Position - X (m)";
h.YLabel = "Bravo EE-Position - Z (m)";
h.XDisplayLabels = x_label_list;
h.YDisplayLabels = x_label_list;
h.GridVisible = "off";
h.MissingDataColor = 'white';
h.MissingDataLabel = 'Unreached';
h.ColorLimits = [5 16];

%% Torque Heatmap
figure
h = heatmap(X, Z, torque_reshape);
colormap(parula(1000))
h.NodeChildren(3).YDir = 'normal';  
h.Title = "Arm Base Torque Intensity (N-m)";
h.XLabel = "Bravo EE-Position - X (m)";
h.YLabel = "Bravo EE-Position - Z (m)";
h.XDisplayLabels = x_label_list;
h.YDisplayLabels = x_label_list;
h.GridVisible = "off";
h.MissingDataColor = 'white';
h.MissingDataLabel = 'Unreached';

%% Stiffness Heatmap
figure
h = heatmap(X, Z, stiffness);
colormap(parula(1000))
h.NodeChildren(3).YDir = 'normal';  
h.Title = "Arm Base Stiffness Intensity (N-m-deg^{-1})";
h.XLabel = "Bravo EE-Position - X (m)";
h.YLabel = "Bravo EE-Position - Z (m)";
h.XDisplayLabels = x_label_list;
h.YDisplayLabels = x_label_list;
h.GridVisible = "off";
h.MissingDataColor = 'white';
h.MissingDataLabel = 'Unreached';


%% Plot 3D arm on heatmap
% % Import robot URDF
% robot = importrobot('bravo7_planar.urdf', DataFormat='column');
% ax = axes;
% joint_angles = [3*pi/4; 3*pi/4; 3*pi/4];
% show(robot, joint_angles, "Visuals","on")
% zlim([-0.75, 1])
% xlim([-0.75, 1])
% view(0, 0)
% axis off
% ax.Color = 'none';




