%% Process Initial Dataset
% clear
% clc

% Load pitch data
vehicle_pitch_data = load('WorkspaceData/pitch_data/bin_pitch_data.csv');

% Convert pitch data to degrees
vehicle_pitch_data = vehicle_pitch_data(:) * (180 / pi);

% % Do you need to do any filtering?
pitch_avg = zeros(10000, 1);
num_neighbors = 5;
idx_count = 0;
for i = 1:length(pitch_avg)
%     if vehicle_pitch_data(i) == 0
%         vehicle_pitch_data(i) = NaN;
%     end
    if isnan(vehicle_pitch_data(idx_count + 1))
        pitch_avg(i) = NaN;
        idx_count = idx_count + 1;
    else
        pitch_avg(i) = sum(vehicle_pitch_data(idx_count:idx_count + num_neighbors - 1)) / num_neighbors;
        idx_count = idx_count + num_neighbors;
    end
    disp(idx_count)
end

%% Processes Heatmap
% Gather data for the X and Y positions of the heatmap
num_points = 100;
X = linspace(-0.75, 1, num_points)';
Z = linspace(-0.75, 1, num_points)';

% Reshape the column pitch data to match the table format of heatmap
pitch_reshape = reshape(pitch_avg, [num_points, num_points]);

% Initialize the heatmap and it's labels
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

% Import robot URDF
robot = importrobot('bravo7_planar.urdf', DataFormat='column');


% figure
h = heatmap(X, Z, pitch_reshape);
colormap(parula(1000))
h.NodeChildren(3).YDir = 'normal';  
h.Title = "Vehicle Pitch Intensity (deg)";
h.XLabel = "Bravo EE-Position - X (m)";
h.YLabel = "Bravo EE-Position - Z (m)";
h.XDisplayLabels = x_label_list;
h.YDisplayLabels = x_label_list;
h.GridVisible = "off";
% h.MissingDataColor = [0.9, 0.9, 0.9];
h.MissingDataColor = 'white';
h.MissingDataLabel = 'Unreached';
h.ColorLimits = [5 16];


% ax = axes;
% joint_angles = [3*pi/4; 3*pi/4; 3*pi/4];
% show(robot, joint_angles, "Visuals","on")
% zlim([-0.75, 1])
% xlim([-0.75, 1])
% view(0, 0)
% axis off
% ax.Color = 'none';




