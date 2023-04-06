clear
clc

workspace_data = load('WorkspaceData/pitch_data/new_space_100.mat');
vehicle_pitch_data = load('WorkspaceData/pitch_data/new_final_pitch_disc_100.csv');

reachable_space = load("WorkspaceData\bravo_workspace_corrected.mat");
reachable_ee_points = reachable_space.ee_points;

% Do you need to do any filtering?
for i = 1:length(vehicle_pitch_data)
    if vehicle_pitch_data(i) > 0.25 | vehicle_pitch_data(i) < 0.05 && vehicle_pitch_data(i) > 0
%         vehicle_pitch_data(i) = vehicle_pitch_data(i - 1);
        vehicle_pitch_data(i) = 0.15;
    end
end

vehicle_pitch_data = vehicle_pitch_data(:) * (180 / pi);

% Gather data for the X and Y positions of the heatmap
% X = workspace_data.ee_points(:, 1);
% Z = workspace_data.ee_points(:, 3);
% Original discritized space
num_points = 100;
X = linspace(-0.75, 1, num_points)';
Z = linspace(-0.75, 1, num_points)';

% Reshape the column pitch data to match the table format of heatmap
pitch_reshape = reshape(vehicle_pitch_data, [num_points, num_points]);

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


figure

h = heatmap(X, Z, pitch_reshape);
h.NodeChildren(3).YDir = 'normal';  
h.Title = "Vehicle Pitch Intensity (deg)";
h.XLabel = "Bravo EE-Position - X (m)";
h.YLabel = "Bravo EE-Position - Z (m)";
h.XDisplayLabels = x_label_list;
h.YDisplayLabels = x_label_list;

ax = axes;
joint_angles = [3*pi/4; 3*pi/4; 3*pi/4];
show(robot, joint_angles, "Visuals","on")
zlim([-0.75, 1])
xlim([-0.75, 1])
view(0, 0)
axis off
ax.Color = 'none';




