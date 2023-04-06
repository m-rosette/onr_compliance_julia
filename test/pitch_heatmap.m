clear
clc

workspace_data = load('WorkspaceData/pitch_data/config_space_for_pitch_100.mat');
vehicle_pitch_data = load('WorkspaceData/pitch_data/final_pitch_disc_100.csv');

reachable_space = load("WorkspaceData\bravo_workspace_corrected.mat");
% reachable_ee_points = downsample(reachable_space.ee_points, 1500);
reachable_ee_points = reachable_space.ee_points;

% Loop through the pitch data to and any unreachable points recieve 0 pitch
for i = 1:length(vehicle_pitch_data)
    if workspace_data.configs(i, 1:3) == zeros(1, 3)
        vehicle_pitch_data(i) = 0;
    end

    % Do you need to do any filtering?
    if vehicle_pitch_data(i) > 0.25
        vehicle_pitch_data(i) = vehicle_pitch_data (i - 3);
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


% figure
% h = heatmap(X, Z, pitch_reshape);
% h.NodeChildren(3).YDir = 'normal';  
% h.Title = "Vehicle Pitch Intensity (deg)";
% h.XLabel = "Bravo EE-Position - X (m)";
% h.YLabel = "Bravo EE-Position - Z (m)";

a = reachable_ee_points(:, 1);
b = reachable_ee_points(:, 3);

ax = axes;
% scatter(a, b, '.')
k = boundary(a, b, 1);
hold on 
plot(a(k), b(k))
ax.Color = 'none';
hold on
test_point = [X(80), Z(80)]
inpolygon(test_point(1), test_point(2), a(k), b(k))
plot(test_point(1), test_point(2))




