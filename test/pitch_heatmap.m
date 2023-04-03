workspace_data = load('WorkspaceData/bravo_workspace_corrected.mat');
vehicle_pitch_data = load('WorkspaceData/pitch_data_copy.mat');

ds_angles = downsample(workspace_data.collision_free_angles, 8750);
ds_ee_points = downsample(workspace_data.ee_points, 8750);

X = ds_ee_points(:, 1);
Z = ds_ee_points(:, 3);
pitch = vehicle_pitch_data.ds_collision_free_configs;
coords = table(X, Z, pitch);

h = heatmap(coords, 'X', 'Z', 'ColorVariable', 'pitch');

