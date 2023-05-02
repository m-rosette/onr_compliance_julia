% Bravo7 workspace calculation
% Marcus Rosette
% 12/08/2022

clear
clc

file = load("WorkspaceData\bravo_workspace_corrected.mat");

data = load('WorkspaceData\pitch_data\disc_bin2_config_space_100.mat');
configurations = data.configs;

% Create Robot Representation
bravo = importrobot('bravo7_planar.urdf', DataFormat='column');
% arm_vehicle = importrobot('arm_vehicle.urdf', DataFormat='column');

% Generate Trajectory and Check for Collisions
% config = file.collision_free_angles(row, 1:3);

% If the two geometries are in collision, is_self_collide is 1. Otherwise, the value is 0
% is_self_collide = checkCollision(bravo, startConfig, 'SkippedSelfCollisions', 'parent');

% Loop through a set of collision free angles
% row = 800000; 
% range = row + 100;
% for i = row:range
%     % Display the URDF
%     show(bravo, file.collision_free_angles(i, :)', 'PreservePlot', false, 'FastUpdate', true);
%     drawnow
% end

% config = [1.806, 1.3614, 0.0785];
row = 10000;
% config = configurations(row, :);
config = [1.5707, pi, pi];
show(bravo, config');
grid off
view(0, 0)
lightangle(0, 60)

% julia_config = [config(1), config(2)-pi, config(3)-pi]