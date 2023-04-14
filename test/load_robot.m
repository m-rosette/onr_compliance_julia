% Bravo7 workspace calculation
% Marcus Rosette
% 12/08/2022

clear
clc

file = load("WorkspaceData\bravo_workspace_corrected.mat");

% Create Robot Representation
bravo = importrobot('bravo7_planar.urdf', DataFormat='column');
% test = importrobot('urdf/three_link_arm.urdf', DataFormat='column');
% bravo = importrobot('base_joint.urdf', DataFormat='column');

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

row = 94892;
config = file.collision_free_angles(row, :)
% config = [0; 0; 0]';
show(bravo, config');

julia_config = [config(1), config(2)-pi, config(3)-pi]