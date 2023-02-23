% Bravo7 workspace calculation
% Marcus Rosette
% 12/08/2022

clear
clc

file = load("WorkspaceData\bravo_workspace_corrected.mat");

% Create Robot Representation
bravo = importrobot('urdf/bravo7_planar.urdf', DataFormat='column');
% bravo = importrobot('base_joint.urdf', DataFormat='column');

% Generate Trajectory and Check for Collisions
% config = file.collision_free_angles(row, 1:3);

% If the two geometries are in collision, is_self_collide is 1. Otherwise, the value is 0
% is_self_collide = checkCollision(bravo, startConfig, 'SkippedSelfCollisions', 'parent');

% Loop through a set of collision free angles
% row = 1;
% range = 10;
% for i = row:range
%     % Display the URDF
%     show(bravo, file.collision_free_angles(i, :)', 'PreservePlot', false, 'FastUpdate', true);
%     hold on
%     drawnow
% end




show(bravo, [pi; pi; pi], 'visuals','on','collision','off');
