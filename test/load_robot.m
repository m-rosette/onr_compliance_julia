% Bravo7 workspace calculation
% Marcus Rosette
% 12/08/2022

clear
clc

% Create Robot Representation
bravo = importrobot('bravo7_planar.urdf', DataFormat='column');
% bravo = importrobot('base_joint.urdf', DataFormat='column');

% Generate Trajectory and Check for Collisions
config = [pi/2; pi; pi];

% If the two geometries are in collision, is_self_collide is 1. Otherwise, the value is 0
% is_self_collide = checkCollision(bravo, startConfig, 'SkippedSelfCollisions', 'parent');

% Get COM location and Jacobian
[com_location, com_jacobian] = centerOfMass(bravo, config);

% Display the URDF
show(bravo, config, 'visuals','on','collision','off')
hold on
scatter3(com_location(1), com_location(2), com_location(3), 10000, ".")





