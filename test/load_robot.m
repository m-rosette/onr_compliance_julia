% Bravo7 workspace calculation
% Marcus Rosette
% 12/08/2022

clear
clc

% Create Robot Representation
bravo = importrobot('bravo7_planar.urdf', DataFormat='column');
% bravo = importrobot('base_joint.urdf', DataFormat='column');
% Generate Trajectory and Check for Collisions
% startConfig = [pi/2, pi, pi]';
% startConfig = 0;
% If the two geometries are in collision, is_self_collide is 1. Otherwise, the value is 0
% is_self_collide = checkCollision(bravo, startConfig, 'SkippedSelfCollisions', 'parent');

% Display the URDF
show(bravo, 'visuals','on','collision','off')

[com_location, com_jacobian] = centerOfMass(bravo);
% massMatrix(bravo, startConfig);




