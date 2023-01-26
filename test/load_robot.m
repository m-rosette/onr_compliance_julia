% Bravo7 workspace calculation
% Marcus Rosette
% 12/08/2022

clear
clc

% Create Robot Representation
bravo = importrobot('bravo7_planar.urdf', DataFormat='column');

% Generate Trajectory and Check for Collisions
startConfig = [pi, pi, pi]';

% If the two geometries are in collision, is_self_collide is 1. Otherwise, the value is 0
is_self_collide = checkCollision(bravo, startConfig, 'SkippedSelfCollisions', 'parent');

% Display the URDF
show(bravo, startConfig,'visuals','on','collision','off')