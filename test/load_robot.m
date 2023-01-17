% Bravo7 workspace calculation
% Marcus Rosette
% 12/08/2022

clear
clc

% Create Robot Representation
bravo = importrobot('bravo7.urdf', DataFormat='column');
% bravo = importrobot('iiwa14.urdf', DataFormat='column');

% jaws = importrobot('jaws_test.urdf', DataFormat='column');



% Generate Trajectory and Check for Collisions
startConfig = [pi, 0, pi, 0, pi, 0, 0, 0]'; % For bravo arm 
% startConfig = [0 -pi/4 pi 3*pi/2 0 -pi/2 pi/8]'; % For iiwa arm 

% startConfig = [pi/2, pi/2]'; % For jaws
    
[is_self_collide, self_sep_dist, self_wit_pts] = checkCollision(bravo, startConfig, 'SkippedSelfCollisions', 'parent');
% If the two geometries are in collision, is_self_collide is 1. Otherwise, the value is 0

show(bravo, startConfig,'visuals','on','collision','off')