% Bravo7 workspace calculation
% Marcus Rosette
% 12/08/2022


% Create Robot Representation
% bravo = importrobot('bravo7.urdf');
% bravo = importrobot('iiwa14.urdf');
% bravo.DataFormat = 'column';

two_link_arm = importrobot('two_link_arm.urdf', DataFormat='column');
show(two_link_arm, 'visuals','off','collision','on')

% show(bravo,'visuals','off','collision','on')

% Generate Trajectory and Check for Collisions
% startConfig = [pi, 0, pi, 0, pi, 0, 0, 0, 0]';
% startConfig = [0 -pi/4 pi 3*pi/2 0 -pi/2 pi/8]';
    
% [is_self_collide, self_sep_dist, self_wit_pts] = checkCollision(bravo, startConfig, 'SkippedSelfCollisions', 'parent')
% If the two geometries are in collision, is_self_collide is 1. Otherwise, the value is 0