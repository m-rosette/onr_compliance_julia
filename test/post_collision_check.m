
% Post process FK and Collision Check
% 02/06/2023
% Marcus Rosette

oldfile = load("Collision Data Storage\bravo_workspace - Copy.mat");
robot = importrobot('bravo7_planar.urdf', DataFormat='column');

% Setup .mat file storage method
filename = 'post_process_collision_check.mat';
if isfile(filename)     % Remove the preexisting file if exists
    delete(filename)
end
newfile = matfile(filename, 'Writable', true); % Create new file instance

tic
for i = 1:length(oldfile.collision_free_angles)
    if ~mod(i,1000)
        disp(i/length(oldfile.collision_free_angles));
        disp(toc)
    end
    collision_check = checkCollision(robot, oldfile.collision_free_angles(i, :)', 'Exhaustive', 'on', 'SkippedSelfCollisions','parent');
    if collision_check
        newfile.collision_check(i, 1) = 1;
    end
end
toc
newfile.ElapsedTime = toc; 
disp("Calculation complete")