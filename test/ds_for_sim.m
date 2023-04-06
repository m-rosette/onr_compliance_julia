clear
clc

% Import collision free workspace data
workspace_data = load('WorkspaceData/bravo_workspace_corrected.mat');

% Setup .mat file storage method
filename = 'WorkspaceData/pitch_data/config_space_for_pitch_100.mat';
file = matfile(filename, 'Writable', true); % Create new file instance

% Identify the number of points to discritize the space to
num_points = 100;

% Discritize the end-effector positions
x = linspace(-0.75, 1, num_points)';
z = linspace(-0.75, 1, num_points)';

tol = 0.5; % Tolerance for finding points nearby the discrete points
[X, Z] = meshgrid(x, z); % Make a grid of the points

% Initialize arrays to save data to
end_effector_points = zeros(num_points^2, 3);
configs = zeros(num_points^2, 3);
idx = zeros(num_points^2, 1);

tic % For timing the system

% Loop over all elements within the grid
for i = 1:numel(X)

    % Print runtime ETA
    if ~mod(i,100)
        elapsedTime = toc;
        estimatedTime = elapsedTime*numel(X)/i;
        ETA = estimatedTime - elapsedTime;
        disp(['ETA: ', num2str(ETA/60), ' minutes']);
    end

    % Gather the current ee_point to search
    ee_points = [X(i), Z(i)];

    % Find the closest collision free ee_point to the discrete ee_point
    closestIndex = dsearchn([workspace_data.ee_points(:, 1), workspace_data.ee_points(:, 3)], ee_points);
    
    % Check the distance tolerance of both the x and z coordinates 
    if (abs(workspace_data.ee_points(closestIndex(1), 1) - ee_points) > tol) | abs(workspace_data.ee_points(closestIndex(1), 3) - ee_points) > tol
        % Outside tolerance: Set data to zero
        end_effector_points(i, 1:3) = [0, 0, 0];
        configs(i, 1:3) = [0, 0, 0];
        idx(i, 1) = 0;
    else
        % Within tolerance: Set data to appropriate collision free point
        end_effector_points(i, 1:3) = workspace_data.ee_points(closestIndex(1), :);
        configs(i, 1:3) = workspace_data.collision_free_angles(closestIndex(1), :);
        idx(i, 1) = closestIndex(1);
    end
end

% Save data to .mat file
file.ee_points(1:num_points^2, 1:3) = end_effector_points(1:num_points^2, 1:3);
file.configs(1:num_points^2, 1:3) = configs(1:num_points^2, 1:3);
file.closestIndex(1:num_points^2, 1) = idx(1:num_points^2, 1);





