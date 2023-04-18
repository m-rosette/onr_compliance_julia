clear
clc

% Import collision free workspace data
disp("Loading workspace data...")
workspace_data = load('WorkspaceData/bravo_workspace_corrected.mat');

% Setup .mat file storage method
filename = 'WorkspaceData/pitch_data/disc_bin_config_space_100.mat';
file = matfile(filename, 'Writable', true); % Create new file instance
disp('done')
disp(' ')

% Identify the boundary around collision free end effector points
ax = workspace_data.ee_points(:, 1);
az = workspace_data.ee_points(:, 3);
disp("Computing workspace boundary...")
k = boundary(ax, az, 1);
disp('done')
disp(' ')

% Identify the number of points to discritize the space to
num_points = 100;

% Discritize the end-effector positions
x = linspace(-0.75, 1, num_points)';
z = linspace(-0.75, 1, num_points)';

% tol = 0.5; % Tolerance for finding points nearby the discrete points
[X, Z] = meshgrid(x, z); % Make a grid of the points

% Initialize arrays to save data to
idx_exists = zeros(num_points^2, 1);
num_neighbors = 5;

temp_ee_point = zeros(100, 3);
temp_configs = zeros(100, 3);
temp_idx = zeros(100, 1);
temp_start = 1;

tic % For timing the system

% Loop over all elements within the grid
disp('Iterating over discretized grid...')
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
    
    % Check if point is within the collision free boundary 
    if inpolygon(ee_points(1), ee_points(2), ax(k), az(k))
        % Find the closest collision free ee_point to the discrete ee_point
        [closestIndex, dist] = knnsearch([ax, az], ee_points, 'K', num_neighbors);

        idx_exists(i, 1) = 1;

        for point = 1:length(closestIndex)
            % Set data to appropriate collision free point
            temp_ee_point(temp_start + point - 1, 1:3) = workspace_data.ee_points(closestIndex(point), :);
            temp_configs(temp_start + point - 1, 1:3) = workspace_data.collision_free_angles(closestIndex(point), :);
            temp_idx(temp_start + point - 1, 1) = closestIndex(point);    
        end

        % Increment matrix saving index
        temp_start = temp_start + num_neighbors;

    else
        % Outside collision free boundary: Set data to zero
        idx_exists(i, 1) = 0;
    end
end

%% Processing Bin Data

config_idx = 1;
% test_configs = zeros(500, 3);
% test_ee_points = zeros(500, 3);
% test_idx = zeros(500, 1);

q = 1;

for i = 1:numel(X)
    if idx_exists(i) == 1
        test_configs(config_idx:config_idx+num_neighbors-1, 1:3) = temp_configs(q:q+num_neighbors-1, 1:3);
        test_ee_points(config_idx:config_idx+num_neighbors-1, 1:3) = temp_ee_point(q:q+num_neighbors-1, 1:3);
        test_idx(config_idx:config_idx+num_neighbors-1, 1) = temp_idx(q:q+num_neighbors-1, 1);
        q = q + num_neighbors;
        config_idx = config_idx + num_neighbors;
    else
        test_configs(config_idx, 1:3) = [0, 0, 0];
        test_ee_points(config_idx, 1:3) = [0, 0, 0];
        test_idx(config_idx, 1) = NaN;
        config_idx = config_idx + 1;
    end
end

disp(' ')
disp('add in null space')
disp(' ')

% Save data to .mat file
disp('Saving to .mat file...')
file.ee_points(1:length(test_idx), 1:3) = test_ee_points;
file.configs(1:length(test_idx), 1:3) = test_configs;
file.closestIndex(1:length(test_idx), 1) = test_idx;
disp('done')
disp(' ')





