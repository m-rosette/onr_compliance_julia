clear 
clc

%% Initial parameters
% Calibration 1 - 2 data points
% volt_min = -4.262; % volts
% volt_max = -1.48; % volts
% extension_min = 0; % inches
% extension_max = 11.25; % inches
% sensor_mount_height = 27.75; % inches

% Calibration 2 - 6 data points
sensor_mount_height = 700; % mm
extension_rest = 93; % mm
voltage_measured = [-4.2675, -3.521, -3.232, -2.9725, -2.4455, -0.4675]; % volts
extension_measured = [93, 173, 204, 232, 286, 495] - extension_rest; % mm

% Select which arm configuration to process/plot
config_num = 1;
waves = true;
num_joints = 7;

%% Load data
% % Load Hinsdale results
% data = readtable('WorkspaceData\pitch_data\logs\hinsdale_config_1_redo.log', 'FileType', 'text');

% Load used configs
configs_file = load("WorkspaceData\pitch_data\hardware_configs_actuated_only.mat");
configs = configs_file.configs;

% Unpack used configs
zero_column = zeros(length(configs), 1);
pi_column = pi * ones(length(configs), 1);

% Save used configs in a 1x7 array for urdf
configs_full = [pi_column, configs(:, 1), configs(:, 2), ...
    zero_column, configs(:, 3), zero_column, zero_column];

% Load urdf of robot
bravo = importrobot('bravo7.urdf', DataFormat='column');

% Load prediction
pitch_pred_file = load('WorkspaceData\pitch_data\arm_camera_hardware_pitch_data.csv');
pitch_pred = rad2deg(pitch_pred_file);

%% Parse hinsdale results
num_exp = 3;
end_var = 291;

for exp = 1:num_exp
    % Load Hinsdale results
    if exp == 1
        data = readtable('WorkspaceData\pitch_data\logs\hinsdale_config_1_redo.log', 'FileType', 'text');
    elseif exp == 2
        data = readtable('WorkspaceData\pitch_data\logs\hinsdale_config_1_2_redo.log', 'FileType', 'text');
    else
        data = readtable('WorkspaceData\pitch_data\logs\hinsdale_config_1_3_redo.log', 'FileType', 'text');
    end

    % Remove unwanted symbols and convert strings to doubles
    for i = 1:height(data)
        data.Var2(i) = erase(data.Var2(i), '[');
        data.Var2{i} = str2double(data.Var2{i});
    
        data.Var8(i) = erase(data.Var8(i), ']');
        data.Var8{i} = str2double(data.Var8{i});
    
        data.Var9(i) = erase(data.Var9(i), '[');
        data.Var9(i) = erase(data.Var9(i), ']');
        data.Var9{i} = str2double(data.Var9{i});
    end
    
    data.Var2 = cell2mat(data.Var2);
    data.Var8 = cell2mat(data.Var8);
    data.Var9 = cell2mat(data.Var9);
    
    % Save data arrays to corresponding variable names
    timestamp(1:end_var, exp) = data.Var1(1:end_var);
    joint_pos(1:end_var, 1:7) = [data.Var2(1:end_var), ...
        data.Var3(1:end_var), data.Var4(1:end_var), data.Var5(1:end_var), ...
        data.Var6(1:end_var), data.Var7(1:end_var), data.Var8(1:end_var)];
    voltage(1:end_var, exp) = data.Var9(1:end_var);
end


if waves
    % Load Hinsdale results
    data = readtable('WorkspaceData\pitch_data\logs\video_config_1.log', 'FileType', 'text');

    % Remove unwanted symbols and convert strings to doubles
    for i = 1:height(data)
        data.Var2(i) = erase(data.Var2(i), '[');
        data.Var2{i} = str2double(data.Var2{i});
    
        data.Var8(i) = erase(data.Var8(i), ']');
        data.Var8{i} = str2double(data.Var8{i});
    
        data.Var9(i) = erase(data.Var9(i), '[');
        data.Var9(i) = erase(data.Var9(i), ']');
        data.Var9{i} = str2double(data.Var9{i});
    end
    
    data.Var2 = cell2mat(data.Var2);
    data.Var8 = cell2mat(data.Var8);
    data.Var9 = cell2mat(data.Var9);
    
    % Save data arrays to corresponding variable names
    timestamp_waves(1:end_var, 1) = data.Var1(1:end_var);
    joint_pos_waves(1:end_var, 1:7) = [data.Var2(1:end_var), ... 
        data.Var3(1:end_var), data.Var4(1:end_var), data.Var5(1:end_var), ...
        data.Var6(1:end_var), data.Var7(1:end_var), data.Var8(1:end_var)];
    voltage_waves(1:end_var, 1) = data.Var9(1:end_var);
end

%% Map from voltage to pitch
linear_map = interp1(voltage_measured, extension_measured, voltage);
pitch = rad2deg(linear_map / sensor_mount_height);
pitch(1, :) = 11.2;
pitch(2, :) = 11.2;

linear_map_waves = interp1(voltage_measured, extension_measured, voltage_waves);
pitch_waves = rad2deg(linear_map_waves / sensor_mount_height);
pitch_waves(1) = 6.591;
pitch_waves(2) = 6.591;

%% Get statistics
pitch_std = std(pitch, 0, 2);
pitch_avg = mean(pitch, 2);

%% Calculate and save end-effector location
ee_points = zeros(end_var, 3);
joint_pos_fl = fliplr(joint_pos);
for i = 1:end_var
    transform = getTransform(bravo, joint_pos_fl(i, 1:num_joints)', "end_effector_tip", "world");
    ee_points(i, :) = transform(1:3, 4)';
end

ee_points(1, :) = [0.253, 0.0003, -0.0728];

%% Plot data
% Preplot processing
time_arr_1 = timestamp(:, 1) - timestamp(1, 1);
X = [time_arr_1; flip(time_arr_1)];
y1 = pitch_avg + pitch_std;
y2 = flip(pitch_avg - pitch_std);
Y = [y1; y2];

% Plot subplots
subplot(1, 3, 1)
show(bravo, configs_full(config_num + 1, :)', "Frames","off");
hold on
plot3(ee_points(:, 1), ee_points(:, 2), ee_points(:, 3))
view(0, 0)
lightangle(0, 60)
xlim([-0.15, 1])
ylim([-0.1, 0.1])
zlim([-0.15, 1])
xlabel('X (m)')
zlabel('Z (m)')
title('Bravo Trajectory')

subplot(1, 3, 2)
plot(time_arr_1, pitch_avg, 'black')
hold on
patch(X, Y, 'black', 'FaceAlpha', 0.3, 'EdgeColor', 'none')
hold on
plot(time_arr_1, repelem(pitch_pred(config_num + 1), end_var), "Color", 'r', 'LineStyle', '--')
ylim([5 25])
grid on
xlabel('Time (s)')
ylabel('Pitch (deg)')
title('Pitch Performance')
legend('Avg', 'Std', 'Prediction', Location='northwest')

subplot(1, 3, 3)
plot(time_arr_1, pitch_avg, 'black')
hold on
patch(X, Y, 'black', 'FaceAlpha', 0.3, 'EdgeColor', 'none')
hold on
plot(time_arr_1, repelem(pitch_pred(config_num + 1), end_var), "Color", 'r', 'LineStyle', '--')
hold on
plot(time_arr_1, pitch_waves, 'b')
ylim([5 25])
grid on
xlabel('Time (s)')
ylabel('Pitch (deg)')
title('Pitch Performance - Waves')
legend('Avg', 'Std', 'Prediction', 'Waves', Location='northwest')

g = gcf;
g.WindowState = 'maximized';



