%% Load Initial Dataset
clear
clc

% Load data
data = load('WorkspaceData/pitch_data/pitch_torque_camera_vehicle_trans.mat');

pitch = data.pitch;
torque = data.torque;
% stiffness_rot = data.stiffness_rot;


%% Processes Heatmap
% Gather data for the X and Y positions of the heatmap
num_points = 100;
X = linspace(-0.75, 1, num_points)';
Z = linspace(-0.75, 1, num_points)';

% Reshape the column data to match the table size of heatmap
pitch_reshape = reshape(pitch, [num_points, num_points]);
torque_reshape = reshape(torque, [num_points, num_points]);
% stiffness_reshape = reshape(stiffness_rot, [num_points, num_points]);

% There was a lone zero in the data in the top right corner (not critical)
if torque_reshape(100, 100) == 0
    torque_reshape(100, 100) = NaN;
end


%% Plotting Heatmaps
% Initialize the heatmap axes labels
x_label_list = cell(1, num_points);
z_label_list = cell(1, num_points);
for i = 1:num_points
    if ~mod(i, 5)
        x_label_list{i} = string(round(X(i), 2));
        z_label_list{i} = string(round(Z(i), 2));
    else
        x_label_list{i} = '';
        z_label_list{i} = '';
    end
end

%% Pitch Heatmap
% figure
subplot(1, 2, 1)
h = heatmap(X, Z, pitch_reshape);
colormap(parula(1000))
h.NodeChildren(3).YDir = 'normal';  
h.Title = "Vehicle Pitch Intensity w/ Camera (deg)";
h.XLabel = "Bravo EE-Position - X (m)";
h.YLabel = "Bravo EE-Position - Z (m)";
h.XDisplayLabels = x_label_list;
h.YDisplayLabels = z_label_list;
h.GridVisible = "off";
h.MissingDataColor = 'white';
h.MissingDataLabel = 'Unreached';
% h.ColorLimits = [5 16];

%% Torque Heatmap
% figure
subplot(1, 2, 2)
h = heatmap(X, Z, torque_reshape);
colormap(parula(1000))
h.NodeChildren(3).YDir = 'normal';  
h.Title = "Arm Base Torque Intensity w/ Z-Frame (N-m)";
h.XLabel = "Bravo EE-Position - X (m)";
h.YLabel = "Bravo EE-Position - Z (m)";
h.XDisplayLabels = x_label_list;
h.YDisplayLabels = z_label_list;
h.GridVisible = "off";
h.MissingDataColor = 'white';
h.MissingDataLabel = 'Unreached';

% %% Stiffness Heatmap
% % figure
% subplot(1, 3, 3)
% h = heatmap(X, Z, stiffness_reshape);
% colormap(parula(1000))
% h.NodeChildren(3).YDir = 'normal';  
% h.Title = "Arm Base Stiffness Intensity (N-m-deg^{-1})";
% h.XLabel = "Bravo EE-Position - X (m)";
% h.YLabel = "Bravo EE-Position - Z (m)";
% h.XDisplayLabels = x_label_list;
% h.YDisplayLabels = z_label_list;
% h.GridVisible = "off";
% h.MissingDataColor = 'white';
% h.MissingDataLabel = 'Unreached';
% 

%% Plot Torque vs. Degrees (Stiffness)
nan_logic = ~isnan(pitch(:, 1));
pitch_new = pitch(nan_logic, 1);
torque_new = torque(nan_logic, 1);

figure
s = scatter(pitch, torque, 'Marker','.');
p = polyfit(pitch_new, torque_new, 1);
y_est = polyval(p, pitch_new);
hold on
plot(pitch_new, y_est, 'r', 'LineWidth', 2)
slope = p(1) * 180 / pi; % Nm/deg to Nm/rad

SStot = sum((torque_new-mean(torque_new)).^2);                    % Total Sum-Of-Squares
SSres = sum((torque_new-y_est).^2);                       % Residual Sum-Of-Squares
Rsq = 1-SSres/SStot;                            % R^2

dim = [.15 .5 .3 .3];
str = 'R^{2} = 0.879';
annotation('textbox',dim,'String',str,'FitBoxToText','on', 'FontSize', 9);

xlabel('Vehicle Pitch (deg)')
ylabel('Arm Base Torque (N-m)')
title("Arm + Z-Frame + Vehicle Torque vs Pitch")
legend('sim data', 'linear fit', Location='northwest')
hold off

%% Plot 3D arm on heatmap
% % Import robot URDF
% robot = importrobot('bravo7_planar.urdf', DataFormat='column');
% ax = axes;
% joint_angles = [3*pi/4; 3*pi/4; 3*pi/4];
% show(robot, joint_angles, "Visuals","on")
% zlim([-0.75, 1])
% xlim([-0.75, 1])
% view(0, 0)
% axis off
% ax.Color = 'none';




