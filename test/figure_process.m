% Experimental Pitch Figure Generation
% Marcus Rosette
% 7/10/2023

clc
clear
clf

%% Load Data
data = load('WorkspaceData\pitch_data\exp_pitch.mat');
com = data.com;
hardware_pitch = data.hardware_pitch;
sim_pitch = data.sim_pitch;

%% Region of Interest (ROI) | idx: 9-19
com_roi = com(9:19, 1);
hardware_roi = hardware_pitch(9:19, 1);
sim_roi = sim_pitch(9:19, 1);

%% Plots
% Plot raw data
plot(com, hardware_pitch)
hold on 
plot(com, sim_pitch)
hold on

% Plot box around ROI
xlim([0, 21]); ylim([0, 21]) % Adjust axes limits
xmin = com_roi(1, 1); xmax = com_roi(end, 1); % Identify x-roi bounds
ymin = hardware_roi(1, 1); ymax = hardware_roi(end, 1); % Identify y-roi bounds
plot([xmin, xmax], [ymin, ymax], '.') % Display roi boinds
dim = [0.445, 0.44, 0.353, 0.41]; % Find scaled dim for rectangle
annotation('rectangle', dim, 'FaceColor','green','FaceAlpha',.15)
annotation('textbox',dim,'String','ROI','FitBoxToText','on');

% Title the figure
title('Experimental Comparison')
xlabel('COM (in)')
ylabel('Pitch (deg)')
legend('Hardware', 'Simulation', Location='northwest')