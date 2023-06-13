% Trajectory genenerator 
% Marcus Rosette
% 6/8/2023

% This file takes in the discritized joint space data that was used in
% Julia and generates a time based trajectory

clc
clear

% Load robot
bravo = importrobot('bravo7.urdf', DataFormat='column');
bravo.Gravity = [0 0 -9.81];

% Load data
% ### Data Descritption - discritized for grid regions with 10 sampled
    % points within each region
data = load("WorkspaceData/pitch_data/disc_bin2_config_space_100.mat");
zero_column = zeros(length(data.configs), 1);
pi_column = pi * ones(length(data.configs), 1);
configs = [pi_column, data.configs(:, 1), data.configs(:, 2), ...
    zero_column, data.configs(:, 3), zero_column, zero_column];

numJoints = numel(homeConfiguration(bravo));

% Set up simulation parameters
tSpan = 0:0.01:0.5;
q0 = [pi, pi, 0, 0, pi, 0, 0];
qd0 = zeros(numJoints, 1);
initialState = [q0'; qd0];

% Set up joint control targets
rand_end_index = randi([1 length(configs(:, 1))], 1);
targetJointPosition = configs(rand_end_index, 1:numJoints)';
% targetJointPosition = [pi/2, pi/4, 0, 0, pi/2, 0, 0];
targetJointVelocity = zeros(numJoints, 1);
targetJointAcceleration = zeros(numJoints, 1);

% Initialize controller and controller gains
pdMotion = jointSpaceMotionModel("RigidBodyTree", bravo, "MotionType", "PDControl");
pdMotion.Kp = diag(100 * ones(1, numJoints));
pdMotion.Kd = diag(10 * ones(1, numJoints));

qDesPD = [targetJointPosition; targetJointVelocity];

% Simulate controller
[tPD,yPD] = ode15s(@(t, y) derivative(pdMotion, y, qDesPD), tSpan, initialState);

% Calculate and save end effector point
ee_points = zeros(length(tPD), 3);
for i = 1:length(tPD)
    transform = getTransform(bravo, yPD(i, 1:numJoints)', "end_effector_tip", "world");
    ee_points(i, :) = transform(1:3, 4)';
end


% %% Cartesian space traj
% gif_filename = 'testing.gif';
% plot_cartesian_traj(bravo, yPD, 2, ee_points, gif_filename)
plot_cartesian_traj(bravo, yPD, 2, ee_points)

% PD with Gravity Compensation
figure
subplot(2,1,1)
plot(tPD,yPD(:,1:numJoints))
hold all
plot(tPD,targetJointPosition*ones(1,length(tPD)),'--')
title('PD Controlled Joint Motion: Position')
xlabel('Time (s)')
ylabel('Position (rad)')
subplot(2,1,2)
plot(tPD,yPD(:,numJoints+1:end))
title('Joint Velocity')
xlabel('Time (s)')
ylabel('Velocity (rad/s)')

