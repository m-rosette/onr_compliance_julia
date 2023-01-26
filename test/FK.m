% Forward Kinematics
% Marcus Rosette
% 01/25/2023

clear
clc

% WORK STARTED IN JULIA "workspace_calcs.jl"
% Need to adapt that code to work for all joint combinations

%% Parameters
% Parameters (from Reach Robotics Bravo7 manual)---------------------------------
% Changed joint progression - J1=Base revolute joint,..., J7=end effector opening

% Joint Limits 
j1 = Inf; % [deg] - Continuous revolute
j2 = 180; % [deg] - Elbow
j3 = 180; % [deg] - Elbow
j4 = Inf; % [deg] - Continuous revolute
j5 = 180; % [deg] - Elbow
j6 = Inf; % [deg] - Continuous revolute
j7 = 118; % [mm]  - Jaw opening

% Body Lengths
a = 365; % [mm] - j5 to end effector
b = 160; % [mm] - j3 to j5
c = 292; % [mm] - j2 to j3
d = 159; % [mm] - table top to j2
e = 92;  % [mm] - max OD
f = 82;  % [mm] - min OD (at tip of wrist) 

%% Forward Kinematics

M = [0 0 1 e;
    0 1 0 0;
    -1 0 0 -(a + b + c -d);
    0 0 0 1];

% Screw Axes - S (base) frame
S1 = [0, 0, 1, 0, 0, 0];     % Base revolute joint
S2 = [0, 1, 0, -d, 0, e/2];
S3 = [0, 1, 0, c-d, 0, e];
S4 = [0, 0, 1, 0, -e, 0];
S5 = [0, 1, 0, c-d, 0, e];
S6 = [0, 0, 1, 0, -e, 0];
S7 = [0, 0, 0, 0, 0, 1];     % End-effector prismatic joint 

screw_axes = [S1; S2; S3; S4; S5; S6; S7];  % Collecting all screw axes in one array
% screw_axes = [S2; S3; S5];

% --------------------- NEED TO MAKE THE PROD OF EXP AND MAT EXP FUNCTION CAPABLE OF
% HANDLING LARGE THETA DATASET -----------------------------------------

% Need to implement all screw axesin prod of exp function
% Meaning that extra columns of zeros are needed in theta_list for all
% joints that are "fixed"

% when passing the theta list into prod_of_exp need to pass in one row at a
% time

% Load in Data File
file = load("Collision Data Storage\thetas_test_Jan26.mat");

file.all_joint_thetas = [zeros(length(file.collision_free_angles), 1) file.collision_free_angles(:, 1:2) ...
    zeros(length(file.collision_free_angles), 1) file.collision_free_angles(:, 3) zeros(length(file.collision_free_angles), 1) ...
    zeros(length(file.collision_free_angles), 1)];



% Loop through all prod of exp from file and append end effector points
tic
for theta_row = 1:length(file.all_joint_thetas)
    if ~mod(theta_row,100)
        elapsedTime = toc;
        estimatedTime = elapsedTime*length(file.all_joint_thetas)/theta_row;
        ETA = estimatedTime - elapsedTime;
        disp(['ETA: ',num2str(ETA/60),' minutes']);
    end


    file.ee_points(theta_row, :) = product_of_exp(M, screw_axes, file.all_joint_thetas(theta_row, :));
end
toc









