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

% screw_axes = [S1; S2; S3; S4; S5; S6; S7];  % Collecting all screw axes in one array
screw_axes = [S2; S3; S5];

% --------------------- NEED TO MAKE THE PROD OF EXP AND MAT EXP FUNCTION CAPABLE OF
% HANDLING LARGE THETA DATASET -----------------------------------------

function T = matrix_exponential(screw_axis, theta)
    % """ Calculate the homogeneous transformation for a set of exponential coordinates
    % input: screw axis
    % input: theta (rads)
    % return: T - homogeneous transformation matrix
    % """
    T = zeros(4); % Initialize transformation matrix

    w_skew = [0 -screw_axis(3) screw_axis(2); screw_axis(3) 0 -screw_axis(1); -screw_axis(2) screw_axis(1) 0];
    v = [screw_axis(4); screw_axis(5); screw_axis(6)];
    R = eye(3) + sin(theta) * w_skew + (1 - cos(theta)) * w_skew^2;
    p = (eye(3) * theta + (1 - cos(theta)) * w_skew + (theta - sin(theta)) * w_skew^2) * v;
    T(1:3, 1:3) = R;
    T(1:3, 4) = p;
    T(4, :) = [0 0 0 1];
end


function T_prod_exp = product_of_exp(M, screw_axes_list, theta_list)
    % """ Product of Exponentials
    % input: zero configuration transformation matrix
    % input: List of all screw axis in arm
    % input: List of joint angles
    % return: Updated transformation matrix
    % """
    % Check list length compatibility
    if length(screw_axes_list) ~= length(theta_list)
        error("Screw axis list should be the same length as theta list")
    end

    T_prod_exp = M;
    for i = 1:lenth(screw_axes_list)
        T_temp = matrix_exponential(screw_axes_list(i), theta_list(i)) * T_prod_exp;
        T_prod_exp = T_temp;
    end
end