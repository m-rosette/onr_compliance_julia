%% Load data
clear
clc
clf

data = load('WorkspaceData\pitch_data\pitch_torque_camera_vehicle_trans.mat');
% data = load('WorkspaceData\pitch_data\pitch_torque_stiff.mat');
pitch = data.pitch * pi / 180; % Deg to Rad
torque = data.torque;
% stiffness_rot = data.stiffness_rot;

% k_rot = 101.98; % N-m/rad ------> ARM ONLY
% k_rot = 181.1768; % N-m/rad ------> ARM, CAMERA, Z-FRAME
k_rot = 102.0910; % N-m/rad ------> ARM ON VEHICLE
% k_rot = 49; % testing lower values to see if the lin fit is incorrect

%% Process Data
num_val = 500;

r = linspace(0.4, 0.7, num_val);

test_pitch = 15.5 * pi / 180;

theta_max = max(pitch);
F_max = max(torque) ./ r;
x_max = r .* theta_max;

k = zeros(1, num_val);
for i = 1:length(r)
    k(1, i) = k_rot / r(i)^2 * cos(test_pitch);

%     % Linear stiffness mapping (see written work)
%     k_linear = k_rot ./ (r(i)^2 .* cos(pitch));
%     nan_logic = ~isnan(k_linear(:, 1));
%     temp_k_lin = k_linear(nan_logic, 1);
%     k_lin_final = sum(temp_k_lin) / length(temp_k_lin);
%     k(1, i) = k_lin_final;
%     % Testing new equation (difference between theta_max and the pitch)
%     k_linear = (k_rot * theta_max ./ (r(i)^2 .* pitch .* cos(pitch)) - (k_rot ./ (r(i)^2 .* cos(pitch))));
%     nan_logic = ~isnan(k_linear(:, 1));
%     temp_k_lin = k_linear(nan_logic, 1);
%     k_lin_final = sum(temp_k_lin) / length(temp_k_lin);
%     k(1, i) = k_lin_final;
end

% Unit conversion 
k = k ./ 175.126835;    % N/m to lbs/in
r = r .* 39.37;         % m to in
x_max = x_max .* 39.37; % m to in
F_max_torque = F_max .* 0.2248089431; % N to lbs
F_max_spring = x_max .* k;
num_springs = 2;

ideal_params_idx = 350;
disp('Parameters:')
disp('number of springs')
disp(num_springs)
disp('spring constant (lbs/in): ')
disp(k(ideal_params_idx) / num_springs)
disp('mount height (in): ')
disp(r(ideal_params_idx))
disp('max force (lbs): ')
disp(F_max_spring(ideal_params_idx) / num_springs)
disp('max displacement (in): ')
disp(x_max(ideal_params_idx))

%% Plotting
% figure
subplot(1, 2, 1)
plot(r, k)
hold on
plot(r, k./2)
hold on
plot(r, k./3)
hold on
plot(r, k./4)
title('Linear Stiffness vs Mount Height (w/ Z-Frame)')
% xlabel('r (m)')
% ylabel('k_{lin} (N/m)')
xlabel('r (in)')
ylabel('k_{lin} (lbs/in)')
legend('1 spring', '2 springs', '3 springs', '4 springs')
grid on

% figure
subplot(1, 2, 2)
plot(r, x_max)
hold on
plot(r, F_max_spring ./ 2)
hold on
plot(r, F_max_torque ./ 2)
title('Max. Deflection & Force vs Mount Height', 'For 2 Springs')
% xlabel('r (m)')
% ylabel('x_{max} - (m)')
xlabel('r (in)')
ylabel('Max. Deflection & Force')
legend('spring deflection (in)', 'max. force [k*x] (lbs)', 'max. force [T/r] (lbs)')
grid on
hold off
