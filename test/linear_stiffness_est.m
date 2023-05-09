%% Load data
clear
clc
clf

data = load('WorkspaceData\pitch_data\pitch_torque_stiff.mat');
pitch = data.pitch * pi / 180; % Deg to Rad
torque = data.torque;
stiffness_rot = data.stiffness_rot;

% k_rot = 101.98; % N-m/rad ------> ARM ONLY
k_rot = 181.1768; % N-m/rad ------> ARM, CAMERA, Z-FRAME

%% Process Data
num_val = 100;

r = linspace(0.4, 0.7, num_val);

theta_max = max(pitch) - min(pitch);
F_max = max(torque) ./ r;
x_max = r .* theta_max;


k = zeros(1, num_val);
for i = 1:length(r)
    % Linear stiffness mapping (see written work)
    k_linear = k_rot ./ (r(i)^2 .* cos(pitch));
    nan_logic = ~isnan(k_linear(:, 1));
    temp_k_lin = k_linear(nan_logic, 1);
    k_lin_final = sum(temp_k_lin) / length(temp_k_lin);
    k(1, i) = k_lin_final;
%     disp(k_lin_final)
end

%% Spring length opt
% % For spring length
% x_spring = linspace(0.025, 0.15, num_val);
% r_height = linspace(0.1, 0.5, num_val);
% [X, R] = meshgrid(x_spring, r_height);

% For max spring length change
r_height = linspace(0.1, 0.5, num_val);
% x_max = r_height .* theta_max;
[X, R] = meshgrid(x_max, r_height);

k_xopt = zeros(1, numel(X));
for i = 1:numel(X)

    % Linear stiffness mapping with spring length (see written work)
%     k_linear = (k_rot .* pitch) ./ (R(i) .* cos(pitch) .* (R(i) .* tan(atan(X(i) ...
%         / R(i)) + pitch) - X(i)));

    % Linear stiffness mapping
    k_linear = k_rot ./ (R(i)^2 .* cos(pitch));

    nan_logic = ~isnan(k_linear(:, 1));
    temp_k_lin = k_linear(nan_logic, 1);
    k_lin_final = sum(temp_k_lin) / length(temp_k_lin);
    k_xopt(1, i) = k_lin_final;
%     disp(k_lin_final)
end

% k_xopt_reshape = reshape(k_xopt, [10, 10]);

% Single Iteration of above Function
% theta_test = 0.2;
% x_test = 0.05;
% r_test = 0.15;
% k_test = (k_rot * theta_test) / (x_test .* cos(theta_test) * (r_test * tan(atan(x_test ...
%         / r_test) + theta_test) - x_test));


%% Plotting
% figure
plot(r, k)
hold on
plot(r, k./2)
hold on
plot(r, k./3)
hold on
plot(r, k./4)
title('Linear Stiffness vs Mount Height (w/ Z-Frame)')
xlabel('r (m)')
ylabel('k_{lin} (N/m)')
legend('1 spring', '2 springs', '3 springs', '4 springs')

figure
plot(r, x_max)
title('Max. Deflection vs Mount Height')
xlabel('r (m)')
ylabel('x_{max} - (m)')

% figure
% surf(R, X, k_xopt_reshape ./ 2)
% title('Linear stiffness estimate - 2 SPRINGS')
% xlabel('mount height - r (m)')
% ylabel('spring length - x (m)')
% zlabel('k_{lin} (N/m)')
