%% Load data
clear
clc

data = load('WorkspaceData\pitch_data\pitch_torque_stiff.mat');
pitch = data.pitch * pi / 180; % Deg to Rad
torque = data.torque;
stiffness_rot = data.stiffness_rot;

k_rot = 101.98; % N-m/rad

%% Process Data
num_val = 10;

r = linspace(0.1, 0.5, num_val);

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




%% Plotting
figure
plot(r, k)
hold on
plot(r, k./2)
hold on
plot(r, k./3)
hold on
plot(r, k./4)
title('Linear Stiffness vs Mount Height')
xlabel('r (m)')
ylabel('k_{lin} (N/m)')
legend('1 spring', '2 springs', '3 springs', '4 springs')

figure
plot(r, x_max)
title('Max. Deflection vs Mount Height')
xlabel('r (m)')
ylabel('x_{max} - (m)')