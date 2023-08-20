clc
clear

% Gather and reshape pitch and torque data
sim_data = load('WorkspaceData\pitch_data\pitch_torque_camera_vehicle_trans.mat');
pitch = sim_data.pitch;
torque = sim_data.torque;
com = sim_data.com;
config = sim_data.config;
ee_point = sim_data.ee_point;

disc_points = 10;
com_to_test = linspace(0.0254, max(com), disc_points);
com_sim_eq = zeros(1, disc_points);
pitch_sim = zeros(1, disc_points);

tol = 0.005;
for i = 1:length(com_to_test)
    logic_list = (com >= com_to_test(i) - tol) & (com <= com_to_test(i) + tol);
    idx = find(logic_list);
    pitch_sim(i) = sum(pitch(idx)) / length(idx);
end

% % Finds the closes value in com list to the com_to_test list
% for i = 1:length(com_to_test)
%     [minDistance, indexOfMin] = min(abs(com-com_to_test(i)));
%     com_sim_eq(i) = com(indexOfMin);
%     pitch_sim(i) = pitch(indexOfMin);
% end

com_imperial = com_to_test * 39.37; % m to in

% % Can find a value within a range
% test_idx = (com>=0.3) & (com<=0.32);
% range_idx = find(test_idx);

