% Plotting Forward Kinematics
% Marcus Rosette
% 01/27/2023

file = load("Collision Data Storage\planar_arm_workspace_data - Copy.mat");

% figure;
% % scatter3(file.file.ee_points(:, 1), file.file.ee_points(:, 2), file.file.ee_points(:, 3), ".");
% scatter(file.file.ee_points(:, 1), file.file.ee_points(:, 3), ".");
% title("Possible Bravo End-Effector Points")
% xlabel("x-axis (mm)")
% ylabel("z-axis (mm)")
% grid on
% set(gca, 'XDir','reverse')


% Import robot URDF
robot = importrobot('bravo7_planar.urdf', DataFormat='column');

% Found a repeated collision at index 7415
% More collisions found at index 13051
% More at index 501058
% IT SEEMS LIKE THE END EFFECTOR MESH IS NOT INCLUEDED IN THE COLLISION CHECK



% figure;
% ax = show(robot, [0; 0; 0]);
% hold all
% for i = 500000:length(file.file.collision_free_angles)
%     show(robot, file.file.collision_free_angles(i, :)', 'PreservePlot', false, 'FastUpdate', true)
%     drawnow
% end

show(robot, file.file.collision_free_angles(501058, :)', 'PreservePlot', false, 'FastUpdate', true)
