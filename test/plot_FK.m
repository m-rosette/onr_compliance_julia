% Plotting Forward Kinematics
% Marcus Rosette
% 01/27/2023

file = load("Collision Data Storage\planar_arm_workspace_data - Copy.mat");

% Import robot URDF
robot = importrobot('urdf\bravo7_planar.urdf', DataFormat='column');

joint_angles = [pi; 1.5708; 1.5708];

transform = getTransform(robot, joint_angles, "end_effector_tip", "world"); % THIS WORKED!!!!!!!----------------------------------
ee_point = transform(1:3, 4)'
% % figure;
% show(robot, joint_angles, "Visuals","on")
% hold on 
% scatter3(ee_point(1), 0, ee_point(3), 10000, '.')
figure;
show(robot, [3*pi/4; pi; pi/2], "Visuals","on");
hold on 
scatter3(file.ee_points(:, 1), file.ee_points(:, 2), file.ee_points(:, 3), "."); % Converted mm to m
xlim([-0.5, 1.1])
ylim([-0.1, 0.1])
zlim([-0.75, 1.1])
title("Bravo Planar Workspace")
xlabel("x-axis (m)")
ylabel("y-axis (m)")
zlabel("z-axis (m)")
grid on

transform = getTransform(robot, joint_angles, "end_effector_tip", "world"); % THIS WORKED!!!!!!!----------------------------------
ee_point = transform(1:3, 4)'
% % figure;
% show(robot, joint_angles, "Visuals","on")
% hold on 
% scatter3(ee_point(1), 0, ee_point(3), 10000, '.')

% figure;
% show(robot, [0; 0; 0])
% hold on 
% scatter3(-file.file.ee_points(:, 1)/1000, file.file.ee_points(:, 2), file.file.ee_points(:, 3)/1000, "."); % Converted mm to m
% scatter(file.file.ee_points(:, 1)/1000, file.file.ee_points(:, 3)/1000, "."); % Converted mm to m
% xlim([-1.5, 1.5])
% zlim([-1.5, 1.5])
% title("Possible Bravo End-Effector Points")
% xlabel("x-axis (m)")
% ylabel("z-axis (m)")
% grid on
% set(gca, 'XDir','reverse')




% Found a repeated collision at index 7415
% More collisions found at index 13051
% More at index 501058
% IT SEEMS LIKE THE END EFFECTOR MESH IS NOT INCLUEDED IN THE COLLISION CHECK

% Collisions consistently 134 to 136 indexes apart

% figure;
% test_window = 7415;
% collision_index = [];
% tic
% for i = test_window:test_window+5000
%     test_collision_check = checkCollision(robot, file.file.collision_free_angles(i, :)', 'SkippedSelfCollisions', 'parent');
%     if test_collision_check
%         collision_index(end+1) = i;
%         disp(i)
%     end
% %     show(robot, file.file.collision_free_angles(i, :)', 'PreservePlot', false, 'FastUpdate', true)
% %     hold on
%     transform = getTransform(robot, file.file.collision_free_angles(i, :)', "end_effector_tip", "world");
%     ee_point = transform(1:3, 4)';
% %     scatter3(ee_point(1), 0, ee_point(3), '.', "blue")
% %     scatter3(-file.file.ee_points(i, 1)/1000, 0, file.file.ee_points(i, 3)/1000, ".");
%     drawnow
% end
% toc
 
% show(robot, file.file.collision_free_angles(501058, :)', 'PreservePlot', false, 'FastUpdate', true)
