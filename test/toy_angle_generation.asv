% Angle Generation
% Marcus Rosette
% 01/03/2023

clear
clc

%% Test Using ndgrid

% robot = importrobot('three_link_arm.urdf', DataFormat='column');
% 
% num_samples = 10;
% j1 = linspace(0, 2*pi, num_samples);
% j2 = linspace(0, 2*pi, num_samples);
% j3 = linspace(0, 2*pi, num_samples);
% 
% [J1, J2, J3] = ndgrid(j1, j2, j3);
% 
% collision_check = zeros(size(J1));
% tic
% for i = 1:numel(J1) % Can iterate over either J1, J2, or J3 (same size)
% %     if ~mod(i,100)
% %         elapsedTime = toc;
% %         estimatedTime = elapsedTime*numel(J1)/i;
% %         disp(i/numel(J1));
% %         ETA = estimatedTime - elapsedTime;
% %         disp(['ETA: ',num2str(ETA/60),' minutes']);
% %     end
%     current_config = [J1(i); J2(i); J3(i)];
%     collision_check(i) = checkCollision(robot, current_config, 'Exhaustive', 'on', 'SkippedSelfCollisions','parent');
% end
% toc


% tic
% collision = find(collision_check == 1);
% collision_free = find(collision_check == 0);
% num_collision = 50;
% test_collision = collision(num_collision);
% 
% current_config = [J1(test_collision); J2(test_collision); J3(test_collision)];
% show(robot, current_config, 'visuals','on','collision','off');
% view(2)
% ax = gca;
% ax.Projection = 'orthographic';
% xlim([-3.5 3.5])
% ylim([-2 2])
% title("Collision Detected")
% toc


%% Test 1/9/23
robot = importrobot('bravo7.urdf', DataFormat='column');


num_samples = 360;            % step size
j1 = linspace(0, 2*pi, num_samples);
j2 = linspace(0, pi, num_samples);
j3 = linspace(0, pi, num_samples);
j4 = linspace(0, 2*pi, num_samples);
j5 = linspace(0, pi, num_samples);
j6 = linspace(0, 2*pi, num_samples);
j7 = linspace(0, 0.65, num_samples);  % Jaw angle - limit found via measuring on screen
j8 = linspace(0, 0.65, num_samples);  % Jaw angle - limit found via measuring on screen

window_size = 3;
num_joints = 8;
i = 1;
cell_num = 1;
theta_combinations = cell(size(zeros(num_samples/window_size, 1)));
collision_check = cell(size(zeros(num_samples/window_size, 1)));

tic
while i ~= num_samples + 1
    window_adjustment = i:i+window_size-1;
    theta_1 = j1(window_adjustment);
    theta_2 = j2(window_adjustment);
    theta_3 = j3(window_adjustment);
    theta_4 = j4(window_adjustment);
    theta_5 = j5(window_adjustment);
    theta_6 = j6(window_adjustment);
    theta_7 = j7(window_adjustment);
    theta_8 = j8(window_adjustment);
    
    [J1, J2, J3, J4, J5, J6, J7, J8] = ndgrid(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, theta_7, theta_8);
    collision_check{cell_num, 1} = zeros(size(J1));
    tic
    for k = 1:numel(J1) % Can iterate over either J1, J2, J3, etc (same size)

        if ~mod(k,1000)
            elapsedTime = toc;
            estimatedTime = elapsedTime*numel(J1)/k;
    %         disp(i/numel(J1));
            ETA = estimatedTime - elapsedTime;
            disp(['ETA: ',num2str(ETA/60),' minutes']);
        end

        current_config = [J1(k); J2(k); J3(k); J4(k); J5(k); J6(k); J7(k); J8(k)];
    
        collision_check{cell_num, 1}(k) = checkCollision(robot, current_config, 'Exhaustive', 'on', 'SkippedSelfCollisions','parent');
    end
    toc
    i = i + window_size;
    cell_num = cell_num + 1;
end
toc







