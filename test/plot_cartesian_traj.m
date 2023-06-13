function plot_cartesian_traj(robot, stateData, dispInterval, ee_points, filename)
%displayRobot Animate the robot motion
%   Given a ROBOT, the array of states defined in STATEDATA, visualize the
%   robot position by animating it inside a for-loop. The frame display
%   interval set by DISPINTERVAL controls the number of frames between each
%   subsequent animation (increase to speed up animation), while the
%   optional value TARGETPOS adds a marker at the specified target
%   position.
        
    % Define number of moving robot joints
    numInputs = numel(homeConfiguration(robot));

    % Create figure and hold it
    figure
    set(gcf,'Visible','on');
    show(robot, stateData(1, 1:numInputs)');
    hold on
    plot3(ee_points(:, 1), ee_points(:, 2), ee_points(:, 3))
    hold on
    
    if nargin > 4
        gif(filename)
    end
    
    
    % Loop through values at specified interval and update figure
    for j = 1:dispInterval:length(stateData)
        % Display manipulator model
        qDisp = stateData(j, 1:numInputs)';
        show(robot, qDisp, 'Frames', 'off', 'PreservePlot', false);
        title(sprintf('Frame = %d of %d', j, length(stateData)));
        
        if nargin > 4
            gif
        end
        
        % Update figure
        drawnow
    end
end

