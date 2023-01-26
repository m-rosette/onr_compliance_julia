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