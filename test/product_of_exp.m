function ee_position = product_of_exp(M, screw_axes_list, theta_list)
    % """ Product of Exponentials
    % input: zero configuration transformation matrix
    % input: List of all screw axis in arm
    % input: List of joint angles
    % return: Updated transformation matrix
    % """
    % Check list length compatibility
    if length(screw_axes_list) ~= length(theta_list)
        % checking length of screw axis list against theta list columns
        error("Screw axis list should be the same length as theta list")
    end

    T_prod_exp = M;
    for i = 1:length(screw_axes_list)
        T_temp = matrix_exponential([screw_axes_list(i, :)], theta_list(i)) * T_prod_exp;
        T_prod_exp = T_temp;
    end
    ee_position = T_prod_exp(1:3, end)';
end