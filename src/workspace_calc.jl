# Workspace Calculation
# Marcus Rosette
# 29 November 2022

using LinearAlgebra

# Parameters (from Reach Robotics Bravo7 manual) -----------------------------
# Joint Limits - J1 = end effector opening, ..., J7 = Base revolute joint
j1 = 118 # [mm] - Jaw opening
j2 = Inf # [deg] - Continuous revolute
j3 = 180 # [deg] - Elbow
j4 = Inf # [deg] - Continuous revolute
j5 = 180 # [deg] - Elbow
j6 = 180 # [deg] - Elbow
j7 = Inf # [deg] - Continuous revolute

# Body Lengths
a = 365 # [mm] - j3 to end effector
b = 160 # [mm] - j5 to j3
c = 292 # [mm] - j6 to j5
d = 159 # [mm] - table top to j6
e = 92  # [mm] - max OD
f = 82  # [mm] - min OD (at tip of wrist) 

# ------------------------------------------------------ Forward Kinematics --------------------------------------------------------
# Setup (at zero configuration) (arm slacked - pointing down)
M = [0 0 1 e; 0 1 0 0; -1 0 0 -(a + b + c -d); 0 0 0 1]

# Screw Axes - S (base) frame
S7 = [0, 0, 1, 0, 0, 0]     # Base revolute joint
S6 = [0, 1, 0, -d, 0, e/2]
S5 = [0, 1, 0, c-d, 0, e]
S4 = [0, 0, 1, 0, -e, 0]
S3 = [0, 1, 0, c-d, 0, e]
S2 = [0, 0, 1, 0, -e, 0]
S1 = [0, 0, 0, 0, 0, 1]     # End-effector prismatic joint 

screw_axes = [[S7] [S6] [S5] [S4] [S3] [S2] [S1]]  # Collecting all screw axes in one array

function matrix_exponential(screw_axis, theta)
    """ Calculate the homogeneous transformation for a set of exponential coordinates
    input: screw axis
    input: theta (rads)
    return: T - homogeneous transformation matrix
    """
    T = zeros(4, 4) # Initialize transformation matrix

    w_skew = [0 -screw_axis[3] screw_axis[2]; screw_axis[3] 0 -screw_axis[1]; -screw_axis[2] screw_axis[1] 0]
    v = [screw_axis[4]; screw_axis[5]; screw_axis[6]]
    R = I + sin(theta) * w_skew + (1 - cos(theta)) * w_skew ^ 2
    p = (I * theta + (1 - cos(theta)) * w_skew + (theta - sin(theta)) * w_skew^2) * v
    T[1:3, 1:3] = R
    T[1:3, 4] = p
    T[4, :] = [0 0 0 1]
    return T
end


function product_of_exp(M, screw_axes_list, theta_list)
    """ Product of Exponentials
    input: zero configuration transformation matrix
    input: List of all screw axis in arm
    input: List of joint angles
    return: Updated transformation matrix
    """
    # Check list length compatibility
    if length(screw_axes_list) != length(theta_list)
        ArgumentError("Screw axis list should be the same length as theta list")
    end

    T_prod_exp = M
    for i in eachindex(screw_axes_list)
        T_temp = matrix_exponential(screw_axes_list[i], theta_list[i]) * T_prod_exp
        T_prod_exp = T_temp
    end

    return T_prod_exp
end


function workspace(M, screw_axes_list, theta_list)
    no_collision = true
    while no_collision
        # Get current current end effector position
        T_end_effector = product_of_exp(M, screw_axes_list, theta_list)

        step_size = 0.1
        for (joint, angle) in enumerate(theta_list)
            new_angle = angle + step_size
            theta_list[joint] = new_angle
            T_end_effector = product_of_exp(M, screw_axes_list, theta_list)
        end
    end
end


