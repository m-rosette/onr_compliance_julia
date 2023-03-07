using MAT
using JLD
using Random
using StaticArrays, RigidBodyDynamics, Rotations

# TODO -> Modify Hannah's code to utilize your collision free workspace

# Load collision free joint configurations from .mat file
src_dir = dirname(pathof(onr_compliance_julia))
file = joinpath(src_dir, "..", "test/WorkspaceData", "bravo_workspace_corrected.mat")
mat_file = matopen(file)
theta_combinations = read(mat_file, "collision_free_angles")


mutable struct jointState
    θs::Array{Float64}
    dθs::Array{Float64}
end

mutable struct Waypoints
    start::jointState 
    goal::jointState
end

mutable struct trajParams
    a::Array        # Array of time scaling coefficients (quintic traj)
    wp::Waypoints   # Waypoints the traj is going between
    T::Float64      # Duration of the trajectory
end

num_its=50
num_actuated_dofs = 3
num_trajectory_dofs = 3
max_duration = 20.
# θa = atan(145.3, 40)
joint_lims = [[-175*pi/180, 175*pi/180], [0, pi], [0, pi], [0, pi]]
# Velocity limits: 30 degrees/s for Joints E, D, C, 50 degrees/s for Joint B (from Reach Alpha Integration Manual V1.15 p8)
    # Marcus -> setting all velocity limits to +-30deg/s
θb = deg2rad(30)
vel_lims = [[-θb, θb], [-θb, θb], [-θb, θb], [-θb, θb]]

equil_pose = zeros(num_actuated_dofs)
equil_pt = jointState(equil_pose, zeros(num_actuated_dofs))
# extended_pt = jointState([0.01, 1.5, 2.6, 0.01, 0.], zeros(num_actuated_dofs))

# ----------------------------------------------------------
#               Point Generation (Joint Space)
# ----------------------------------------------------------
""" 
    gen_rand_feasible_point()
Randomly generates a jointState, checking for joint and velocity limits.
"""
function gen_rand_feasible_point()
    angle_set = rand(1:length(theta_combinations[:, 1]))
    θs = Array{Float64}(theta_combinations[angle_set, :]) 

    dθs = Array{Float64}(undef,num_actuated_dofs)    
    for jt_idx in  1:num_actuated_dofs
        dθs[jt_idx] = rand(vel_lims[jt_idx][1]:0.001:vel_lims[jt_idx][2])
    end

    return jointState(θs, dθs)
end

"""
    gen_rand_feasible_point_at_rest()
Randomly generates a jointState where the velocity is 0. 
"""
function gen_rand_feasible_point_at_rest()
    jS = gen_rand_feasible_point()
    jS.dθs = [0.0 0.0 0.0 0.0]
    return jS
end

# ----------------------------------------------------------
#             Waypoint Generation (Joint Space)
# ----------------------------------------------------------
"""
Generates a random Waypoint struct. Not advisable to start a trajectory.  
"""
function gen_rand_waypoints()
    Waypoints(gen_rand_feasible_point(), gen_rand_feasible_point())    
end

"""
Generates a random Waypoint struct, starting at rest at the home position. 
"""
function gen_rand_waypoints_from_equil()
    Waypoints(equil_pt, gen_rand_feasible_point()) 
end

"""
    set_waypoint_from_equil(θs, dθs)

Generates a Waypoint struct. Starts at rest at the home position; ...
ends at the provided position (θs) and velocity (dθs) of the arm.
"""
function set_waypoints_from_equil(θs, dθs)
    Waypoints(equil_pt, jointState(θs, dθs))
end

function gen_rand_waypoints_to_rest()
    Waypoints(equil_pt, gen_rand_feasible_point_at_rest())
end

"""
    save_waypoints(wp::Waypoints, name::String)

Save an already generated waypoint to a file. The file will be located in "src/tmp/", ...
and will be called [name].jld
"""
function save_waypoints(wp::Waypoints, name::String)
    save(string("src/tmp/", name, ".jld"), "start_θs", wp.start.θs, "start_dθs", wp.start.dθs, "end_θs", wp.goal.θs, "end_dθs", wp.goal.dθs)
end

""" 
    load_waypoints(name::String)

Load a waypoint from a file. The file must be located in "src/tmp/" and be in JLD file format.
"""
function load_waypoints(name::String)
    wp_raw = load(string("src/tmp/", name, ".jld"))
    new_wp = Waypoints(jointState(wp_raw["start_θs"], wp_raw["start_dθs"]), jointState(wp_raw["end_θs"], wp_raw["end_dθs"]))
    return new_wp 
end

# ----------------------------------------------------------
#               Trajectory Generation (Joint Space)
# ----------------------------------------------------------
""" 
    get_coeffs(pts::Waypoints, T, idx)
Given a set of waypoints and a time scaling, determine the time scaling coefficients
for a quintic trajectory. 
"""
function get_coeffs(pts::Waypoints, T, idx)
    # λ1 = dθ1/(θ2-θ1)
    λ1 = pts.start.dθs[idx]/(pts.goal.θs[idx]-pts.start.θs[idx])
    # λ2 = dθ2/(θ2-θ1)
    λ2 = pts.goal.dθs[idx]/(pts.goal.θs[idx]-pts.start.θs[idx])

    a0 = 0
    a1 = λ1
    a2 = 0
    a3 = -2(3T*λ1+2T*λ2-5)/(T^3)
    a4 = (8T*λ1 + 7T*λ2-15)/(T^4)
    a5 = -3(T*λ1+T*λ2-2)/(T^5)
    return [a0, a1, a2, a3, a4, a5]
end

function pos_scale_at_t(a, t)
    s = a[1] + a[2]*t + a[3]*t^2 + a[4]*t^3 + a[5]*t^4 + a[6]*t^5
end

function vel_scale_at_t(a, t)
    ds = a[2] + 2a[3]*t + 3a[4]*t^2 + 4a[5]*t^3 + 5a[6]*t^4
end

function acc_scale_at_t(a, t)
    dds = 2a[3] + 6a[4]*t + 12a[5]*t^2 + 20a[6]*t^3
end

function get_path!(poses, vels, θ1, θ2, T, a, num_its=num_its)
    dt = T/num_its

    for i = 1:num_its
        t = dt*i
        s = pos_scale_at_t(a, t)
        ds = vel_scale_at_t(a, t)
        poses[i] = θ1 + s*(θ2-θ1)
        vels[i] = ds*(θ2-θ1)
    end
    return poses, vels 
end

"""
    get_desv_at_t(t, p)
    
Given a set of trajectory parameters p and a time t, determine what the desired 
velocity is. 
"""
function get_desv_at_t(t, p)
    # println("Got request for desv. Params $(p))")
    des_vel = zeros(8)
    if t <= p.T # If the current time is less than the trajectory duration
        for i = 1:num_trajectory_dofs # des vel for last joint is always 0
            ds = vel_scale_at_t(p.a[i,:], t)
            des_vel[i+4] = ds*(p.wp.goal.θs[i]-p.wp.start.θs[i])
        end
    end
    # fill!(des_vel, 0)
    return des_vel
end

function get_desq_at_t(t, p)
    des_qs = zeros(8)
    if t < p.T 
        for i = 1:num_trajectory_dofs
            s = pos_scale_at_t(p.a[i,:], t)
            des_qs[i+4] = p.wp.start.θs[i] + s*(p.wp.goal.θs[i]-p.wp.start.θs[i])
        end
    end
    return des_qs
end

function check_lim(vals::Array, lims, idx)
    if minimum(vals) < lims[idx][1] || maximum(vals) > lims[idx][2]
        is_in_range = false
    else
        is_in_range = true
    end
    return is_in_range
end

# TODO -> What is the third element "6" in the a array??? ---------------------------------------------------------------------------------------------=-=-=-=-=-=-==-=-=-=-=-=-

function scale_trajectory(params, poses, vels)
    a = Array{Float64}(undef, num_trajectory_dofs, 6)
    scale_factor = rand(1:.01:2)
    T = params.T*scale_factor
    # println("Scaling factor: $(scale_factor)")
    for i in 1:num_trajectory_dofs
        a[i,:] = get_coeffs(params.wp, T, i)
        (poses[:,i], vels[:,i]) = get_path!(poses[:,i], vels[:,i], params.wp.start.θs[i], params.wp.goal.θs[i], T, a[i,:])
    end
    return [trajParams(a, params.wp, T), poses, vels]
end

function find_trajectory(pts::Waypoints; num_its=num_its, T_init=1.0)
    T = T_init
    poses = Array{Float64}(undef, num_its, num_actuated_dofs)
    vels = Array{Float64}(undef, num_its, num_actuated_dofs)
    feasible_ct = 0
    a = Array{Float64}(undef, num_actuated_dofs, 6)

    while feasible_ct < num_actuated_dofs && T < max_duration
        feasible_ct = 0
        for i in 1:num_actuated_dofs
            # println("Joint $(i)")
            # Get trajectory 
            a[i,:] = get_coeffs(pts, T, i)
            (poses[:,i], vels[:,i]) = get_path!(poses[:,i], vels[:,i], pts.start.θs[i], pts.goal.θs[i], T, a[i,:])

            # Evaluate if possible
            is_in_range_poses = check_lim(poses[:,i], joint_lims, i)
            is_in_range_vels = check_lim(vels[:,i], vel_lims, i)
            # println("OK poses = $(is_in_range_poses), OK vels = $(is_in_range_vels)")
            if is_in_range_poses == true && is_in_range_vels == true
                feasible_ct = feasible_ct + 1
            # else
                # println("Max joint: $(maximum(poses[:,i])), min joint: $(minimum(poses[:,i]))")
            end
        end
        if feasible_ct < num_actuated_dofs
            T = T + 0.2
        end
    end

    if feasible_ct == num_actuated_dofs
        # println("Trajectory Parameters set")
        # println("Poses: $(poses)")
        # println("Trajectory duration is $(T) seconds")
        return [trajParams(a, pts, T), poses, vels]
        # println("Trajectory parameters set")
    else
        # println("No path between points; try again.")
        return nothing
    end

end