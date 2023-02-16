using RigidBodyDynamics
using LinearAlgebra, StaticArrays
using Rotations
using MeshCat, MeshCatMechanisms, MechanismGeometries
using CoordinateTransformations
using GeometryBasics
using Revise
using Plots, CSV, Tables, ProgressBars
using onr_compliance_julia

include("FrameSetup.jl")
include("HydroCalc.jl")
include("SimWExt.jl")
include("PIDCtlr.jl")
include("TrajGenJoints.jl")

println("Libraries imported.")


# Loading files ------------------------------------------------------
src_dir = dirname(pathof(onr_compliance_julia))
# urdf_file = joinpath(src_dir, "..", "urdf", "bravo7_planar_obj.urdf")     # Arm only
# urdf_file = joinpath(src_dir, "..", "urdf", "bravo7_test_axis_flop.urdf")     # Arm only
urdf_file = joinpath(src_dir, "..", "urdf", "bravo7_planar_seabotix.urdf")   # Arm and Seabotix


# Visualizer ---------------------------------------------------------
vis = Visualizer()
mechanism_bravo_vehicle = parse_urdf(urdf_file, floating=true, gravity=[0.0, 0.0, -10.0]) # gravity Default: = [0.0, 0.0, -9.81])

delete!(vis)

# Visulaize the URDFs
mvis = MechanismVisualizer(mechanism_bravo_vehicle, URDFVisuals(urdf_file), vis[:bravo])



# Name the joints and bodies of the mechanism
vehicle_joint, joint1, joint2, joint3 = joints(mechanism_bravo_vehicle)
~, vehicle_body, body1_1013, body2_1026, body3_1023 = bodies(mechanism_bravo_vehicle)

body_frame = default_frame(vehicle_body)
body1_frame = default_frame(body1_1013)
body2_frame = default_frame(body2_1026)
body3_frame = default_frame(body3_1023)
base_frame = root_frame(mechanism_bravo_vehicle)


# ----------------------------------------------------------
#                 COM and COB Frame Setup
# ----------------------------------------------------------
frame_names_cob = ["vehicle_cob", "body1_1013_cob", "body2_1026_cob", "body3_1023_cob", "jaw_cob"]
frame_names_com = ["vehicle_com", "body1_1013_com", "body2_1026_com", "body3_1023_com", "jaw_com"]
# Assume default frame = COM
# TODO -> verify all inertial terms in the URDF - they seem to be mismatched with the inertial terms from the alpha arm...
    # bravo_1013_joint_link is correct (joint 3 in manual)
    # bravo_1026_0_joint_link is corrected (joint 4 in manual)
    # bravo_1023_0_joint_link NOT SURE (DONT KNOW in manual) (guessing joint 5 in manual)
cob_vecs = [SVector{3, Float64}([0.0, 0.0, 0.02]), SVector{3, Float64}([0.033, -0.043, -0.07]), SVector{3, Float64}([0.020, 0.012, -0.140]), SVector{3, Float64}([0.033, -0.038, -0.008]), SVector{3, Float64}([0.0, 0.0, 0.0])]
com_vecs = [SVector{3, Float64}([0.0, 0.0, 0.0]), SVector{3, Float64}([0.022, -.029, 0.001]), SVector{3, Float64}([0.017, -0.026, -0.002]), SVector{3, Float64}([0.020, -0.024, 0.001]), SVector{3, Float64}([0.0, 0.0, 0.0])]
cob_frames = []
com_frames = []
setup_frames!(mechanism_bravo_vehicle, frame_names_cob, frame_names_com, cob_vecs, com_vecs, cob_frames, com_frames)


# ---------------------------------------------------------------
#                       BUOYANCY SETUP
# ---------------------------------------------------------------
# f = rho * g * V
# f = 997 (kg/m^3) * 9.81 (m/s^2) * V_in_L *.001 (m^3) = kg m / s^2
# One time setup of buoyancy forces
# KEEP ARM BASE VALUES AT END OF LIST for HydroCalc
rho = 997
volumes = [10.23 / (.001*rho), .47, .51, .43, .72] # vehicle, shoulder, ua, elbow, wrist, jaw, armbase
buoy_force_mags = volumes * rho * 9.81 * .001
buoy_lin_forces = []
for mag in buoy_force_mags
    lin_force = FreeVector3D(base_frame, [0.0, 0.0, mag])
    push!(buoy_lin_forces, lin_force)
end

masses = [10.0, 1.14, 1.14, 1.03, 1.25]
grav_forces = masses * 9.81
grav_lin_forces = []
for f_g in grav_forces
    lin_force = FreeVector3D(base_frame, [0.0, 0.0, -f_g])
    push!(grav_lin_forces, lin_force)
end

# No provided drag terms for the Bravo arm 
# Using alpha arm terms and scaling them
scale_factor = 3
drag_link1 = [0.26 0.26 0.3] * rho * scale_factor
drag_link2 = [0.3 1.6 1.6] * rho * scale_factor
drag_link3 = [0.26 0.3 0.26] * rho * scale_factor
link_drag_coeffs = [drag_link1, drag_link2, drag_link3]

println("CoM and CoB frames initialized. \n")


# ----------------------------------------------------------
#                 State Initialization
# ----------------------------------------------------------

function reset_to_equilibrium!(state)
    zero!(state)
    set_configuration!(state, vehicle_joint, [.9777, -.0019, 0.2098, .0079, 0., 0., 0.])
end

# Simulation ---------------------------------------------------------
state = MechanismState(mechanism_bravo_vehicle)
Δt = 1e-3
ctrl_freq = 100
final_time = 5.0
goal_freq = 100
sample_rate = Int(floor((1/Δt)/goal_freq))

# Control variables
do_scale_traj = true   # Scale the trajectory?
duration_after_traj = 1.0   # How long to simulate after trajectory has ended

# # include("PIDCtlr.jl")
# # include("TrajGenJoints.jl")
# # # include("HydroCalc.jl")
# # include("SimWExt.jl")


# ----------------------------------------------------------
#                      Gather Sim Data
# ----------------------------------------------------------

num_trajs = 1 
save_to_csv = false
show_animation = true
bool_plot_velocities = false
bool_plot_taus = false
bool_plot_positions = false

# Reset the sim to the equilibrium position
reset_to_equilibrium!(state)

# Start up the controller
ctlr_cache = CtlrCache(Δt, ctrl_freq, mechanism_bravo_vehicle) # -------------------------------------------- DO I NEED THIS??? ----------------------------------------



# ----------------------------------------------------------
#                          Simulate
# ----------------------------------------------------------
# Generate a random waypoint and see if there's a valid trajectory to it
wp = gen_rand_waypoints_to_rest()

traj = find_trajectory(wp) 

# Keep trying until a good trajectory is found
while traj === nothing
    global wp = gen_rand_waypoints_to_rest()
    global traj = find_trajectory(wp)
end

# Scale that trajectory to 1x-3x "top speed"
if do_scale_traj == true
    scaled_traj = scale_trajectory(traj...)
else
    scaled_traj = traj 
end
params = scaled_traj[1]
duration = params.T
println("Scaled trajectory duration: $(duration) seconds")
poses = scaled_traj[2]
vels = scaled_traj[3]

# Make vector of waypoint values and time step to save to csv
waypoints = [Δt*sample_rate params.wp.start.θs[1:3]... params.wp.goal.θs[1:3]... params.wp.start.dθs[1:3]... params.wp.goal.dθs[1:3]...]
wp_data = Tables.table(waypoints)

# Save waypoints (start and goal positions, velocities) to CSV file
if save_to_csv == true
    if n == 1
        goal_headers = ["dt", "E_start", "D_start", "C_start", "B_start", "E_end", "D_end", "C_end", "B_end", "dE_start", "dD_start", "dC_start", "dB_start", "dE_end", "dD_end", "dC_end", "dB_end"]
        CSV.write("data/full-sim-data-110822/full-sim-waypoints_110822.csv", wp_data, header=goal_headers)
    else 
        CSV.write("data/full-sim-data-110822/full-sim-waypoints_110822.csv", wp_data, header=false, append=true)
    end
end

# Simulate the trajectory
if save_to_csv != true; println("Simulating... ") end
ts, qs, vs = simulate_with_ext_forces(state, duration+duration_after_traj, params, ctlr_cache, hydro_calc!, pid_control!; Δt=Δt)
# ts, qs, vs = simulate_with_ext_forces(state, 5, params, ctlr_cache, hydro_calc!, pid_control!; Δt=Δt)
if save_to_csv != true; println("done.") end

# Downsample the desired velocities
ts_down = [ts[i] for i in 1:sample_rate:length(ts)]
des_vs = [get_desv_at_t(t, params) for t in ts_down]
des_qs = [get_desq_at_t(t, params) for t in ts_down]
paths = OrderedDict();

# Downsample the simulation output
paths["qs0"] = [qs[i][1] for i in 1:sample_rate:length(qs)]
for idx = 1:10
    joint_poses = [qs[i][idx+1] for i in 1:sample_rate:length(qs)]
    paths[string("qs", idx)] = joint_poses
end
for idx = 1:10
    joint_vels = [vs[i][idx] for i in 1:sample_rate:length(vs)]
    paths[string("vs", idx)] = joint_vels
end

if show_animation == true
    print("Animating... ")
    MeshCatMechanisms.animate(mvis, ts, qs; realtimerate = 1.0)
    println("done.")
end






# range_length = 10
# time = LinRange(0, 1, range_length)
# vehicle_rotation = RotXYX(0, 0.5, 0)
# println(vehicle_rotation)
# config0 = QuatRotation(vehicle_rotation)
# println(config0)
# println(MRP(config0))
# config1 = LinRange(0, -3*pi/4, range_length)
# config2 = LinRange(0, pi/2, range_length)
# config3 = LinRange(0, pi/4, range_length)
# test_config = []

# frame_step = 10
# animation = MeshCat.Animation()
# for i in 1:range_length
#     if i == 1
#         MeshCat.atframe(animation, i) do 
#             set_configuration!(mvis, vehicle_joint, config0) #[1, 0, 0, 0, 0, 0, 0])
#             set_configuration!(mvis, joint1, config1[i])
#             set_configuration!(mvis, joint2, config2[i])
#             set_configuration!(mvis, joint3, config3[i])
#         end
#     else
#         MeshCat.atframe(animation, i * frame_step) do 
#             set_configuration!(mvis, vehicle_joint, config0) #[1, 0, 0, 0, 0, 0, 0])
#             set_configuration!(mvis, joint1, config1[i])
#             set_configuration!(mvis, joint2, config2[i])
#             set_configuration!(mvis, joint3, config3[i])
#         end
#     end
# end


# setanimation!(mvis, animation)
# render(mvis)



