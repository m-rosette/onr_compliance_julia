# ----------------------------------------------------------
#                     Import Libraries
# ----------------------------------------------------------
using RigidBodyDynamics, Rotations
using LinearAlgebra, StaticArrays, DataStructures
using MeshCat, MeshCatMechanisms, MechanismGeometries
using CoordinateTransformations
using GeometryBasics
using Printf, Plots, CSV, Tables, ProgressBars, Revise
using Random
using onr_compliance_julia

include("FrameSetup.jl")
include("HydroCalc.jl")
include("SimWExt.jl")
# include("PIDCtlr.jl")
include("TrajGenJoints.jl")
include("SimpleController.jl")
include("PIDCtlr_new.jl")

println("Libraries imported.")

# Loading files ------------------------------------------------------
urdf_file = joinpath("urdf", "bravo7_planar_toy_saab copy.urdf") 

# Visualizer ---------------------------------------------------------
vis = Visualizer()
mechanism_bravo_vehicle = parse_urdf(urdf_file, floating=false, gravity=[0.0, 0.0, -9.81])

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
# TODO: -> verify all inertial terms in the URDF - they seem to be mismatched with the inertial terms from the alpha arm...
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
# TODO: Need to verify vehicle volume calc
volumes = [60 / (.001*rho), .47, .51, .43, .72] # vehicle, shoulder, ua, elbow, wrist, jaw, armbase
buoy_force_mags = volumes * rho * 9.81 * .001
buoy_lin_forces = []
for mag in buoy_force_mags
    lin_force = FreeVector3D(base_frame, [0.0, 0.0, mag])
    push!(buoy_lin_forces, lin_force)
end

# TODO: Need to verify vehicle mass
masses = [60, 1.14, 1.14, 1.03, 1.25]
grav_forces = masses * 9.81
grav_lin_forces = []
for f_g in grav_forces
    lin_force = FreeVector3D(base_frame, [0.0, 0.0, -f_g])
    push!(grav_lin_forces, lin_force)
end

# No provided drag terms for the Bravo arm 
# Using alpha arm terms and scaling them
# TODO: Determine effective way to approximate drg terms for bravo
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
    # zero!(state)
    set_configuration!(state, joint1, pi/2)
    set_configuration!(state, joint2, pi)
    set_configuration!(state, joint3, pi)
    set_configuration!(state, vehicle_joint, 0) # For fixed base joint with grav.
    # set_configuration!(state, vehicle_joint, [.9777, 0, 0, 0, 0., 0., 0.]) # For floating base joint with no grav.

    zero_velocity!(state)
end

# Constants ---------------------------------------------------------
state = MechanismState(mechanism_bravo_vehicle)
Δt = 1e-3
final_time = 10.0
ctrl_freq = 100 # Control frequency -- how often the control input can be changed
duration_after_traj = 1.0   # How long to simulate after trajectory has ended
do_scale_traj = true   # Scale the trajectory?

# Control variables -------------------------------------------------
show_animation = true

# Create trajectory 
# params = trajParams[]

# swap_times = Vector{Float64}()
# define_multiple_waypoints!(params, swap_times, 2)

# # Reset the sim to the equilibrium position
reset_to_equilibrium!(state)

ctlr_cache = CtlrCache(Δt, ctrl_freq, state)

reset_to_equilibrium!(state)

# Start up the controller
ctlr_cache = CtlrCache(Δt, ctrl_freq, state)
ctlr_cache.taus[:,1] = [0.; 0.; 0.; 0.; 0.; 10.; 0.; 0.; 0.]

# ----------------------------------------------------------
#                          Simulate
# ----------------------------------------------------------
# Generate a random waypoint and see if there's a valid trajectory to it
wp = gen_rand_waypoints_from_equil()

traj = find_trajectory(wp) 

# TODO: See if you can generate constant joint position (as shown in "test") -----------------------------------------------------------------
    # Still would need to simulate the motion of qs[1]
# test = fill([pi/2, pi, pi], 10002) # This could be used to generate qs[2:4]


# # Keep trying until a good trajectory is found
# while traj === nothing
#     wp = gen_rand_waypoints_to_rest()
#     traj = find_trajectory(wp)
#     println("stuck here...?")
# end

# Scale that trajectory to 1x-3x "top speed"
# if do_scale_traj == true
#     scaled_traj = scale_trajectory(traj...)
# else
#     scaled_traj = traj 
# end
# params = scaled_traj[1]
# duration = params.T
# poses = scaled_traj[2]
# vels = scaled_traj[3]

# # ----------------------------------------------------------
# #                          Simulate
# # ----------------------------------------------------------

# # Simulate the trajectory
# println("Simulating... ")   
# # # ts, qs, vs = simulate_with_ext_forces(state, duration+duration_after_traj, hydro_calc!; Δt=Δt)
# ts, qs, vs = simple_simulate_with_ext_forces(state, final_time, hydro_calc!, simple_control!; Δt=Δt)
# ts, qs, vs = vanilla_pid_sim_ext_force(state, final_time, params, ctlr_cache, hydro_calc!, pid_control!; Δt=Δt)
# println("done.")

# if show_animation == true
#     print("Animating... ")
#     setanimation!(mvis, MeshCat.Animation(mvis, ts, qs))
#     open(mvis)
#     println("done.")
# end

# # # The last vehicle orientation Quaternion from simulation
# # # last(qs)[1:4]
# # # last_quaternion = last(qs)[1:4]
# # # vehicle_final_rot = QuatRotation(last_quaternion)

# # # println(vehicle_final_rot)