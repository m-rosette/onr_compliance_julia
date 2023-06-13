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
using MAT
using onr_compliance_julia

include("FrameSetup.jl")
include("HydroCalc.jl")
include("SimWExt.jl")
include("PIDCtlr.jl")
include("TrajGenJoints.jl")

# println("Libraries imported.")


# Load collision free joint configurations from .mat file
# src_dir = dirname(pathof(onr_compliance_julia))
# file = joinpath(src_dir, "..", "test/WorkspaceData", "bravo_workspace_corrected.mat")
# mat_file = matopen(file)
# theta_combinations = read(mat_file, "collision_free_angles")


# Loading files ------------------------------------------------------
src_dir = dirname(pathof(onr_compliance_julia))
# urdf_file = joinpath(src_dir, "..", "urdf", "bravo7_planar_toy_saab copy.urdf")   # Arm and Seabotix
urdf_file = joinpath("urdf", "arm_camera_vehicle.urdf")

# i = 10000 
# urdf_file = joinpath("urdf/planar_configs/urdf/bin2_arm_only_configs", "bravo_config_" * "$i.urdf")

# Visualizer ---------------------------------------------------------
vis = Visualizer()
mechanism_bravo = parse_urdf(urdf_file, floating=false, gravity=[0.0, 0.0, -9.81]) # gravity Default: = [0.0, 0.0, -9.81])

delete!(vis)

# Visulaize the URDFs
mvis = MechanismVisualizer(mechanism_bravo, URDFVisuals(urdf_file), vis[:bravo])

# Name the joints and bodies of the mechanism
# vehicle_joint, joint1, joint2, joint3 = joints(mechanism_bravo)
# joint1, joint2, joint3 = joints(mechanism_bravo)
# world, body1, body2, body3 = bodies(mechanism_bravo)
world, z_frame_perception = bodies(mechanism_bravo)

# body_frame = default_frame(world)
# base_frame = root_frame(mechanism_bravo)


# ----------------------------------------------------------
#                 State Initialization
# ----------------------------------------------------------
state = MechanismState(mechanism_bravo)

# set_configuration!(mvis, joint1, pi/2)
# set_configuration!(mvis, joint2, pi/2 - pi)
# set_configuration!(mvis, joint3, pi/2 - pi)

com_wrt_vehframe = center_of_mass(state).v

com_wrt_vehframe = SVector{3, Float64}([center_of_mass(state).v[1], center_of_mass(state).v[2], center_of_mass(state).v[3]])
# com_wrt_vehframe = SVector{3, Float64}([0.3908982672531896, 5.1278841366611546e-5, 0.2849878845117186])
com_frame = CartesianFrame3D("com_cob")
com_transform = Transform3D(com_frame, default_frame(world), com_wrt_vehframe)
add_frame!(world, com_transform)
setelement!(mvis, com_frame)

# function reset_to_equilibrium!(state)
#     zero!(state)
#     # set_configuration!(mvis, vehicle_joint, pi)
# end
# reset_to_equilibrium!(state)


## Use this section of code to animate a trajectory from collision free dataset
# range_length = 10
# time = LinRange(0, 1, range_length)

# config1 = LinRange(0, pi, range_length)
# config2 = LinRange(3*pi/4, pi, range_length)
# config3 = LinRange(3*pi/4, pi, range_length)

# frame_step = 10
animation = MeshCat.Animation()
# for i in 1:range_length
#     if i == 1
#         MeshCat.atframe(animation, i) do 
#             set_configuration!(mvis, joint1, theta_combinations[i*50000, 1])
#             set_configuration!(mvis, joint2, config2[i])
#             set_configuration!(mvis, joint3, config3[i])
#         end
#     else
#         MeshCat.atframe(animation, i * frame_step) do 
#             set_configuration!(mvis, vehicle_joint, pi + i)
#             set_configuration!(mvis, joint1, theta_combinations[i*50000, 1])
#             set_configuration!(mvis, joint2, config2[i])
#             set_configuration!(mvis, joint3, config3[i])
#         end
#     end
# end

setanimation!(mvis, animation)
open(mvis)

println(center_of_mass(state).v[1]*mass(mechanism_bravo))
println(mass(mechanism_bravo))



