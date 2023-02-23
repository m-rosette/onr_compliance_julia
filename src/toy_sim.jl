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

println("Libraries imported.")


# Load collision free joint configurations from .mat file
src_dir = dirname(pathof(onr_compliance_julia))
file = joinpath(src_dir, "..", "test/WorkspaceData", "bravo_workspace_corrected.mat")
mat_file = matopen(file)
theta_combinations = read(mat_file, "collision_free_angles")


# Loading files ------------------------------------------------------
src_dir = dirname(pathof(onr_compliance_julia))
urdf_file = joinpath(src_dir, "..", "urdf", "bravo7_planar_seabotix.urdf")   # Arm and Seabotix


# Visualizer ---------------------------------------------------------
vis = Visualizer()
mechanism_bravo_vehicle = parse_urdf(urdf_file, floating=false, gravity=[0.0, 0.0, -9.81]) # gravity Default: = [0.0, 0.0, -9.81])

delete!(vis)

# Visulaize the URDFs
mvis = MechanismVisualizer(mechanism_bravo_vehicle, URDFVisuals(urdf_file), vis[:bravo])

# Name the joints and bodies of the mechanism
# vehicle_joint, joint1, joint2, joint3 = joints(mechanism_bravo_vehicle)
joint1, joint2, joint3 = joints(mechanism_bravo_vehicle)
# ~, vehicle_body, body1_1013, body2_1026, body3_1023 = bodies(mechanism_bravo_vehicle)

println(joint1)


# ----------------------------------------------------------
#                 State Initialization
# ----------------------------------------------------------

function reset_to_equilibrium!(state)
    zero!(state)
    set_configuration!(state, vehicle_joint, [.9777, -.0019, 0.2098, .0079, 0., 0., 0.])
end

# state = MechanismState(mechanism_bravo_vehicle)

# reset_to_equilibrium!(state)

# range_length = 10
# time = LinRange(0, 1, range_length)

# vehicle_config = [1, 0, 0, 0, 0, 0, 0]

# joint_index = 100000
# thetas = Array{Float64}(theta_combinations[(joint_index:10:joint_index+100), :]) 
# # println(thetas)

# config1 = LinRange(0, -3*pi/4, range_length)

# frame_step = 10
# animation = MeshCat.Animation()
# for i in 1:range_length
#     if i == 1
#         MeshCat.atframe(animation, i) do 
#             # set_configuration!(mvis, vehicle_joint, vehicle_config)
#             set_configuration!(mvis, joint1, config1[1])
#             set_configuration!(mvis, joint2, thetas[i, 2])
#             set_configuration!(mvis, joint3, thetas[i, 3])
#         end
#     else
#         MeshCat.atframe(animation, i * frame_step) do 
#             # set_configuration!(mvis, vehicle_joint, vehicle_config)
#             set_configuration!(mvis, joint1, config1[1])
#             set_configuration!(mvis, joint2, thetas[i, 2])
#             set_configuration!(mvis, joint3, thetas[i, 3])
#         end
#     end
# end


range_length = 10
time = LinRange(0, 1, range_length)

config1 = LinRange(0, pi, range_length)
config2 = LinRange(pi/2, pi, range_length)
config3 = LinRange(pi/2, pi, range_length)

frame_step = 10
animation = MeshCat.Animation()
for i in 1:range_length
    if i == 1
        MeshCat.atframe(animation, i) do 
            set_configuration!(mvis, joint1, theta_combinations[i*50000, 1])
            set_configuration!(mvis, joint2, config2[i])
            set_configuration!(mvis, joint3, config3[i])
        end
    else
        MeshCat.atframe(animation, i * frame_step) do 
            set_configuration!(mvis, joint1, theta_combinations[i*50000, 1])
            set_configuration!(mvis, joint2, config2[i])
            set_configuration!(mvis, joint3, config3[i])
        end
    end
end

setanimation!(mvis, animation)

render(mvis)


