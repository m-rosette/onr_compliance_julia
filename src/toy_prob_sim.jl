using RigidBodyDynamics
using LinearAlgebra, StaticArrays
using MeshCat, MeshCatMechanisms, MechanismGeometries
using CoordinateTransformations
using GeometryBasics
using Revise
using Plots, CSV, Tables, ProgressBars
using onr_compliance_julia
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
    # bravo_1023_0_joint_link NOT SURE (DONT KNOW in manual)
cob_vecs = [SVector{3, Float64}([0.0, 0.0, 0.02]), SVector{3, Float64}([0.033, -0.043, -0.07]), SVector{3, Float64}([0.020, 0.012, -0.140]), SVector{3, Float64}([0.0, 0.003, -.098]), SVector{3, Float64}([0.0, 0.0, 0.0])]
com_vecs = [SVector{3, Float64}([0.0, 0.0, 0.0]), SVector{3, Float64}([0.022, -.029, 0.001]), SVector{3, Float64}([0.017, -0.026, -0.002]), SVector{3, Float64}([0.0, 0.003, -.098]), SVector{3, Float64}([0.0, 0.0, 0.0])]
cob_frames = []
com_frames = []


# Initialize the mechanism state -------------------------------------
state = MechanismState(mechanism_bravo_vehicle)

joint_config = [0, 0, 0]
joint_vel_config = [0, 0, 0]

function reset_to_equilibrium(state)
    set_configuration!(state, joint_config)
    set_velocity!(state, joint_vel_config)
end

# Simulation ---------------------------------------------------------
reset_to_equilibrium(state)

range_length = 10
time = LinRange(0, 1, range_length)
config1 = LinRange(0, -3*pi/4, range_length)
config2 = LinRange(0, pi/2, range_length)
config3 = LinRange(0, pi/4, range_length)
test_config = []

frame_step = 10
animation = MeshCat.Animation()
for i in 1:range_length
    if i == 1
        MeshCat.atframe(animation, i) do 
            set_configuration!(mvis, vehicle_joint, [1, 0, 0, 0, 0, 0, 0])
            set_configuration!(mvis, joint1, config1[i])
            set_configuration!(mvis, joint2, config2[i])
            set_configuration!(mvis, joint3, config3[i])
        end
    else
        MeshCat.atframe(animation, i * frame_step) do 
            set_configuration!(mvis, vehicle_joint, [1, 0, 0, 0, 0, 0, 0])
            set_configuration!(mvis, joint1, config1[i])
            set_configuration!(mvis, joint2, config2[i])
            set_configuration!(mvis, joint3, config3[i])
        end
    end
end


setanimation!(mvis, animation)
render(mvis)



