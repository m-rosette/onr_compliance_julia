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


# Load collision free joint configurations from .mat file
src_dir = dirname(pathof(onr_compliance_julia))
# file = joinpath(src_dir, "..", "urdf/planar_configs", "new_space_100.mat")
file = joinpath(src_dir, "..", "urdf/planar_configs", "disc_bin2_config_space_100.mat")
mat_file = matopen(file)
discretized_index = read(mat_file, "closestIndex")

global num_config = 44668
global final_torque = Array{Float64}(undef, num_config)
global com_wrt_bravo_base = Array{Float64}(undef, num_config)

for i in 1:num_config
    println(" Iteration: ")
    println("--------    $i    --------")
    println("   ")

    if isnan(discretized_index[i])
        final_torque[i] = NaN
        com_wrt_bravo_base[i] = NaN
        continue
    end

    # Loading files ------------------------------------------------------
    bravo_urdf_file = joinpath("urdf/planar_configs/urdf/bin2_arm_only_configs", "bravo_config_" * "$i.urdf")
    arm_vehicle_urdf_file = joinpath("urdf/planar_configs/urdf/arm_camera_vehicle", "bravo_config_" * "$i.urdf")

    # Mechanism Setup
    mechanism_bravo = parse_urdf(bravo_urdf_file, floating=false, gravity=[0.0, 0.0, -9.81])
    mechanism_vehicle = parse_urdf(arm_vehicle_urdf_file, floating=false, gravity=[0.0, 0.0, -9.81])

    # Name the joints and bodies of the mechanism
    vehicle_joint = joints(mechanism_vehicle)

    # global world, body1, body2, body3 = bodies(mechanism_bravo)
    global world, body1 = bodies(mechanism_vehicle)

    body_frame = default_frame(world)
    base_frame = root_frame(mechanism_vehicle)
    vect_to_vehicle_base = SVector{3, Float64}([0.425, 0.0, -.39])
    x_to_vehicle_base = vect_to_vehicle_base[1]

    # Initialize the state
    bravo_state = MechanismState(mechanism_bravo)

    # Find the COM about X (out of the plane)
    com_wrt_bravo_base[i] = center_of_mass(bravo_state).v[1] 
    com_wrt_vehicle_joint = com_wrt_bravo_base[i] + x_to_vehicle_base

    # Calc torque from total mass times COM
    final_torque[i] = com_wrt_vehicle_joint * mass(mechanism_bravo)

end

# println(final_torque)
println("done")

# CSV.write("test/WorkspaceData/pitch_data/arm_vehicle_com.csv", Tables.table(com_wrt_bravo_base), writeheader=false)
# CSV.write("test/WorkspaceData/pitch_data/arm_vehicle_torque.csv", Tables.table(final_torque), writeheader=false)