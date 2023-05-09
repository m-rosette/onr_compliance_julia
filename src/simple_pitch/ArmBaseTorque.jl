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

for i in 1:num_config
    println(" Iteration: ")
    println("--------    $i    --------")
    println("   ")

    if isnan(discretized_index[i])
        final_torque[i] = NaN
        continue
    end

    # Loading files ------------------------------------------------------
    urdf_file = joinpath("urdf/planar_configs/urdf/arm_zframe", "bravo_config_" * "$i.urdf")
    # urdf_file = joinpath("urdf/planar_configs/urdf/arm_only_configs", "bravo_config_2035.urdf")

    # Mechanism Setup
    mechanism_bravo = parse_urdf(urdf_file, floating=false, gravity=[0.0, 0.0, -9.81])

    # Name the joints and bodies of the mechanism
    vehicle_joint = joints(mechanism_bravo)

    global world, body1, body2, body3 = bodies(mechanism_bravo)

    body_frame = default_frame(world)
    base_frame = root_frame(mechanism_bravo)

    # Initialize the state
    state = MechanismState(mechanism_bravo)

    # Find the COM about X (out of the plane)
    com_wrt_arm_base = center_of_mass(state).v[1]

    # Calc torque from total mass times COM
    final_torque[i] = com_wrt_arm_base * mass(mechanism_bravo)
end

# println(final_torque)
println("done")

CSV.write("test/WorkspaceData/pitch_data/arm_zframe_torque_data.csv", Tables.table(final_torque), writeheader=false)