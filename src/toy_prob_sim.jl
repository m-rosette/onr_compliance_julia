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
urdf_file = joinpath(src_dir, "..", "urdf", "bravo7_planar_obj.urdf")     # Arm only
# urdf_file = joinpath(src_dir, "..", "urdf", "bravo7_test_axis_flop.urdf")     # Arm only
# urdf_file = joinpath(src_dir, "..", "urdf", "bravo7_seabotix.urdf")   # Arm and Seabotix


# Visualizer ---------------------------------------------------------
vis = Visualizer()
mechanism_toy = parse_urdf(urdf_file) # gravity Default: = [0.0, 0.0, -9.81])

delete!(vis)
visuals = URDFVisuals(urdf_file)
mvis_toy = MechanismVisualizer(mechanism_toy, visuals, vis[:toy])
# render(mvis_toy)

# Initialize the mechanism state -------------------------------------
state = MechanismState(mechanism_toy)

# Joints for planar example
jount1, joint2, joint3 = joints(mechanism_toy)

joint_config = [1, 0, 0]
joint_vel_config = [0, 0, 0]

function reset_to_equilibrium(state)
    set_configuration!(state, joint_config)
    set_velocity!(state, joint_vel_config)
end

# Simulation ---------------------------------------------------------
reset_to_equilibrium(state)

ts, qs, vs = simulate(state, 10., Δt = 1e-3)

# Animate Trajectory
animation = MeshCat.Animation(mvis_toy, ts, qs)
setanimation!(mvis_toy, animation)

render(mvis_toy)

center_of_mass(state)

