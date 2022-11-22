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
urdf_file = joinpath(src_dir, "..", "urdf", "bravo_7_example.urdf")     # Arm only
# urdf_file = joinpath(src_dir, "..", "urdf", "bravo7_test_axis_flop.urdf")     # Arm only
# urdf_file = joinpath(src_dir, "..", "urdf", "bravo7_seabotix.urdf")   # Arm and Seabotix


# Visualizer ---------------------------------------------------------
vis = Visualizer()
mechanism_toy = parse_urdf(urdf_file; gravity = [0.0, 0.0, -9.81])

delete!(vis)
visuals = URDFVisuals(urdf_file)
mvis_toy = MechanismVisualizer(mechanism_toy, URDFVisuals(urdf_file), vis[:toy])
# render(mvis_toy)

# Initialize the mechanism state
state = MechanismState(mechanism_toy)
bravo_axis_g, bravo_axis_f, bravo_axis_e, bravo_axis_d, bravo_axis_c, bravo_axis_b, bravo_axis_a, bravo_finger_jaws_rs2_300_joint, bravo_finger_jaws_rs2_301_joint = joints(mechanism_toy)
axis_g, axis_f, axis_e, axis_d, axis_c, axis_b, axis_a, finger_jaws_300, finger_jaws_301 = joints(mechanism_toy)
# Δt = 1e-3

# set_configuration!(state, finger_jaws_300, 3)
# set_velocity!(state, finger_jaws_300, 1.)


# Simulation
ts, qs, vs = simulate(state, 10., Δt = 1e-3)

# Animate Trajectory
animation = MeshCat.Animation(mvis_toy, ts, qs)
setanimation!(mvis_toy, animation)

render(mvis_toy)