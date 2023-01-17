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
# test
# urdf_file = joinpath(src_dir, "..", "urdf", "bravo_7_example.urdf")     # Arm only
# urdf_file = joinpath(src_dir, "..", "urdf", "bravo7_test_axis_flop.urdf")     # Arm only
# urdf_file = joinpath(src_dir, "..", "urdf", "bravo7_seabotix.urdf")   # Arm and Seabotix

# urdf_file = joinpath(src_dir, "..", "urdf", "bravo5_seabotix.urdf")
urdf_file = joinpath(src_dir, "..", "urdf", "bravo7_seabotix.urdf")
# urdf_file = "C:/Users/marcu/OneDrive/Documents/GitHub/onr_compliance_julia/bpl_bravo_description_real_test/urdf/bravo_7_example.urdf"


println("Libraries imported.")
# main


# Visualizer ---------------------------------------------------------
vis = Visualizer()
mechanism_toy = parse_urdf(urdf_file) # gravity Default: = [0.0, 0.0, -9.81])

delete!(vis)
visuals = URDFVisuals(urdf_file)
mvis_toy = MechanismVisualizer(mechanism_toy, visuals, vis[:toy])
# render(mvis_toy)

# Initialize the mechanism state -------------------------------------
state = MechanismState(mechanism_toy)

# Initialize the joints from URDF
    # List goes down the arm chain starting at the base of the bravo
axis_g, axis_f, axis_e, axis_d, axis_c, axis_b, axis_a, finger_jaws_300,
 finger_jaws_301 = joints(mechanism_toy)

# axis_g = j7
# axis_f = j6
# axis_e = j5
# axis_d = j4
# axis_c = j3
# axis_b = j2
# axis_a = j1
# finger_300 = Need to figure out how it translates to j1
# finger_301 = Need to figure out how it translates to j1

joint_config = [0, 0, 3.14, 0, 0, 0, 0, 0, 0]
joint_vel_config = [0, 0, 0, 0, 0, 0, 0, 0, 0]

function reset_to_equilibrium(state)
    set_configuration!(state, joint_config)
    set_velocity!(state, joint_vel_config)
end


# Simulation ---------------------------------------------------------
reset_to_equilibrium(state)

ts, qs, vs = simulate(state, 10., Δt = 1e-3)

# for i in len(qs)
#     axis_g = 0
#     set_configuration!(state, axis_g)
# end

# Animate Trajectory
animation = MeshCat.Animation(mvis_toy, ts, qs)
setanimation!(mvis_toy, animation)

render(mvis_toy)

