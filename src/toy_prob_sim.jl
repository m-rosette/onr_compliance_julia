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
mechanism_toy = parse_urdf(urdf_file) # gravity Default: = [0.0, 0.0, -9.81])

delete!(vis)
visuals = URDFVisuals(urdf_file)
mvis_toy = MechanismVisualizer(mechanism_toy, visuals, vis[:toy])
# render(mvis_toy)

# Initialize the mechanism state -------------------------------------
state = MechanismState(mechanism_toy)

# Joints for planar example
jount1, joint2, joint3 = joints(mechanism_toy)

joint_config = [0, 0, 0]
joint_vel_config = [0, 0, 0]

function reset_to_equilibrium(state)
    set_configuration!(state, joint_config)
    set_velocity!(state, joint_vel_config)
end

# Simulation ---------------------------------------------------------
# reset_to_equilibrium(state)

# ts = time vector, qs = configuration vector, vs, velocity vector
# ts, qs, vs = simulate(state, 10., Δt = 1e-3)
# println(typeof(qs))

range_length = 10
time = LinRange(0, 1, range_length)
config1 = LinRange(0, -3*pi/4, range_length)
config2 = LinRange(0, pi/2, range_length)
config3 = LinRange(0, pi/4, range_length)
test_config = []

frame_step = 5
animation = MeshCat.Animation()
for i in 1:range_length
    if i == 1
        MeshCat.atframe(animation, i) do 
            set_configuration!(mvis_toy, [config1[i], config2[i], config3[i]])
        end
    else
        MeshCat.atframe(animation, i * frame_step) do 
            set_configuration!(mvis_toy, [config1[i], config2[i], config3[i]])
        end
    end
end

# MeshCat.atframe(animation, 0) do 
#     set_configuration!(mvis_toy, [0, 0, 0])
# end

# MeshCat.atframe(animation, 15) do 
#     set_configuration!(mvis_toy, [-pi/2, pi/4, pi/4])
# end

# MeshCat.atframe(animation, 30) do 
#     set_configuration!(mvis_toy, [pi, 0, pi/2])
# end

# Animate Trajectory
# animation = MeshCat.Animation(mvis_toy, ts, qs)
# animation = MeshCat.Animation(mvis_toy, time, test_config)
setanimation!(mvis_toy, animation)
render(mvis_toy)



