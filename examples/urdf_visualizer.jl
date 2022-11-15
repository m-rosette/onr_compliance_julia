using MeshCat
using RigidBodyDynamics
using MeshCatMechanisms

vis = Visualizer()
open(vis) # open the visulaizer in a separate window/tab
# render(vis) # render the visualizer here inside the script

urdf = "C:/Users/marcu/OneDrive/Documents/GitHub/onr_compliance_julia/urdf/bravo_5_example.urdf"

robot = parse_urdf(urdf)

mvis = MechanismVisualizer(robot, URDFVisuals(urdf), vis)
set_configuration!(mvis, [0.0, 0.0])

state = MechanismState(robot, randn(2), randn(2))
t, q, v = simulate(state, 5.0);

animation = Animation(mvis, t, q)
setanimation!(mvis, animation)