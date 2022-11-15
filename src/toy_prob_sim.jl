using RigidBodyDynamics
using LinearAlgebra, StaticArrays
using MeshCat, MeshCatMechanisms, MechanismGeometries
using CoordinateTransformations
using GeometryBasics
using Revise
using Plots, CSV, Tables, ProgressBars
using onr_compliance_julia


src_dir = dirname(pathof(onr_compliance_julia))
urdf_file = joinpath(src_dir, "..", "urdf", "bravo_seabotix.urdf")


println("Libraries imported.")


vis = Visualizer()
mechanism_toy = parse_urdf(urdf_file; gravity = [0.0, 0.0, -9.81])

delete!(vis)
visuals = URDFVisuals(urdf_file)
mvis_toy = MechanismVisualizer(mechanism_toy, URDFVisuals(urdf_file), vis[:toy])
render(mvis_toy)