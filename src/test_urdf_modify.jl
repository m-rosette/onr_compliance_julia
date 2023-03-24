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
using onr_compliance_julia

println("Libraries imported.")

# Loading files ------------------------------------------------------
urdf_file = joinpath("urdf", "bravo7_planar_toy_saab.urdf") 

# Visualizer ---------------------------------------------------------
vis = Visualizer()
mechanism_bravo_vehicle = parse_urdf(urdf_file, floating=true, gravity=[0.0, 0.0, 0.0])

delete!(vis)

# Visulaize the URDFs
mvis = MechanismVisualizer(mechanism_bravo_vehicle, URDFVisuals(urdf_file), vis[:bravo])

# Name the joints and bodies of the mechanism
vehicle_joint, joint1, joint2, joint3 = joints(mechanism_bravo_vehicle)
# println(joint1)

remove_joint!(mechanism_bravo_vehicle, joint1; spanning_tree_next_edge=Fixed)


# println(joints(mechanism_bravo_vehicle))