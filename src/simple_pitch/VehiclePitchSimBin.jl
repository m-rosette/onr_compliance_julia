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

include("FrameSetup_simple.jl")
include("HydroCalc_simple.jl")
include("SimWExt_simple.jl")

# println("Libraries imported.")

# Load collision free joint configurations from .mat file
src_dir = dirname(pathof(onr_compliance_julia))
file = joinpath(src_dir, "..", "urdf/planar_configs", "disc_bin2_config_space_100.mat")
mat_file = matopen(file)
discretized_index = read(mat_file, "closestIndex")

disc_space = 100 
 
global num_config = 44668 #25408 # #_#_#_#_#_#_#_#_#_#_#_#__$$_$_$$$$_$_$_$_$_$_$_$_$_$_$_$_$_$_$_$_$__$_$_$$$ NEED TO UPDATE THIS - depending on dataset size ------------------------------
global final_pitches = Array{Float64}(undef, num_config)

global i = 0
global pitch = 0

while pitch >= 0 && pitch <= 0.32 && i <= num_config - 1
    println(" Iteration: ")
    println("--------    $i    --------")
    println("   ")

    if isnan(discretized_index[i + 1])
        global i = i + 1
        final_pitches[i] = NaN
        continue
    end

    # Loading files ------------------------------------------------------
    urdf_file = joinpath("urdf/planar_configs/urdf/arm_camera_vehicle", "bravo_config_" * "$i.urdf")

    mechanism_bravo_vehicle = parse_urdf(urdf_file, floating=false, gravity=[0.0, 0.0, -9.81])

    # Name the joints and bodies of the mechanism
    vehicle_joint = joints(mechanism_bravo_vehicle)

    global ~, vehicle_body = bodies(mechanism_bravo_vehicle)

    body_frame = default_frame(vehicle_body)
    base_frame = root_frame(mechanism_bravo_vehicle)


    # ----------------------------------------------------------
    #                 COM and COB Frame Setup
    # ----------------------------------------------------------
    frame_names_cob = ["vehicle_cob"]
    frame_names_com = ["vehicle_com"]
    # Assume default frame = COM 
    # TODO: Verify the location of the COB and COM on vehicle body
    # cob_vecs = [SVector{3, Float64}([0.0, 0.0, 0.02]), SVector{3, Float64}([0.033, -0.043, -0.07]), SVector{3, Float64}([0.020, 0.012, -0.140]), SVector{3, Float64}([0.033, -0.038, -0.008]), SVector{3, Float64}([0.0, 0.0, 0.0])]
    # com_vecs = [SVector{3, Float64}([0.0, 0.0, 0.0]), SVector{3, Float64}([0.022, -.029, 0.001]), SVector{3, Float64}([0.017, -0.026, -0.002]), SVector{3, Float64}([0.020, -0.024, 0.001]), SVector{3, Float64}([0.0, 0.0, 0.0])]
    global cob_vecs = [SVector{3, Float64}([0.0, 0.0, 0.02])] #, SVector{3, Float64}([-0.018, -0.003, -0.003]), SVector{3, Float64}([0.027, -0.011, 0.092]), SVector{3, Float64}([0.145, 0.035, -0.001]), SVector{3, Float64}([0.033, -0.043, -0.07]), SVector{3, Float64}([0.020, 0.012, -0.140]), SVector{3, Float64}([0.033, -0.038, -0.008]), SVector{3, Float64}([0.0, 0.0, -0.152]), SVector{3, Float64}([0.028, 0.001, 0.0])]
    global com_vecs = [SVector{3, Float64}([0.0, 0.0, 0.0])] #, SVector{3, Float64}([-0.018, -.004, -0.001]), SVector{3, Float64}([0.017, -.007, 0.057]), SVector{3, Float64}([0.117, 0.015, 0.006]), SVector{3, Float64}([0.022, -.029, 0.001]), SVector{3, Float64}([0.018, 0.006, -0.117]), SVector{3, Float64}([0.020, -0.024, 0.001]), SVector{3, Float64}([0, 0, -0.128]), SVector{3, Float64}([0.028, -0.001, 0.0])]

    global cob_frames = []
    global com_frames = []

    # TODO: Need to implement a "simple" version of FrameSetup ------------------------------------------------------------------------------------########################### HERE ###############
    setup_frames!(mechanism_bravo_vehicle, frame_names_cob, frame_names_com, cob_vecs, com_vecs, cob_frames, com_frames)

    # ---------------------------------------------------------------
    #                       BUOYANCY SETUP
    # ---------------------------------------------------------------
    # f = rho * g * V
    # f = 997 (kg/m^3) * 9.81 (m/s^2) * V_in_L *.001 (m^3) = kg m / s^2
    # One time setup of buoyancy forces
    # KEEP ARM BASE VALUES AT END OF LIST for HydroCalc
    rho = 997
    # TODO: Need to verify vehicle volume calc
    volumes = [60 / (.001*rho)] #, 0.60, 1.94, 0.47, 0.51, 0.43, 0.48, 0.16, 0.72] # vehicle, ........, armbase
    buoy_force_mags = volumes * rho * .001
    global buoy_lin_forces = []
    for mag in buoy_force_mags
        lin_force = FreeVector3D(base_frame, [0.0, 0.0, mag])
        push!(buoy_lin_forces, lin_force)
    end

    # TODO: Need to verify vehicle mass
    masses = [60] #, 1.55, 1.98, 1.14, 1.14, 1.03, 1.04, 0.47, 1.25] # vehicle, ........, armbase
    grav_forces = masses
    global grav_lin_forces = []
    for f_g in grav_forces
        lin_force = FreeVector3D(base_frame, [0.0, 0.0, -f_g])
        push!(grav_lin_forces, lin_force)
    end

    function reset_to_equilibrium!(state)
        zero!(state)
        zero_velocity!(state)
    end

    # Constants ---------------------------------------------------------
    state = MechanismState(mechanism_bravo_vehicle)
    final_time = 10
    Δt = 1e-3

    # Reset the sim to the equilibrium position
    reset_to_equilibrium!(state)

    ts, qs, vs = simulate_with_ext_forces(state, final_time, hydro_calc!; Δt=Δt)

    # Output the final pitch value of the simulation --------------------
    pitch = last(qs)[1]
    
    if pitch >= 0 && pitch <= 0.32
        global i = i + 1
        final_pitches[i] = pitch
        # println(pitch)
    end     
end

CSV.write("test/WorkspaceData/pitch_data/arm_camera_pitch_data.csv", Tables.table(final_pitches), writeheader=false)