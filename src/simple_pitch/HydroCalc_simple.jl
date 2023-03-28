# Hydrodynamics calculator functions 
using RigidBodyDynamics

# ------------------------------------------------------------------------
#                HYDRODYNAMICS (Grav & Buoy) CALCULATOR
# ------------------------------------------------------------------------
function hydro_calc!(hydro_wrenches::Dict{BodyID, Wrench{Float64}}, t, state::MechanismState)
    buoy_wrenches = []
    grav_wrenches = []
    names = ["cob1"]

    # Get the body
    ~, bod = bodies(state.mechanism)

    # Get default frame of the body
    body_default_frame = default_frame(bod)

    # -------- Calculate Buoyancy Wrench-------
    # Get transform between the defualt frame and the center of buoyancy
    # TODO: don't use fixed_transform because it's bad for computation time
    def_to_cob = fixed_transform(bod, body_default_frame, cob_frames[1])

    # Transform buoyancy force vector to the body's default frame (rotation only)
    buoy_force_trans = transform(state, buoy_lin_forces[1], body_default_frame)

    # Make the wrench: the buoyancy force through a point, the center of buoyancy.
    buoy_wrench = Wrench(Point3D(body_default_frame, translation(inv(def_to_cob))), buoy_force_trans)
    push!(buoy_wrenches, buoy_wrench)

    # -------- Calculate Gravity Wrench -------
    def_to_com = fixed_transform(bod, body_default_frame, com_frames[1])
    grav_force_trans = transform(state, grav_lin_forces[1], body_default_frame)
    # println(grav_force_trans)
    
    # Make the wrench: the buoyancy force through a point, the center of buoyancy.
    grav_wrench = Wrench(Point3D(body_default_frame, translation(inv(def_to_com))), grav_force_trans)

    # Add wrench to buoy_wrenches
    push!(grav_wrenches, grav_wrench)

    # Add the buoyancy wrench and grav wrench together
    wrench = buoy_wrench + grav_wrench
    # println(wrench)

    # ----- Special calculaitons for the vehicle -----
    # ----- Grav/buoy for arm base link ----- 
    def_to_armbase_cob = fixed_transform(bod, body_default_frame, cob_frames[end])
    def_to_armbase_com = fixed_transform(bod, body_default_frame, com_frames[end])
    buoy_force_trans_armbase = transform(state, buoy_lin_forces[end], body_default_frame)
    grav_force_trans_armbase = transform(state, grav_lin_forces[end], body_default_frame)
    buoy_wrench_arm = Wrench(Point3D(body_default_frame, translation(inv(def_to_armbase_cob))), buoy_force_trans_armbase)
    grav_wrench_arm = Wrench(Point3D(body_default_frame, translation(inv(def_to_armbase_com))), grav_force_trans_armbase)
    wrench = wrench + buoy_wrench_arm + grav_wrench_arm
    # println("armbase gravity wrench in vehicle frame")
    # println(grav_wrench_arm)

    # println("Wrench without drag:")
    # println(wrench)

    # ----- Drag of the vehicle -----
    vel=velocity(state, joints(state.mechanism)[1])

    # TODO: Dont know how to do the drag on only the vehicle
    d_lin_coeffs = [4.03, 20, 5.18, .07, .07, .07]
    d_nonlin_coeffs = [18.18, 21.66, 36.99, 1.55, 1.55, 1.55]
    tau_d = -d_lin_coeffs .* vel .+ -d_nonlin_coeffs .* vel .* abs.(vel)
    drag_wrench = Wrench(body_default_frame, tau_d[1:3], tau_d[4:6])   
    # # println("Vehicle velocity is $(vel)")
    # # println("Vehicle drag is $(drag_wrench)")
    
    wrench = wrench + drag_wrench
    # println("Wrench drag:")
    # println(drag_wrench)

    # Transform the wrench to the root frame and assign it to the body
    hydro_wrenches[BodyID(bod)] = transform(state, wrench, root_frame(state.mechanism))
    
end;