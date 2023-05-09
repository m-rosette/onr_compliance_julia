# ---------------------------------------------------------------
#                       BUOYANCY SETUP
# ---------------------------------------------------------------
# f = rho * g * V
# f = 997 (kg/m^3) * 9.81 (m/s^2) * V_in_L *.001 (m^3) = kg m / s^2
# One time setup of buoyancy forces
# KEEP ARM BASE VALUES AT END OF LIST for HydroCalc
rho = 997
# TODO: Need to verify vehicle volume calc
volumes = [60 / (.001*rho), 1, 0.60, 1.94, 0.47, 0.51, 0.43, 0.48, 0.16, 0.72] # vehicle, perception head, ........, armbase
# buoy_force_mags = volumes * rho * 9.81 * .001 # with grav
buoy_force_mags = volumes * rho * .001 # No grav
buoy_lin_forces = []
for mag in buoy_force_mags
    lin_force = FreeVector3D(base_frame, [0.0, 0.0, mag])
    push!(buoy_lin_forces, lin_force)
end
# print("Bouyancy: ")
# println(buoy_lin_forces)

# TODO: Need to verify vehicle mass
masses = [60, 1, 1.55, 1.98, 1.14, 1.14, 1.03, 1.04, 0.47, 1.25] # vehicle, perception head, ........, armbase
# grav_forces = masses * 9.81 # with grav
grav_forces = masses # No grav
grav_lin_forces = []
for f_g in grav_forces
    lin_force = FreeVector3D(base_frame, [0.0, 0.0, -f_g])
    push!(grav_lin_forces, lin_force)
end
# print("Mass w/ grav: ")
# println(grav_lin_forces)

# println("Buoyancy Balance w/ Gravity: ") # with grav
println("Buoyancy Balance w/o Gravity: ") # No grav
hydro_balance = grav_lin_forces + buoy_lin_forces
for i in hydro_balance
    println(i)
end