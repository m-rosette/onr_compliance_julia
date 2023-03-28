# ---------------------------------------------------------------
#                       BUOYANCY SETUP
# ---------------------------------------------------------------
# f = rho * g * V
# f = 997 (kg/m^3) * 9.81 (m/s^2) * V_in_L *.001 (m^3) = kg m / s^2
# One time setup of buoyancy forces
# KEEP ARM BASE VALUES AT END OF LIST for HydroCalc
rho = 997
# TODO: Need to verify vehicle volume calc
volumes = [60 / (.001*rho), 0.60, 1.94, 0.47, 0.51, 0.43, 0.48, 0.16, 0.72] # vehicle, ........, armbase
buoy_force_mags = volumes * rho * 9.81 * .001
buoy_lin_forces = []
for mag in buoy_force_mags
    lin_force = FreeVector3D(base_frame, [0.0, 0.0, mag])
    push!(buoy_lin_forces, lin_force)
end
print("Bouyancy: ")
println(buoy_lin_forces)

# TODO: Need to verify vehicle mass
masses = [60, 1.55, 1.98, 1.14, 1.14, 1.03, 1.04, 0.47, 1.25] # vehicle, ........, armbase
grav_forces = masses * 9.81
grav_lin_forces = []
for f_g in grav_forces
    lin_force = FreeVector3D(base_frame, [0.0, 0.0, -f_g])
    push!(grav_lin_forces, lin_force)
end
print("Mass w/ grav: ")
println(grav_lin_forces)