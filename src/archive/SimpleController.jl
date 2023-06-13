function simple_control!(torques::AbstractVector, t, state::MechanismState)
    # torques[velocity_range(state, vehicle_joint)] .= [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    torques[velocity_range(state, vehicle_joint)] .= -15 .* velocity(state, vehicle_joint)

    torques[velocity_range(state, joint1)] .= -15 .* velocity(state, joint1)
    torques[velocity_range(state, joint2)] .= -15 .* velocity(state, joint2)
    torques[velocity_range(state, joint3)] .= -15 .* velocity(state, joint3)

end;