using RigidBodyDynamics, Distributions, Random

# ------------------------------------------------------------------------
#                              SETUP 
# ------------------------------------------------------------------------
# TODO: need to tune the PID controller - see Hannah's notes about this. Also see https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
arm_Kp = .025
arm_Kd = 0.0006
arm_Ki = 0.005
v_Kp = 2.0
v_Kd = 0.012
v_Ki = 0.015
Kp = [1.5, v_Kp, v_Kp, v_Kp, arm_Kp, .015, .006, .0003, 20.]
Kd = [0.0002, v_Kd, v_Kd, v_Kd, arm_Kd, 0.0004, 0.0004, 0.00004, .002]
Ki = [0.0001, v_Ki, v_Ki, v_Ki, arm_Ki, 0.005, 0.01, 0.0008, .1]

# TODO: Need to update the torque_lims - check Bravo manual for some of the values - can approx. the torque lims of the vehicle
torque_lims = [20., 71.5, 88.2, 177., 10.0, 10.0, 10.0, 0.6, 600]


mutable struct CtlrCache
    time_step::Float64
    ctrl_freq::Float64
    ctrl_steps::Float64
    vel_error_cache::Array{Float64}
    vel_int_error_cache::Array{Float64}
    step_ctr::Int
    joint_vec
    des_vel::Array{Float64}
    taus
    noisy_qs
    noisy_vs
    filtered_vs
    
    function CtlrCache(dt, control_frequency, state)
        mechanism = state.mechanism
        if length(joints(mechanism)) == 4
            vehicle_joint, jointE, jointD, jointC = joints(mechanism)
            joint_vec = [vehicle_joint, jointE, jointD, jointC]
            num_actuated_dofs = 7
            num_dofs = 9
        elseif length(joints(mechanism)) == 5
            vehicle_joint, jointE, jointD, jointC, jawjoint = joints(mechanism)
            joint_vec = [vehicle_joint, jointE, jointD, jointC, jawjoint]
            num_actuated_dofs = 8
            num_dofs = 10
        end
        ctrl_loop_num_steps = 4*(1/dt)/control_frequency
        new(dt, control_frequency, ctrl_loop_num_steps, #=
        =# zeros(num_actuated_dofs), zeros(num_actuated_dofs), 0, joint_vec, #=
        =# zeros(num_actuated_dofs), Array{Float64}(undef, num_dofs, 1), #=
        =# zeros(num_dofs+1), zeros(num_dofs), #=
        =# zeros(num_dofs)) 
    end
end

# ------------------------------------------------------------------------
#                          UTILITY FUNCTIONS
# ------------------------------------------------------------------------
"""
Given a `torque`` value and a torque `limit``, bounds `torque`` to be within the bounds of `limit`.

Assumes the torque limit is the same in the positive and negative directions.
"""
function impose_torque_limit!(torque, limit)
    if torque > limit
        torque = limit
    elseif torque < -limit
        torque = -limit
    end
end

function limit_d_tau(d_tau, limit)
    if d_tau < -limit
        d_tau = -limit
    elseif d_tau > limit
        d_tau = limit
    end
    return d_tau
end

function resettimestep(c::CtlrCache)
    c.step_ctr = 0
end

function skew(x1, x2, x3)
    output = [0 -x3 x2;
            x3 0 -x1;
            -x2 x1 0]
end

# ------------------------------------------------------------------------
#                              CONTROLLER
# ------------------------------------------------------------------------
"""
Imposes a PID controller to follow a velocity specified by TrajGen.get_desv_at_t().
In this case, also imposes damping to each joint. 
Requires the parameters of the trajectory to be followed (pars=trajParams), which consists of the quintic coefficients `a` and the two waypoints to travel between.
Only happens every 4 steps because integration is done with Runge-Kutta.
"""
function pid_control!(torques::AbstractVector, t, state::MechanismState, pars, c)
    # If it's the first time called in the Runge-Kutta, update the control torque
    if rem(c.step_ctr, 4) == 0
        # Set up empty vector for control torques
        c_taus = zeros(size(c.taus, 1),1)
        if c.step_ctr == 0
            torques[6] = 5.2 # ff z value
            torques[3] = 0.
            torques[5] = 0.
            torques[4] = -2.3 # ff x value
            torques[7] = -.004 # ff joint E value
            torques[8] = -.325 # ff Joint D value 
            torques[9] = -.034 # ff Joint C value
            # torques[10] = .004
            
            # Vehicle roll
            torques[1] = 0.
        end
        
        # Vehicle pitch is not controlled
        torques[2] = 0.
        
        c.des_vel = get_desv_at_t(t, pars)
        if rem(c.step_ctr, 1000) == 0
            println("Desired velocity vector: $(c.des_vel)")
        end

        # Don't move the manipulator
        # c.des_vel[end-3:end] = zeros(4,1)

        if rem(c.step_ctr, c.ctrl_steps) == 0 && c.step_ctr != 0
            # Get forces for vehicle (yaw, surge, sway, heave)
            for dir_idx = 3:6
                # TODO: Need to see if actual_vel needs to be incremented via c.joint_vec[dir_idx]                
                actual_vel = velocity(state, c.joint_vec[1])

                ctlr_tau = PID_ctlr(torques[dir_idx][1], t, actual_vel[dir_idx], dir_idx, c)
                c_taus[dir_idx] = ctlr_tau 
                torques[dir_idx] = ctlr_tau
            end
            
            # Get torques for the arm joints
            for jt_idx in 2:length(c.joint_vec) # Joint index (1:vehicle, 2:baseJoint, etc)
                idx = jt_idx+5 # velocity index (7 to 10)

                # TODO: Need to see if actual_vel needs to be incremented via c.joint_vec[jt_idx]                
                actual_vel = velocity(state, c.joint_vec[jt_idx])

                ctlr_tau = PID_ctlr(torques[idx][1], t, actual_vel[idx], idx, c) 
                torques[velocity_range(state, c.joint_vec[jt_idx])] .= [ctlr_tau] 
                c_taus[idx] = ctlr_tau 
            end
            #TODO switch to push! ?
            c.taus = cat(c.taus, c_taus, dims=2)
            # push!(c.taus, copy(c_taus))
        end
    end
    if rem(c.step_ctr, 4000) == 0
        println("At time $(c.step_ctr/4000)...")
    end
    c.step_ctr = c.step_ctr + 1
end;

"""
Imposes a PID controller on one joint. 

`torque` = storage variable, does not modify
`t` = current time. Not in use. 
`vel_act` = the actual instantaneous velocity of the joint 
`des_vel` = the desired velocity of the joint, as found by the trajectory generator
`j_idx` = the index of the joint (of the actuated ones). Here, will be 1 or 2.

Returns a controller torque value, bounded by the torque limits and some dτ/dt value. 
"""
function PID_ctlr(torque, t, vel_act, idx, c)
    actuated_idx = idx-2
    d_vel = c.des_vel[actuated_idx]
    vel_error = vel_act[1] - d_vel
    d_vel_error = (vel_error - c.vel_error_cache[actuated_idx])/(1/c.ctrl_freq)
    # println("D_Velocity error on idx$(j_idx): $(d_vel_error)")
    c.vel_int_error_cache[actuated_idx] = c.vel_int_error_cache[actuated_idx] + vel_error*(1/c.ctrl_freq)
    d_tau = -Kp[actuated_idx]*vel_error - Kd[actuated_idx]*d_vel_error - Ki[actuated_idx]*c.vel_int_error_cache[actuated_idx]
    # println("Ideal tau: $(d_tau)")

    # Can only change torque a small amount per time step 
    # arm joints can change faster than thrusters
    5 <= actuated_idx ? lim = 0.0001 : lim = 0.001
    lim = lim*(1/c.time_step)/c.ctrl_freq
    d_tau = limit_d_tau(d_tau, lim)
    
    # Torque limits
    #TODO see if this is actually limiting the torque change
    new_tau = torque .+ d_tau
    impose_torque_limit!(new_tau, vel_act[actuated_idx])

    # # store velocity error term
    c.vel_error_cache[actuated_idx]=vel_error
    # c.vel_int_error = c.vel_int_error + vel_error

    return new_tau
end