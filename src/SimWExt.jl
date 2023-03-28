using RigidBodyDynamics.OdeIntegrators
import RigidBodyDynamics: default_constraint_stabilization_gains, MechanismState
import RigidBodyDynamics
import RigidBodyDynamics.cache_eltype

function simulate_with_ext_forces(state0::MechanismState{X}, final_time, hydro_calc!;
        Δt = 1e-4, stabilization_gains=default_constraint_stabilization_gains(X)) where X 
        T = cache_eltype(state0)
    result = DynamicsResult{T}(state0.mechanism)
    control_torques = similar(velocity(state0))
    hydro_wrenches = Dict{BodyID, Wrench{Float64}}()
    closed_loop_dynamics! = let result=result, hydro_wrenches=hydro_wrenches, control_torques=control_torques, stabilization_gains=stabilization_gains # https://github.com/JuliaLang/julia/issues/15276
        function (v̇::AbstractArray, ṡ::AbstractArray, t, state)

            hydro_calc!(hydro_wrenches, t, state)

            # control!(control_torques, t, state, pars, ctlr)

            dynamics!(result, state, control_torques, hydro_wrenches; stabilization_gains=stabilization_gains)
            
            copyto!(v̇, result.v̇)
            copyto!(ṡ, result.ṡ)
            nothing
        end
    end

tableau = runge_kutta_4(T)
storage = ExpandingStorage{T}(state0, ceil(Int64, final_time / Δt * 1.001)) # very rough overestimate of number of time steps
integrator = MuntheKaasIntegrator(state0, closed_loop_dynamics!, tableau, storage)
integrate(integrator, final_time, Δt)
storage.ts, storage.qs, storage.vs
end


function simple_simulate_with_ext_forces(state0::MechanismState{X}, final_time, hydro_calc!, control! = zero_torque!;
        Δt = 1e-4, stabilization_gains=default_constraint_stabilization_gains(X)) where X 
        T = cache_eltype(state0)
    result = DynamicsResult{T}(state0.mechanism)
    control_torques = similar(velocity(state0))
    hydro_wrenches = Dict{BodyID, Wrench{Float64}}()
    closed_loop_dynamics! = let result=result, hydro_wrenches=hydro_wrenches, control_torques=control_torques, stabilization_gains=stabilization_gains # https://github.com/JuliaLang/julia/issues/15276
        function (v̇::AbstractArray, ṡ::AbstractArray, t, state)

            hydro_calc!(hydro_wrenches, t, state)

            # println("made it here")

            control!(control_torques, t, state)
            # set_configuration!(state, joint1, pi/2)
            # set_configuration!(state, joint2, pi)
            # set_configuration!(state, joint3, pi)
            # println("then made it here")

            dynamics!(result, state, control_torques, hydro_wrenches; stabilization_gains=stabilization_gains)
            
            copyto!(v̇, result.v̇)
            copyto!(ṡ, result.ṡ)
            nothing
        end
    end

tableau = runge_kutta_4(T)
storage = ExpandingStorage{T}(state0, ceil(Int64, final_time / Δt * 1.001)) # very rough overestimate of number of time steps
integrator = MuntheKaasIntegrator(state0, closed_loop_dynamics!, tableau, storage)
integrate(integrator, final_time, Δt)
storage.ts, storage.qs, storage.vs
end


function vanilla_pid_sim_ext_force(state0::MechanismState{X}, final_time, pars, ctlr, hydro_calc!, control! = zero_torque!;
    Δt = 1e-4, stabilization_gains=default_constraint_stabilization_gains(X)) where X 
# println("Made it to the simulate function!")
    T = cache_eltype(state0)
result = DynamicsResult{T}(state0.mechanism)
control_torques = similar(velocity(state0))
hydro_wrenches = Dict{BodyID, Wrench{Float64}}()
closed_loop_dynamics! = let result=result, hydro_wrenches=hydro_wrenches, control_torques=control_torques, stabilization_gains=stabilization_gains # https://github.com/JuliaLang/julia/issues/15276
    function (v̇::AbstractArray, ṡ::AbstractArray, t, state)

        hydro_calc!(hydro_wrenches, t, state)

        control!(control_torques, t, state, pars, ctlr)

        dynamics!(result, state, control_torques, hydro_wrenches; stabilization_gains=stabilization_gains)

        copyto!(v̇, result.v̇)
        copyto!(ṡ, result.ṡ)
        nothing
    end
end
tableau = runge_kutta_4(T)
storage = ExpandingStorage{T}(state0, ceil(Int64, final_time / Δt * 1.001)) # very rough overestimate of number of time steps
integrator = MuntheKaasIntegrator(state0, closed_loop_dynamics!, tableau, storage)
integrate(integrator, final_time, Δt)
storage.ts, storage.qs, storage.vs
end


function joint_space_sim(state0::MechanismState{X}, final_time, des_config, hydro_calc!, control! = zero_torque!;
    Δt = 1e-4, stabilization_gains=default_constraint_stabilization_gains(X)) where X 
    T = cache_eltype(state0)
result = DynamicsResult{T}(state0.mechanism)
control_torques = similar(velocity(state0))
hydro_wrenches = Dict{BodyID, Wrench{Float64}}()

mechanism = state.mechanism
world = root_frame(mechanism)
joint_path = path(mechanism, root_body(mechanism), body)
point_world = transform(state, point, root_frame(mechanism))
Jp = point_jacobian(state, joint_path, point_world)

closed_loop_dynamics! = let result=result, hydro_wrenches=hydro_wrenches, control_torques=control_torques, stabilization_gains=stabilization_gains # https://github.com/JuliaLang/julia/issues/15276
    function (v̇::AbstractArray, ṡ::AbstractArray, t, state)

        hydro_calc!(hydro_wrenches, t, state)

        function controller!(control_torques, t, state, des_config)
            # desired = Point3D(world, circle_origin .+ radius .* SVector(sin(t / ω), 0, cos(t / ω)))
            point_in_world = transform_to_root(state, body) * point
            point_jacobian!(Jp, state, joint_path, point_in_world)
            Kp = 200
            Kd = 20
            Δp = desired - point_in_world
            v̇ .= Kp * Array(Jp)' * Δp.v .- Kd .* velocity(state)
            control_torques .= inverse_dynamics(state, v̇)
        end

        dynamics!(result, state, control_torques, hydro_wrenches; stabilization_gains=stabilization_gains)

        copyto!(v̇, result.v̇)
        copyto!(ṡ, result.ṡ)
        nothing
    end
end
tableau = runge_kutta_4(T)
storage = ExpandingStorage{T}(state0, ceil(Int64, final_time / Δt * 1.001)) # very rough overestimate of number of time steps
integrator = MuntheKaasIntegrator(state0, closed_loop_dynamics!, tableau, storage)
integrate(integrator, final_time, Δt)
storage.ts, storage.qs, storage.vs
end