module VMRobotControlDifferentialEquationsExt

import VMRobotControl: get_ode_problem

using DifferentialEquations: ODEProblem, Tsit5, solve
using DifferentialEquations.SciMLBase: NullParameters
using Observables
using VMRobotControl
using VMRobotControl: 
    CompiledMechanism,
    MechDynamicsBundle,
    VMSDynamicsBundle
using StaticArrays

function get_ode_problem(
        m::MechDynamicsBundle, 
        gravity::SVector{3},
        q::Vector{T}, 
        q̇::Vector{T},                  
        tspan, 
        p=NullParameters(); 
        f_setup=VMRobotControl.DEFAULT_F_SETUP,
        f_control=VMRobotControl.DEFAULT_F_CONTROL,
        kwargs...
    ) where T
    f_dynamics = get_ode_dynamics(m, gravity; f_setup, f_control)
    x0 = assemble_state(m.mechanism, q, q̇)
    ODEProblem(f_dynamics, x0, tspan, p; kwargs...)    
end

function get_ode_problem(
        m::VMSDynamicsBundle, 
        gravity::SVector{3},
        q::Tuple{Vector{T}, Vector{T}}, 
        q̇::Tuple{Vector{T}, Vector{T}},                 
        tspan, 
        p=NullParameters(); 
        f_setup=VMRobotControl.DEFAULT_F_SETUP,
        f_control=VMRobotControl.DEFAULT_F_CONTROL,
        kwargs...
    ) where T
    f_dynamics = get_ode_dynamics(m, gravity; f_setup, f_control)
    x0 = assemble_state(m.vms, q, q̇)
    ODEProblem(f_dynamics, x0, tspan; kwargs...)
end

end
