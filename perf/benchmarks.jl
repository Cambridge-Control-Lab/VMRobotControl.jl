# using Pkg
# Pkg.activate(@__DIR__)
# Pkg.instantiate()

using Revise

using BenchmarkTools
# using CairoMakie
using DifferentialEquations
using ForwardDiff
using ForwardDiff: Dual, Tag, Partials
import ForwardDiff: ≺
using GLMakie
using LinearAlgebra
using Random
using VMRobotControl
using StaticArrays

struct TestTag end
≺(a::TestTag, b) = true
≺(a, b::TestTag) = false

N_DUAL_BENCHMARKS = 5

function create_franka()
    rson = "./RSONs/rsons/franka_panda/pandaSurgical.rson"
    parseRSON(rson)
end

function runbenchmarks(suite)
    overhead = BenchmarkTools.estimate_overhead()
    Random.seed!(1)
    results = run(suite, verbose=true, overhead=overhead, gctrial=false)
    for result in results
        println("$(first(result)):")
        display(last(result))
        println()
    end
    results
end

###############################
# Benchmarks
###############################

function create_transforms_benchmarks(::Type{T}) where T
    suite = BenchmarkGroup()
    suite["transform*transform"] = @benchmarkable(begin
        tf*tf
    end, setup = begin
        tf = rand(Transform{$T})
    end, evals = 10)
    suite["transform*vector"] = @benchmarkable(begin
        tf * v
    end, setup = begin
        tf = rand(Transform{$T})
        v = rand(SVector{3, $T})
    end, evals = 10)
    suite["rotor*rotor"] = @benchmarkable(begin
        r*r
    end, setup = begin
        r = rand(Rotor{$T})
    end, evals = 10)
    suite["rotor*vector"] = @benchmarkable(begin
        r*v
    end, setup = begin
        r = rand(Rotor{$T})
        v = rand(SVector{3, $T})
    end, evals = 10)
    suite
end

function create_kinematics_benchmarks(immut_mechanism)
    suite = BenchmarkGroup()
    
    t = 0.0
    q = rand(Float64, config_size(immut_mechanism))
    q̇ = rand(Float64, velocity_size(immut_mechanism))
    gravity = VMRobotControl.DEFAULT_GRAVITY

    suite["kinematics"] = @benchmarkable(begin
        kinematics!(cache, $t, $q)
    end, setup = begin
        cache = new_kinematics_cache($immut_mechanism)
    end, evals = 10)

    suite["velocity_kinematics"] = @benchmarkable(begin
        velocity_kinematics!(cache, $t, $q, $q̇)
    end, setup = begin
        cache = new_rbstates_cache($immut_mechanism)
    end, evals = 10)

    suite["jacobians"] = @benchmarkable(begin
        jacobians!(cache, $t, $q)
    end, setup = begin
        cache = new_jacobians_cache($immut_mechanism)
    end, evals = 10)

    suite["precomp"] = @benchmarkable(begin
        precompute!(cache, $t, $q, $q̇, $gravity)
    end, setup = begin
        cache = new_dynamics_cache($immut_mechanism)
    end, evals = 10)


    # suite["hessianresult"] = @benchmarkable(begin
    #     VMRobotControl.compute_hessianresult($immut_mechanism, q)
    # end, setup = begin
    #     q = rand($q_type)
    # end, evals = 10)

    suite
end

function create_components_benchmarks(immut_mechanism)
    suite = BenchmarkGroup()

    t = 0.0
    q = rand(Float64, config_size(immut_mechanism))
    q̇ = rand(Float64, velocity_size(immut_mechanism))
    gravity = VMRobotControl.DEFAULT_GRAVITY
    cache = new_dynamics_cache(immut_mechanism)
    precompute!(cache, t, q, q̇, gravity)

    suite["inertance_matrix"] = @benchmarkable(begin
        inertance_matrix($cache)
    end, evals = 10)

    suite["stored_energy"] = @benchmarkable(begin
        stored_energy($cache)
    end, evals = 10)
    
    suite["generalized_force"] = @benchmarkable(begin
        generalized_force($cache)
    end, evals = 10)
    suite
end

function create_dynamics_benchmarks(immut_mechanism)
    suite = BenchmarkGroup()
    q_type = q̇_type = u_type = SVector{7, Float64}
    t = 0.0
    q = rand(Float64, config_size(immut_mechanism))
    q̇ = rand(Float64, velocity_size(immut_mechanism))
    u = rand(Float64, velocity_size(immut_mechanism))
    gravity = VMRobotControl.DEFAULT_GRAVITY

    suite["dynamics"] = @benchmarkable(begin
        dynamics!(cache, $t, $q, $q̇, $gravity, $u)
    end, setup = begin
        cache = new_dynamics_cache($immut_mechanism)
    end, evals = 10)

    suite["dynamics_rne!"] = @benchmarkable(begin
        VMRobotControl.dynamics_rne!(cache, $t, $q, $q̇, $gravity, $u)
    end, setup = begin
        cache = new_rne_cache($immut_mechanism)
    end, evals = 10)

    suite
end

function create_simulation_benchmarks()
    suite = BenchmarkGroup()
    gravity = VMRobotControl.DEFAULT_GRAVITY

    suite["simulate"] = @benchmarkable(begin
        solve(prob, Tsit5(); save_everystep=false)
    end, setup = begin
        mechanism = create_franka()
        immut_mechanism = compile(mechanism)
        q = rand(Float64, config_size(immut_mechanism))
        q̇ = rand(Float64, velocity_size(immut_mechanism))    
        cache = new_dynamics_cache(immut_mechanism)
        prob = ODEProblem{false}(
            get_ode_dynamics(cache, $gravity), 
            vcat(q, q̇), 
            1.0
        ) 
    end, evals = 10)

    suite["simulate_with_spring"] = @benchmarkable(begin
    solve(prob, Tsit5(); save_everystep=false)
    end, setup = begin
        mechanism = create_franka()
        add_coordinate!(mechanism, FrameOrigin("robot_EE_frame"); id="ext")
        add_component!(mechanism, LinearSpring(3000.0, "ext"); id="spring" )
        immut_mechanism = compile(mechanism)
        q = rand(Float64, config_size(immut_mechanism))
        q̇ = rand(Float64, velocity_size(immut_mechanism))    
        cache = new_dynamics_cache(immut_mechanism)
        prob = get_ode_problem(cache, $gravity, q, q̇, 1.0) 
    end, evals = 10)

    suite["simulate_with_springdamper"] = @benchmarkable(begin
    solve(prob, Tsit5(); save_everystep=false)
    end, setup = begin
        mechanism = create_franka()
        push!(mechanism, LinearSpring(3000.0, FrameOrigin("robot_EE_frame")))
        push!(mechanism, LinearDamper(30.0, FrameOrigin("robot_EE_frame")))
        immut_mechanism = compile(mechanism)
        q = rand(Float64, config_size(immut_mechanism))
        q̇ = rand(Float64, velocity_size(immut_mechanism))    
        cache = new_dynamics_cache(immut_mechanism)
        prob = get_ode_problem(cache, $gravity, q, q̇, 1.0) 
    end, evals = 10)


    suite
end

# function create_dual_benchmarks(immut_mechanism)
#     suite = BenchmarkGroup()
    
#     q_type = SVector{7, Float64}
#     q̇_type = SVector{7, Float64}
#     gravity = VMRobotControl.DEFAULT_GRAVITY

#     suite["dynamics_dual_0_partial"] = @benchmarkable(begin
#         cache = new_dynamics_cache($immut_mechanism)
#         dynamics!(cache, 0.0, q, q̇, $gravity, u)
#     end, setup = begin
#         q = rand(Float64, config_size(immut_mechanism))
#         q̇ = rand(Float64, velocity_size(immut_mechanism))    
#         u = rand($q̇_type)
#     end, evals = 20)
    
#     tag = TestTag()
#     for i = 1:N_DUAL_BENCHMARKS
#         q_type_dual = SVector{7, Dual{tag, Float64, i}}
#         q̇_type_dual = SVector{7, Dual{tag, Float64, i}}

#         suite["dynamics_dual_$(i)_partial"] = @benchmarkable(begin
#             dynamics!(cache, 0.0, q, q̇, $gravity, u)
#         end, setup = begin
#             q = rand($q_type_dual)
#             q̇ = rand($q̇_type_dual)
#             u = rand($q̇_type_dual)
#             T = eltype($q_type_dual)
#             cache = new_dynamics_cache($immut_mechanism, T)
#         end, evals = 20)
#     end
#     suite
# end

###################################
# Run
###################################

function create_full_benchmark_suite()
    mechanism = create_franka()
    immut_mechanism = compile(mechanism)   
    
    full_suite = BenchmarkGroup()
    full_suite["transforms"] = create_transforms_benchmarks(Float64)
    full_suite["kinematics"] = create_kinematics_benchmarks(immut_mechanism)
    full_suite["components"] = create_components_benchmarks(immut_mechanism)
    full_suite["dynamics"] = create_dynamics_benchmarks(immut_mechanism)
    full_suite["simulation"] = create_simulation_benchmarks()
    # full_suite["dual"] = create_dual_benchmarks(immut_mechanism)
    full_suite
end

suite = create_full_benchmark_suite()
# results = run(suite)

# bm = suite["kinematics"]["velocity_kinematics"]
# bm = suite["dynamics"]["dynamics_rne!"]
bm = suite["dynamics"]["dynamics"]
# bm = suite["dual"]["dynamics_dual_1_partial"]
# bm = suite["dual"]["dynamics_dual_0_partial"]
# bm = suite["simulation"]["simulate_with_spring"]
result = run(bm)
display(result)
# @profview run(bm)

