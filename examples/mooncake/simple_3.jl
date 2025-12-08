using DifferentialEquations
using DifferentiationInterface
using Mooncake
using LinearAlgebra
using VMRobotControl
using SciMLSensitivity
using StaticArrays

# Functions

function f_setup_3(cache)
    id = get_compiled_coordID(cache, "target")
    (id,)
end

function f_control_3(cache, _, args, extra)
    id = args[1]
    (_, _, p) = extra
    new_coord = remake(cache[id]; coord_data=ConstCoord(SVector(p[1], p[2], p[3])))
    cache[id] = new_coord 
end

# Create mechanism
cfg = RSONParserConfig(; parse_visuals=false)
mechanism = parseRSON("./RSONs/rsons/pendulum.rson")
add_coordinate!(mechanism, ConstCoord(SVector(0., 0., 0.)); id="target")
add_coordinate!(mechanism, CoordDifference("EE_coord", "target"); id="spring_ext")
add_component!(mechanism, LinearSpring(100.0, "spring_ext"); id="spring")
m = compile(mechanism)

# Setup optimisation function
p0 = [1.0, 2.0, 3.0]


f = let
    dcache = new_dynamics_cache(m)
    ee_coord_id = get_compiled_coordID(dcache, "EE_coord")
    tspan = (0.0, 1.0)
    q0 = [0.,]
    q̇0 = [1.,]
    gravity = VMRobotControl.DEFAULT_GRAVITY
    dx = [0., 0.]
    z_target = SVector(1.0, 0.0, 0.0)
    f_dynamics = get_ode_dynamics(dcache, SVector(0., 0., 0.); f_setup=f_setup_3, f_control=f_control_3)
    x0 = vcat(q0, q̇0)
    sensealg=InterpolatingAdjoint(;autojacvec=SciMLSensitivity.MooncakeVJP())
    prob = ODEProblem(f_dynamics, x0, tspan, p; sensealg)
    f(p) = begin
        # Solve ode
        sol = solve(prob, Tsit5(); p)
        # Compute terminal cost
        q_idxs, q̇_idxs = state_idxs(dcache)
        q = sol.u[end][q_idxs]
        q̇ = sol.u[end][q̇_idxs]
        dynamics!(dcache, tspan[end], q, q̇, gravity) # Update coordinates with final q
        z_final = configuration(dcache, ee_coord_id) 
        [norm(z_final - z_target)]
    end
end

p = p0
@show f(p)
@show f(p + [0.01, 0.01, 0.01])
backend = AutoMooncake(; config=nothing)
prep = prepare_jacobian(f, backend, p)
@show DifferentiationInterface.jacobian(f, prep, backend, p)
