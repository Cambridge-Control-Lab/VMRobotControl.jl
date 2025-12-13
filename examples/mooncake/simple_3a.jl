using DifferentialEquations
using DifferentiationInterface
using Mooncake
using Printf
using LinearAlgebra
using VMRobotControl
using SciMLSensitivity
using StaticArrays

# Functions

function f_setup_3(cache)
    spring_id = get_compiled_componentID(cache, "spring")
    damper_id = get_compiled_componentID(cache, "damper")
    target_id = get_compiled_coordID(cache, "target")
    (spring_id, damper_id, target_id)
end

function f_control_3(cache, _, args, extra)
    (spring_id, damper_id, target_id) = args
    (_, _, p) = extra
    new_target = remake(cache[target_id]; coord_data=ConstCoord(SVector(p[1], p[2], p[3])))
    new_spring = remake(cache[spring_id]; stiffness=p[4])
    new_damper = remake(cache[damper_id]; damping=p[5])
    cache[target_id] = new_target
    cache[spring_id] = new_spring
    cache[damper_id] = new_damper
    nothing
end

function train(f, df, p, N, η)
    for i = 1:N
        ∂C∂p = df(p)
        p -= η * ∂C∂p
        C = f(p)
        @printf "C = %5.3f, p=" C
        println(p)
    end
    return p
end

# Create mechanism
cfg = RSONParserConfig(; parse_visuals=false)
mechanism = parseRSON("./RSONs/rsons/pendulum.rson")
add_coordinate!(mechanism, ConstCoord(SVector(0., 0., 0.)); id="target")
add_coordinate!(mechanism, CoordDifference("EE_coord", "target"); id="spring_ext")
add_component!(mechanism, LinearSpring(100.0, "spring_ext"); id="spring")
add_component!(mechanism, LinearDamper(1.0, "spring_ext"); id="damper")
m = compile(mechanism)

# Setup optimisation function
p0 = [1.0, 2.0, 3.0, 100., 1.]

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
prob = ODEProblem(f_dynamics, x0, tspan, p0; sensealg)

f = let dcache=dcache, prob=prob, gravity=gravity, ee_coord_id=ee_coord_id, z_target=z_target
    f(p) = begin
        # Solve ode
        sol = solve(prob, Tsit5(); p)
        # Compute terminal cost
        q_idxs, q̇_idxs = state_idxs(dcache)
        q = sol.u[end][q_idxs]
        q̇ = sol.u[end][q̇_idxs]
        dynamics!(dcache, tspan[end], q, q̇, gravity) # Update coordinates with final q
        z_final = configuration(dcache, ee_coord_id) 
        norm(z_final - z_target) + norm(q̇)
    end
end

@show f(p0)
@show f(p0 + [0.01, 0.01, 0.01, 0.01, 0.01])
println("Preparing gradient")
backend = AutoMooncake(; config=nothing)
@time prep = prepare_gradient(f, backend, p0)
df = (p) -> DifferentiationInterface.gradient(f, prep, backend, p)
@time @show df(p0)
@time @show df(p0)
@time @show df(p0)




