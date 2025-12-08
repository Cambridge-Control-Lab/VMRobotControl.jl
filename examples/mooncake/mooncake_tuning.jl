using DifferentialEquations
using GLMakie
using LinearAlgebra
using Mooncake
using Optimization
using OptimizationPolyalgorithms
using VMRobotControl
using SciMLSensitivity
using StaticArrays

# Load pendulum mechanism
cfg = RSONParserConfig(; parse_visuals=false)
mechanism = parseRSON("./RSONs/rsons/scara.rson")

add_coordinate!(mechanism, ConstCoord(SVector(0.0, 0.0, 0.0)); id="target")
add_coordinate!(mechanism, CoordDifference("EE", "target"); id="error")
add_component!(mechanism, LinearSpring(100.0, "error"); id="spring")
add_component!(mechanism, LinearDamper(100.0, "EE"); id="damper")

m = compile(mechanism)

# Setup parameters
tspan = (0.0, 0.0)
q0 = [0., 0.] # Initial joint angle
q̇0 = [1., 1.] # Initial joint velocity
gravity = VMRobotControl.DEFAULT_GRAVITY
p = [1.0, 1.0, 1.0, 1.0] # Parameters to be tuned

# Setup funciton, gets end-effector coord id
f_setup(cache) = begin
    coord_id = get_compiled_coordID(cache, "target")
    spring_id = get_compiled_componentID(cache, "spring")
    damper_id = get_compiled_componentID(cache, "damper")
    (;coord_id, spring_id, damper_id) 
end

# Control function, called once per loop, directly modifies torques 'u'
# based on parameters 'p'.
f_control = (cache, t, args, extra) -> begin
    (_, _, p) = extra
    (;coord_id, spring_id, damper_id) = args
    coords = coordinates(cache)
    coords[coord_id] = remake(coords[coord_id]; coord_data=ConstCoord(SVector(p[2], p[3], p[4])))
    nothing
end

# Create the dynamics cache used for computations.
cache = new_dynamics_cache(m)
# Create the closure for the ODE dynamics, which uses the cache.
f = get_ode_dynamics(cache, gravity; f_setup, f_control)

# Setup the ode problem, and solve it.
x0 = vcat(q0, q̇0)
prob = ODEProblem(f, x0, tspan, p)
sol = solve(prob, Tsit5(); p=p,) 


# Setup cost functions to minimize.
g(x, p, t) = (sum(x.^2)./2)
function dg(out, x, p, t)
    out[1] = x[1]
    out[2] = x[2]
end

# Setup loss function to minimize
function loss(p, _)
    sol = solve(prob, Tsit5(), p = p,) 
    loss = mapreduce(x -> g(x, nothing, nothing), +, sol.u)
    return loss
end

function loss(p)
    loss(p, nothing)
end

# Setup gradient of loss function
function dloss(G, p, _) 
    sol = solve(prob, Tsit5(), p = p,)
    res = adjoint_sensitivities(
        sol, Tsit5(); dgdu_continuous=dg, g=g,
        sensealg=InterpolatingAdjoint(;autojacvec=SciMLSensitivity.MooncakeVJP())
    )
    _, adjoint = res
end

callback = function (state, l)
    println("Loss: $(l) \tParameters: $(state.u)")
    pred = solve(prob, Tsit5(), p=state.u, )
    plt = lines!(ax, pred; alpha=0.01)
    return halt = false
end

adtype = Optimization.AutoMooncake()
optf = Optimization.OptimizationFunction(loss; grad=dloss)
optprob = Optimization.OptimizationProblem(optf, p)

result_ode = Optimization.solve(optprob, PolyOpt(),
    callback = callback,
    maxiters = 1000)

