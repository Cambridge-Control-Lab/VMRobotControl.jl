using DifferentialEquations
using Enzyme
using LinearAlgebra
using VMRobotControl
using SciMLSensitivity
using StaticArrays

cfg = RSONParserConfig(; parse_visuals=false)
mechanism = parseRSON("./RSONs/rsons/pendulum.rson")
m = compile(mechanism)

dcache = new_dynamics_cache(m)
tspan = (0.0, 1.0)
q0 = [0.,]
q̇0 = [1.,]
gravity = VMRobotControl.DEFAULT_GRAVITY
p = [1.0,]

f_setup(cache) = ()
f_control = (cache, t, args, extra) -> begin
    q̇ = get_q̇(cache)
    u = get_u(cache)
    (dx, x, p) = extra
    u .= -p .* q̇
    nothing
end

f = get_ode_dynamics(dcache, gravity; f_setup, f_control)
x0 = vcat(q0, q̇0)
prob = ODEProblem(f, x0, tspan, p)
sol = solve(prob, Tsit5())
g(x, p, t) = (sum(x).^2)./2
function dg(out, x, p, t)
    out[1] = x[1] + x[2]
    out[2] = x[1] + x[2]
end

SciMLSensitivity.STACKTRACE_WITH_VJPWARN[] = true
@time res = adjoint_sensitivities(sol, Tsit5(); g=g, dgdu_continuous=dg, sensealg=InterpolatingAdjoint(;autojacvec=EnzymeVJP()))
@time res = adjoint_sensitivities(sol, Tsit5(); g=g, dgdu_continuous=dg, sensealg=InterpolatingAdjoint(;autojacvec=EnzymeVJP()))

