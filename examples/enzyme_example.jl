using DifferentialEquations
using Enzyme
using LinearAlgebra
using VMRobotControl
using SciMLSensitivity
using StaticArrays

cfg = RSONParserConfig(; parse_visuals=false)
mechanism = parseRSON("./RSONs/rsons/scara.rson")
m = compile(mechanism)

dcache = new_dynamics_cache(m)
tspan = (0.0, 1.0)
q0 = [0., 0.]
q̇0 = [1., 1.]
gravity = VMRobotControl.DEFAULT_GRAVITY

f_control = (cache, t, args, extra) -> begin
    q̇ = get_q̇(cache)
    u = get_u(cache)
    (dx, x, p) = extra
    u .= -p .* q̇
    nothing
end

f = get_ode_dynamics(dcache, gravity; f_setup, f_control)
x0 = vcat(q0, q̇0)
p = [1.0, 1.0]
prob = ODEProblem(f, x0, tspan, p)
sol = solve(prob, Tsit5())
dCdx(out, x, p, t) = x

lines(sol)

SciMLSensitivity.STACKTRACE_WITH_VJPWARN[] = true
@time res = adjoint_sensitivities(sol, Tsit5(); dgdu_continuous=dCdx, sensealg=InterpolatingAdjoint(;autojacvec=EnzymeVJP()))
@time res = adjoint_sensitivities(sol, Tsit5(); dgdu_continuous=dCdx, sensealg=InterpolatingAdjoint(;autojacvec=EnzymeVJP()))

