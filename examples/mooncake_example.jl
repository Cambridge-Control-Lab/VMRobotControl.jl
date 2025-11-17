using DifferentialEquations
using Mooncake
using LinearAlgebra
using VMRobotControl
using SciMLSensitivity
using StaticArrays

cfg = RSONParserConfig(; parse_visuals=false)
mechanism = parseRSON("./RSONs/rsons/pendulum.rson")
m = compile(mechanism)

dcache = new_dynamics_cache(m)
tspan = (0.0, 1.0)
q0 = [1.,]
q̇0 = [0.,]
gravity = VMRobotControl.DEFAULT_GRAVITY

f_control = (cache, t, args, extra) -> begin
    q̇ = get_q̇(cache)
    u = get_u(cache)
    (dx, x, p) = extra
    u .= -p .* q̇
    nothing
end

# f = get_ode_dynamics(dcache, gravity; f_control)
f2(dx, x, p, t) = begin
    dx[1] = x[2]
    dx[2] = -p[1] * x[2]
    nothing
end
x0 = vcat(q0, q̇0)
p = [1.0,]
prob = ODEProblem(f2, x0, tspan, p)
sol = solve(prob, Tsit5())
g(x, p, t) = p
dgdu(out, u, p, t) = (out .= u)
dgdp(out, u, p, t) = (out .= 0*p)

SciMLSensitivity.STACKTRACE_WITH_VJPWARN[] = true
for i = 1:4
    @time global res = adjoint_sensitivities(sol, Tsit5();
                                             g,
                                             dgdu_continuous=dgdu, 
                                             dgdp_continuous=dgdp,
                                             sensealg=InterpolatingAdjoint(;autojacvec=SciMLSensitivity.MooncakeVJP()))
end
res
