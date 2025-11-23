using DifferentialEquations
using ForwardDiff
using Mooncake
using LinearAlgebra
using QuadGK
using VMRobotControl
using SciMLSensitivity
using StaticArrays

cfg = RSONParserConfig(; parse_visuals=false)
mechanism = parseRSON("./RSONs/rsons/pendulum.rson")
m = compile(mechanism)

dcache = new_dynamics_cache(m)
tspan = (0.0, 1.0)
q0 = [1.,]
q̇0 = [1.,]
gravity = VMRobotControl.DEFAULT_GRAVITY

f_control = (cache, t, args, extra) -> begin
    q̇ = get_q̇(cache)
    u = get_u(cache)
    (dx, x, p) = extra
    u .= -p .* q̇
    nothing
end

f = get_ode_dynamics(dcache, gravity; f_control)
x0 = vcat(q0, q̇0)
p = [1.0,]
prob = ODEProblem(f, x0, tspan, p)
sol = solve(prob, Tsit5())

# Cost functions
g(x, p, t) = (sum(x).^2)./2
function dg(out, x, p, t)
    out[1] = x[1] + x[2]
    out[2] = x[1] + x[2]
end

for i = 1:3
    # Run 3 times, third time all compilation is done and we can
    # see how much time it actually takes to run.
    @time global res = adjoint_sensitivities(
        sol, Tsit5(); dgdu_continuous=dg, g=g,
        sensealg=InterpolatingAdjoint(;autojacvec=SciMLSensitivity.MooncakeVJP())
    )
end
@show res

# Now do with finite diff to compare result (will only be numerically accurate 
# for small examples, low timespans).
ϵ = 1e-8
@show (G(p .+ ϵ) - G(p))/ϵ
@show res
