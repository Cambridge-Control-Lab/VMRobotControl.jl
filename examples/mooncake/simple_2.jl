using DifferentialEquations
using DifferentiationInterface
using Mooncake
using LinearAlgebra
using VMRobotControl
using SciMLSensitivity
using StaticArrays

cfg = RSONParserConfig(; parse_visuals=false)
mechanism = parseRSON("./RSONs/rsons/pendulum.rson")
add_coordinate!(mechanism, ConstCoord(SVector(0., 0., 0.)); id="target")
add_coordinate!(mechanism, CoordDifference("EE_coord", "target"); id="spring_ext")
add_component!(mechanism, LinearSpring(100.0, "spring_ext"); id="spring")
m = compile(mechanism)

dcache = new_dynamics_cache(m)
tspan = (0.0, 1.0)
q0 = [0.,]
q̇0 = [1.,]
gravity = VMRobotControl.DEFAULT_GRAVITY
p = [1.0, 2.0, 3.0]

function f_setup_2(cache)
    id = get_compiled_coordID(cache, "target")
    (id,)
end

function f_control_2(cache, _, args, extra)
    id = args[1]
    (_, _, p) = extra
    new_coord = remake(cache[id]; coord_data=ConstCoord(SVector(p[1], p[2], p[3])))
    cache[id] = new_coord 
end

f = let
    t = 0
    dx = [0., 0.]
    x = [0., 0.]
    gravity = SVector(0., 0., 0.)
    f2 = get_ode_dynamics(dcache, SVector(0., 0., 0.); f_setup=f_setup_2, f_control=f_control_2)
    f(p) = begin 
        f2(dx, x, p, t)
        dx
    end
end

p = [3.0, 3.0, 3.0]
@show f(p)
@show f(p + [0.01, 0.01, 0.01])
backend = AutoMooncake(; config=nothing)
prep = prepare_jacobian(f, backend, p)
@show DifferentiationInterface.jacobian(f, prep, backend, p)
