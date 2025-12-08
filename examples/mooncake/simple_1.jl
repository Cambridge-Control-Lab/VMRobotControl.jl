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
m = compile(mechanism)

dcache = new_dynamics_cache(m)
tspan = (0.0, 1.0)
q0 = [0.,]
q̇0 = [1.,]
gravity = VMRobotControl.DEFAULT_GRAVITY
p = [1.0, 2.0, 3.0]

function get_f(cache)
    id = get_compiled_coordID(cache, "target")
    function f_to_differentiate(p)
        # Set const coordinate
        @show new_coord = remake(cache[id]; coord_data=ConstCoord(SVector(p[1], p[2], p[3])))
        cache[id] = new_coord 
        # Compute coordinate
        z = configuration(cache, id)
        # Return coordinate
        sum(z)
    end
    return f_to_differentiate
end

f = get_f(dcache)
@show f(p)

backend = AutoMooncake(; config=nothing)
prep = prepare_gradient(f, backend, p)
@show gradient(f, prep, backend, p)
