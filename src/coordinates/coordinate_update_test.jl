using VMRobotControl

rson_parser_config = RSONParserConfig(;parse_visuals=false)
# robot = parseRSON("./RSONs/rsons/franka_panda/panda_kinematics.rson", rson_parser_config)
robot = parseRSON("./RSONs/rsons/franka_panda/pandaSurgical.rson", rson_parser_config)
robot = parseRSON("./RSONs/rsons/all_coords.rson", rson_parser_config)
m = compile(robot);

t = 0.0
q = q̇ = q̈ = u = zeros(ndof(m))
g = VMRobotControl.DEFAULT_GRAVITY

cache = new_kinematics_cache(m)
kinematics!(cache, t, q)
configuration(cache, m(c6))

cache = new_jacobians_cache(m)
jacobians!(cache, t, q)
jacobian(cache, m(c6))

cache = new_rbstates_cache(m)
velocity_kinematics!(cache, t, q, q̇.+2, g)
velocity(cache, m, m(c6))


cache = new_dynamics_cache(m)
precompute!(cache, t, q, q̇, g)
dynamics!(cache, t, q, q̇, g, u)
@profview for i = 1:100000
    dynamics!(cache, t, q, q̇, g, u)
end

configuration(cache.coordinate_cache, compiled_coordinates[1])



tup = (10, 100, 1., 2., 3., Float32(1.), Float16(1.), Int16(1), UInt16(2), (1, 2, 3))

function test(tup, idx, type::Type{T}) where T
    # @inline Core.getfield(tup, idx)::T
    # Core.unsafe_convert(T, Core.getfield(tup, idx))
    # Core.get_binding_type
    ret::T = Base.getindex(tup, idx)
end

function outer_test()
    tup = (10, 100, 1., 2., 3.)
    idx = rand((1, 2))
    # @noinline ret = test(tup, idx, Int)
    ret = test(tup, idx, Float64)
end

@time outer_test()
@code_warntype outer_test()
@code_native outer_test()

@profview for i = 1:10000000
    test(tup, 1, Int)
end


function test4(tup,idx,typ::Type{T}) where T
    idx == 1 && (x=first(tup); return x::T) 
    #or if you want an explicit error
    #isa(x,T) ? (return x) : error("Bla"))
    test4(Base.tail(tup),idx-1,T)
end

@time test4(tup, 1, Int)