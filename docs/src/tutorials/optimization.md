# Optimization/Algorithmic Differentiation

Once you have designed a virtual mechanism controller, you will want to tune the parameters.
You can do that by hand or optimize numerically.

Algorithmic Differentiation (AD) is incredibly useful for optimization, as you can easily compute 
a gradient to speed up the search for the best parameters.
There are two tested ways to perform Algorithmic Differentation (AD) with this package.
The first and most reliable is using [`ForwardDiff.jl`](https://github.com/JuliaDiff/ForwardDiff.jl),
and the other using [`Enzyme.jl`](https://github.com/EnzymeAD/Enzyme.jl).

`ForwardDiff` is easier to use but scales poorly with the number of parameters. `Enzyme` is harder 
to use and does not work over the entire interface. However, it can use reverse mode/
backpropagation, so scales extremely well for large numbers of parameters.

This topic is for advanced users, and requires a strong understanding of julia. 

## Using ForwardDiff.jl

As `ForwardDiff.jl` uses dual numbers you will get an error if you try to compute directly.
This is because by default, the any cache created uses the type `Float64`.

```@example 1
using ForwardDiff, VMRobotControl
robot = parseRSON("../../../RSONs/rsons/simple2link.rson")
m = compile(robot)
cache = new_kinematics_cache(m)
t = 0.0
EE_frame = get_compiled_frameID(m, "EE_frame")

try
    results = ForwardDiff.gradient([1.0, 2.0]) do q
        kinematics!(cache, t, q)
        o = origin(get_transform(m, EE_frame))
        return o
    end
catch e
    print(e)
end
nothing
```

Instead, you can wrap the cache in type `EltypeCache`. To access a concrete cache with element type
`T`, use the syntax `eltype_cache[T]`. This will create a cache if one does not already exist, or
return the existing cache.

```@example  1
eltype_cache = let m=m
    # This let block is required. If it is not used, then the type of m is not known.
    # By introducing a hard scope, the type is fixed.
    initializer = (T) -> new_kinematics_cache(m, T)
    VMRobotControl.EltypeCache(initializer)
end

results = ForwardDiff.jacobian([1.0, 2.0]) do q
    T = eltype(q)
    cache = eltype_cache[T]
    kinematics!(cache, t, q)
    o = origin(get_transform(cache, EE_frame))
    return o
end
```

This can also be used in conjunction with `DifferentialEquations.jl`, to differentiate through
a simulation:
```@example 1
using DifferentialEquations

eltype_cache = let m=m
    initializer = (T) -> new_dynamics_cache(m, T)
    VMRobotControl.EltypeCache(initializer)
end

EE_frame = get_compiled_frameID(m, "EE_frame")

results = ForwardDiff.gradient([1.0, 2.0]) do q
    T = eltype(q)
    cache = eltype_cache[T]
    g, q̇, tspan = VMRobotControl.DEFAULT_GRAVITY, zero(q), 5.0
    prob = get_ode_problem(cache, g, q, q̇, tspan, )
    sol = solve(prob)
    
    x = sol(tspan) # Get final state
    dynamics!(cache, tspan, x[q_idxs(m)], x[q̇_idxs(m)], g) # Compute using final state
    o = origin(get_transform(cache, EE_frame))
    return sqrt(o' * o)
end
```

## Using Enzyme.jl

For examples of using `Enzyme` look at `./test/Enzyme_compat_test`. At the time of writing,
this works for nearly everything, except computing inertance matrices, and thus dynamics.

