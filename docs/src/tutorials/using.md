# Using a Mechanism

## Compiling 

The [`Mechanism`](@ref) type is designed to be used for building the mechanism, not for performing
fast computations. 
Its internal data structures rely on Abstract types, which makes construction easier, but is 
inherently type unstable.
Once you have built your mechanism it can be "compiled" into a [`CompiledMechanism`](@ref), which
is fully type stable (see [`TypeStableCollection`](@ref) for more info on how this is achieved.).

First, we will load a mechanism: a 3 link robot defined by an RSON file.

```@example 1
using VMRobotControl
mechanism = parseRSON("../../../RSONs/rsons/simple3link.rson")
```

Compiling a mechanism is as simple as 

```@example 1
m = compile(mechanism)
```

This returns the type-stable version of your mechanism.
The compiled mechanism's type stable data structures can be accessed with fast indices, for example 
let us consider the coordinate we added earlier for the link 3 centre of mass `"L3_com"`.

```@example 1
L3_com = get_compiled_coordID(m, "L3_com")
```

This returns the `CompiledCoordID` for the coordinate. Note that [`VMRobotControl.CompiledCoordID`](@ref) 
is a parametric type, so its type varies depending on the type of the coordinate.
We can use this ID on the mechanism to get the compiled coordinate data:

```@example 1
m[L3_com]
```

In this case, the compiled coordinate data has replaced `String` frame ID "L3_frame" with the
compiled frame ID of the compiled frame.

```@example 1
L3_frame_id = get_compiled_frameID(m, "L3_frame")
L3_frame_id == m[L3_com].coord_data.frameID
```

For the full API see [API](@ref)

## Caches and "Cache-Bundles"

To use a mechanism to perform computations we need to create a cache for storing intermediate
computations and results in.
The cache and mechanism are normally needed together, so they are bundled together by default.

There are several functions for creating cache-bundles for different purposes:

| Constructor | Methods | Description |
| :-------------------------------- | :--------------   | :------------------ |
| [`new_kinematics_cache`](@ref)    | [`kinematics!`](@ref) | For transforms/coordinate configurations, used for plotting |
| [`new_jacobians_cache`](@ref)     | [`jacobians!`](@ref) | For kinematics and jacobians |
| [`new_rbstates_cache`](@ref)      | [`velocity_kinematics!`](@ref), [`acceleration_kinematics!`](@ref) | For kinematics and velocities/velocity product accelerations (VPAs) |
| [`new_dynamics_cache`](@ref)      | [`velocity_kinematics!`](@ref), [`precompute!`](@ref), [`inertance_matrix!`](@ref), [`generalized_force!`](@ref), [`dynamics!`](@ref) | For kinematics, velocities, VPAs, jacobians, and inertance matrices/generalized forces, used for simulating |
| [`new_control_cache`](@ref)       | [`control_step!`](@ref) | For realtime control with a virtual-mechanism-system |


For examples of using the kinematics cache for plotting skip to [Plotting with Makie](@ref).
For examples of using the dynamics cache for simulation skip to [Simulating with DifferentialEquations.jl](@ref).

```@example 1
dcache = new_dynamics_cache(m)

t = 0.0
q = zero_q(m)
q̇ = zero_q̇(m)
g = VMRobotControl.DEFAULT_GRAVITY
u = zero_u(m)
dynamics!(dcache, t, q, q̇, g, u)
```

## Accessing results

There are several functions for accessing results from a cache/bundle.

!!! warning

    The contents of a cache are NOT guaranteed to be initialized when it is created. Ensure you call
    the method needed to compute your results before accessing results from the cache.

!!! warning
    Note that some of these results are Vectors/Matrices from the cache, or other heap-allocated 
    objects, that will be updated when you use the cache again. 
    If you are using a result from the cache, and need to use the cache again, make sure to copy the
    results first!

As we have called `dynamics!`, the results in the cache are ready to view. Let's look at some of the
computed values for the compiled frame ID from before.

```@repl 1
get_inertance_matrix(dcache)
get_generalized_force(dcache)
get_transform(dcache, L3_frame_id)
get_linear_vel(dcache, L3_frame_id)
get_angular_vel(dcache, L3_frame_id)
get_linear_vpa(dcache, L3_frame_id)
get_angular_vpa(dcache, L3_frame_id)
get_linear_jacobian(dcache, L3_frame_id)
get_angular_jacobian(dcache, L3_frame_id)
```

However, more often than not you will instead use *coordinates* to access positions/velocities and
jacobians of interest. 

## Coordinate computations

There are four functions to access computation results for coordinates: `configuration`, `velocity`,
`vpa`, and `jacobian`. 
As before, you must make sure the cache is populated with results before calling these functions,
and copy the result elsewhere if you are then going to use the cache again.

```@repl 1
configuration(dcache, L3_com)
velocity(dcache, L3_com)
vpa(dcache, L3_com)
jacobian(dcache, L3_com)
```

Configuration is dependent only on the time and joint state `q`, so can also be used with the
kinematics cache.

## Component computations

TODO

## Modifying a compiled mechanism
 
A compiled mechanism *can* be modified using `Base.setindex!`, by modifying its joints/coordinates/components, only if the 
type of the joint/component does not change. 
This is made simpler by using the `remake` function which allows you to easily remake a compiled
joint/coordinate/component and modify one or more of its fields using keyword arguments.
The interface to do so is like so:
```
using VMRobotControl: remake
m[ee_com] = remake(m[ee_com]; point=SVector(0.3*z))
```

