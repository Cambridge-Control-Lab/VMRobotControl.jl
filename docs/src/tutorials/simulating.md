
# Simulating with DifferentialEquations.jl

## Getting started

To simulate the dynamics of the system, `DifferentialEquations.jl` can be used.
The extension `VMRobotControlDifferentialEquationsExt.jl` is loaded when both `VMRobotControl.jl` and
`DifferentialEquation.jl` are loaded.

```@example 1
using DifferentialEquations
using VMRobotControl
using StaticArrays
```

`DifferentialEquations.jl` expects a function with the signature `f(du, u, p, t)`, where `u` is the 
state, `t` is the current time, `p` is an optional vector of parameters (which we can ignore), and
the answer is returned by modifying `du` in-place, to hold the derivative of `u`.
The state is composed of the configurations and velocities of each joint, and the
`du` is composed of the velocities and accelerations.

The function `get_ode_dynamics(dynamics_cache, gravity; [f_setup, f_control])` returns a function 
with the correct call signature. The dynamics cache will be mutated while simulating, so take care
to avoid memory aliasing if your are multithreading. The optional functions `f_setup` and 
`f_control` are called before starting and at every time step respectively. 
Their type signatures can be seen by looking at the help for `DEFAULT_F_SETUP` and 
`DEFAULT_F_CONTROL`.

`get_ode_dynamics` works the same way for mechanisms and virtual mechanism systems.

```@example 1
mechanism = parseRSON("../../../RSONs/rsons/franka_panda/pandaSurgical.rson")

m = compile(mechanism)
dcache = new_dynamics_cache(m)
gravity = SVector{3, Float64}(0.0, 0.0, -10.0)

f_dynamics = get_ode_dynamics(dcache, gravity;)
```

Once we have the dynamics function, we can make an ODE problem, but we need to define the initial 
state vector. 
We can use two functions to convert between `q`, `q̇` and a state vector `u`: `assemble_state`
and `state_idxs`.
The usage of these functions differs slightly depending on if you are working with a mechanism or
virtual mechanism system, as you will have an extra configuration and velocity vector for the virtual
mechanism if you are working with a virtual mechanism system.

```@example 1
q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
q̇ = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
tspan = 5.0

u0 = assemble_state(m, q, q̇)
prob = ODEProblem(f_dynamics, u0, tspan)
sol = solve(prob, Tsit5())
sol.retcode
```

Alternatively you can the step of defining `f_dynamics`, and use the function `get_ode_problem` from
to do everything in one go:
```@example 1
prob = get_ode_problem(dcache, gravity, q, q̇, tspan)
sol = solve(prob, Tsit5())
sol.retcode
```

## Using `f_setup` and `f_control`

Two callbacks, `f_setup` and `f_control` can be provided when simulating. The first runs once before the simulation starts. The second runs once per step. The examples section demonstrates their use and signatures.