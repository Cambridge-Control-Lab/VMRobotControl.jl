# Virtual Model Control of a Real Robot using ROS2

This library is not just for simulation, but can be used for realtime control of torque-controlled
robots. The library aims to avoid any allocations so that it can be used in tight loops without
invoking the garbage collector.

The library provides an interface for realtime control via the two methods:
- [`new_control_cache`](@ref) and
- [`control_step!`](@ref)
New control cache creates a cache bundle: a structure containing the mechanism and a cache for 
storing intermediate computations.

The step function should be called once per loop, and needs to be provided with the current state:
the time `t`, joint configurations `q`, and joint velocities `q̇`.
Then the effect of the virtual mechanism system is applied in two steps:
- The virtual mechanism advances in time to `t` (by taking one euler step)
- The new demanded robot torques are returned

Any component added to the robot is treated as part of the model of the robot, not as part of the 
controller. Therefore, any component added directly to the robot is ignored by `control_step!` and 
does not generate any forces/torques.
If you wish to create a components that acts upon the robot, it must be added to the 
`VirtualMechanismSystem` and may use coordinates from the robot by prefixing the id with `".robot."`.

For example, if I have a robot with coordinate `"tip_error"`, I can create a spring using coordinate
`"tip_error"` by using `".robot.tip_error"`:
```
vms = VirtualMechanismSystem("MyVMS", robot, virtual_mechanism)
spring = LinearSpring(1000.0, ".robot.tip_error")
add_component!(vms, spring; id="Tip spring")
```
If instead I did
```
spring = LinearSpring(1000.0, "tip_error")
add_component!(robot, spring; id="Tip spring")
vms = VirtualMechanismSystem("MyVMS", robot, virtual_mechanism)
```
then the controller will not apply a force due to the spring, because it thinks the spring already 
exists and is part of the actual robot.

# Implementation tips

We run on a realtime patch of linux, which is required for the robot we control.
We have had the most success using UDP sockets for inter-process-communication using the `Sockets`
library.

We recommend creating a function that takes the cache-bundle from `new_control_cache`, and 
additional arguments `f_setup` and `f_control`. These are functioned called before starting and 
called every loop respectively. They should have the function signature described in the 
documentation for [`get_ode_dynamics`](@ref), so that the same functions can be used for simulation
and control. Additional information only available during control can be provided to `f_control` in
the argument `extra`.