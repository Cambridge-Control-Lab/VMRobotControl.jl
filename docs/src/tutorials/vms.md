# Virtual Model Control of a Simulated Robot

## What is a virtual mechanism system?

A virtual mechanism system is contains a model of the robot, a model of a virtual mechanism and 
interface components that act between the robot and the virtual mechanism.
It can be used for two purposes: either for simulation or for control of a real robot.
If used for simulation, a full robot model is needed, with dynamic components (like mass/inertia).
If used for control, the robot model is still needed, but at a minimum only a kinematics model
is needed (frames, joints and coordinates) so no inertial model is nescessary.
When controlling a real robot, *components on the robot are ignored*.

The [`VirtualMechanismSystem`](@ref) thus contains a `robot::Mechanism`, a 
`virtual_mechanism::Mechanism`, which can be built just like a robot,
and also contains it's own `coordinates` and `components` that connect *between*
the robot and virtual mechanism.

## Connecting a spring/damper between a robot and a virtual mechanism.

First, lets build a super simple virtual mechanism system.
It will consist of a 1DOF robot, and a 1DOF virtual mechanism.
A virtual spring and damper will join the two together.

First, we make the robot, virtual mechanism, and virtual mechanism system.

```@example 1
using VMRobotControl, StaticArrays

robot = Mechanism{Float64}("Piston")
add_frame!(robot, "L1")
add_joint!(robot, Prismatic(SVector(1.0, 0.0, 0.0)); parent="root_frame", child="L1", id="J1")
add_coordinate!(robot, FrameOrigin("L1"); id="ee_pos")
virtual_mechanism = deepcopy(robot)
vms = VirtualMechanismSystem("MySystem", robot, virtual_mechanism)
```

Then, we add a coordinate to the virtual mechanism system using [`CoordDifference`](@ref) which
will is the typical way to define a coordinate between the robot and virtual mechanism.
It represents a vector from the tip of the robot end-effector to the tip of the virtual mechanism
end-effector.
To identify robot/virtual mechanism coordinates from the virtual-mechanism-system, the prefixes
`".robot."` or `".virtual_mechanism."` must be used.

```@example 1
add_coordinate!(vms, CoordDifference(".robot.ee_pos", ".virtual_mechanism.ee_pos"); id="ee_err")
add_component!(vms, LinearSpring(100.0, "ee_err"); id="ee_spring")
add_component!(vms, LinearDamper(1.0, "ee_err"); id="ee_damper")
```

We can only add spring/damper like components to the virtual mechanism system, and not inertias/
masses/inertances.
This is because the robot/virtual mechanism both act as admitances (taking forces as input, and 
computing accelerations as outputs), so the interface between them must act as an impedance (taking
motion as input and returning a force).
 
## Using a virtual mechanism system

The interface for using a virtual mechanism system is broadly the same as for using mechanism.
However, joint configurations, velocities and actuation torques must now be tuples, with the 
first entry corresponding to the robot and the second to the virtual mechanism

```@repl 1
m = compile(vms)
t = 0.0
q = zero_q(m)
kcache = new_kinematics_cache(m)
kinematics!(kcache, t, q)
```

Note that q is a tuple 
