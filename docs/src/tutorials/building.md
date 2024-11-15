# Building a Mechanism

Let's build a model robot: a 3DOF arm on a 2DOF sliding table, using
2 prismatic joints followed by 2 revolute joints.

We will supplement this intro with pictures/videos, but not present the plotting code here.
To learn more about plotting, you may read the section [Plotting with Makie](@ref).

```@setup 1
using GLMakie
using DifferentialEquations
```

```@example 1
using VMRobotControl
using StaticArrays
using LinearAlgebra
```

To begin we make the mechanism and use [`add_frame!`](@ref) to add the named
rigid-body-frames that we will use.
All the frames must be joined in a tree from the root frame of the mechanism 
which is called `"root_frame"`:

```@example 1
mechanism = Mechanism{Float64}("TableArmRobot")
F0 = root_frame(mechanism)
```

## Adding rigid body frames and joints

Lets add our named frames. Note that `add_frame!` returns the name of the frame
(as a string) for convenience.
```@example 1
F1 = add_frame!(mechanism; id="L1_frame")
F2 = add_frame!(mechanism; id="L2_frame")
F3 = add_frame!(mechanism; id="L3_frame")
F4 = add_frame!(mechanism; id="L4_frame")
F5 = add_frame!(mechanism; id="L5_frame")
F_EE = add_frame!(mechanism; id="EE_frame")
```

Now: how do we define the joints that join these frames together? First we must
define the joint data.
The form of the joint data depends on the type of joint: a [`Rigid`](@ref) joint
needs a [`Transform`] whereas a [`Revolute`](@ref) or [`Prismatic`](@ref) 
joint requires the axis of rotation/translation (and optionally a transform too!).
This is not a minimal representation (such as denavit hartenburg) but it is a 
flexible representation.

```@example 1
# Define the unit vectors to make things easier
X = SVector(1., 0., 0.)
Y = SVector(0., 1., 0.)
Z = SVector(0., 0., 1.)

# First two prismatic joints, one in the X, direction, one in Y
J1 = Prismatic(X)
J2 = Prismatic(Y)
# Then a simple articulated arm: J3 rotates around Z, J4 is the shoulder, 
# and J5 the elbow.
J3 = Revolute(Z)
J4 = Revolute(X)
J5 = Revolute(X, Transform(0.4*Z)) 
# Transform(::SVector{3}) is a translation, so J5 is 40cm above J4.

# Finally define a rigid joint that will connect the elbow to end-effector 
# frame, again 40cm above J5
J_EE = Rigid(Transform(0.4*Z))
```
Now that we have defined the joints, we must add them to the mechanism by 
specifying which frames they each connect, and giving each joint an ID
(a name), using `add_joint!`.

```@example 1
add_joint!(mechanism, J1; parent=F0, child=F1, id="J1")
add_joint!(mechanism, J2; parent=F1, child=F2, id="J2")
add_joint!(mechanism, J3; parent=F2, child=F3, id="J3")
add_joint!(mechanism, J4; parent=F3, child=F4, id="J4")
add_joint!(mechanism, J5; parent=F4, child=F5, id="J5")
add_joint!(mechanism, J_EE; parent=F5, child=F_EE, id="J_EE")
```

Let's inspect our mechanism now:
```@repl 1
mechanism
frames(mechanism)
joints(mechanism)
```

## Adding mass, inertia, and damping

Now, the rigid body tree of our robot is defined. 
However to do simulations we need to give it some mass and inertia.
Mass, inertia, damping and other dynamic effects are achieved by adding *components* to the
mechanism. These are generally added using the function `add_component!`, but there are 
some special helper functions like [`add_inertia!`](@ref), to help in certain cases.

Point masses are defined by their mass, and a coordinate. 
The coordinate should represent the centre of mass, in the root frame of the mechanism.
To do this we use the [`FrameOrigin`](@ref) and [`FramePoint`](@ref) coordinate types.
The `configuration` of a `FrameOrigin` coordinate is the position of the origin of a frame, 
represented in the root frame.
The `configuration` of a `FramePoint` coordinate is the position of a point in a frame,
represented in the root frame.

```@example 1
add_coordinate!(mechanism, FrameOrigin(F2); id="base_centre_of_mass")
add_coordinate!(mechanism, FrameOrigin(F5); id="elbow_centre_of_mass")
add_coordinate!(mechanism, FramePoint(F_EE, 0.2*Z); id="ee_centre_of_mass")

add_component!(mechanism, PointMass(10.0, "base_centre_of_mass"); id="base_mass") 
add_component!(mechanism, PointMass(2.0, "elbow_centre_of_mass"); id="lower_arm_mass") 
add_component!(mechanism, PointMass(1.0, "ee_centre_of_mass"); id="ee_mass") 

I_mat = @SMatrix [
    0.01  0.    0.  ;
    0.    0.01  0.  ;
    0.    0.    0.01
]
add_inertia!(mechanism, F3, I_mat; id="L3_inertia")
```

If there are insufficient point masses/inertias the robot inertance matrix
will be singular, and you will not be able to solve the dynamics.
This happens because at least one of the joints can accelerate without a 
point mass or inertia accelerating, so the computed acceleration of the joint
would be infinite.

```@setup 1
fig = Figure(size=(854, 480))
ls = LScene(fig[1, 1]; show_axis=false)
m = compile(mechanism)
kcache = Observable(new_kinematics_cache(m))
robotsketch!(ls, kcache; linewidth=2.0)
q, q̇, tspan = zero_q(m) .+ 1e-2, zero_q̇(m), 5.0
sol = solve(get_ode_problem(new_dynamics_cache(m), VMRobotControl.DEFAULT_GRAVITY, q, q̇, tspan))
animate_robot_odesolution(fig, sol, kcache, "building1.mp4")
nothing
```
![](building1.mp4)

We will now also add some damping to each joint, otherwise our system is undamped and will never
settle.
For this we use the `JointSubspace` coordinate type to define a coordinate for each joint.
The `configuration` of the `JointSubspace` coordinate is the extension of a prismatic joint, or the
angle of a revolute joint.

Then we add a `LinearDamper` using each `JointSubspace` coordinate, so that we have linear damping
acting on each joint.

```@example 1
for i = 1:5
    add_coordinate!(mechanism, JointSubspace("J$i"); id="J$i")
    add_component!(mechanism, LinearDamper(0.5, "J$i"); id="J$(i)_damper")
end
```

```@setup 1
fig = Figure(size=(854, 480))
ls = LScene(fig[1, 1]; show_axis=false)
m = compile(mechanism)
kcache = Observable(new_kinematics_cache(m))
robotsketch!(ls, kcache; linewidth=2.0)
q, q̇, tspan = zero_q(m) .+ 1e-2, zero_q̇(m), 5.0
sol = solve(get_ode_problem(new_dynamics_cache(m), VMRobotControl.DEFAULT_GRAVITY, q, q̇, tspan))
animate_robot_odesolution(fig, sol, kcache, "building2.mp4")
nothing
```
![](building2.mp4)


## Gravity compensation

First we will add gravity compensation. Gravity compensation works by applying a constant force
equal to the weight of each link at its centre of mass. This is made easy by the 
[`GravityCompensator`](@ref) component, and the [`add_gravity_compensation!`](@ref).

```@example 1
add_gravity_compensation!(mechanism, VMRobotControl.DEFAULT_GRAVITY)
```

!!! note
    For gravity compensation, you should typically add the component to the virtual mechanism system,
    as when using `control_step!`, all components on the *robot* are treated as part of the robot
    model, not part of the controller, so forces due to them are not applied by the controller. 

```@setup 1
fig = Figure(size=(854, 480))
ls = LScene(fig[1, 1]; show_axis=false)
m = compile(mechanism)
kcache = Observable(new_kinematics_cache(m))
robotsketch!(ls, kcache; linewidth=2.0)
q, q̇, tspan = zero_q(m) .+ 1e-2, zero_q̇(m), 5.0
sol = solve(get_ode_problem(new_dynamics_cache(m), VMRobotControl.DEFAULT_GRAVITY, q, q̇, tspan))
animate_robot_odesolution(fig, sol, kcache, "building3.mp4")
nothing
```
![](building3.mp4)


## End effector impedance control

Finally we will add a linear spring and damper between the end-effector and a reference position.
First we define a coordinate for the end-effector position


```@example 1
refpos = -0.5*X + -0.5*Y + 0.5*Z
add_coordinate!(mechanism, FrameOrigin(F_EE); id="ee_position")
add_coordinate!(mechanism, ConstCoord(refpos); id="ref")
add_coordinate!(mechanism, CoordDifference("ee_position", "ref"); id="ee_error")

ee_stiffness = SMatrix{3, 3}(100.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 100.0)
ee_damping = SMatrix{3, 3}(5.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 5.0)

add_component!(mechanism, LinearSpring(ee_stiffness, "ee_error"); id="ee_spring")
add_component!(mechanism, LinearDamper(ee_damping, "ee_error"); id="ee_damper")
```

Note that the same `LinearDamper` type is used here as for the joint damping. This is because the 
coordinate system handles all the geometry! As long as the damping coefficient can multiply the 
velocity, the `LinearDamper` will work, so we can use a 3x3 matrix as shown here, or a scalar as done
for joint damping.

```@setup 1
fig = Figure(size=(854, 480))
ls = LScene(fig[1, 1]; show_axis=false)
m = compile(mechanism)
kcache = Observable(new_kinematics_cache(m))
robotsketch!(ls, kcache; linewidth=2.0)

scatter!(ls, refpos; color=:red, markersize=10)

q, q̇, tspan = zero_q(m) .+ 1e-2, zero_q̇(m), 5.0
sol = solve(get_ode_problem(new_dynamics_cache(m), VMRobotControl.DEFAULT_GRAVITY, q, q̇, tspan))
animate_robot_odesolution(fig, sol, kcache, "building4.mp4")
nothing
```
![](building4.mp4)
