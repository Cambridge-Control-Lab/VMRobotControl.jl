# Shadow Coord Example
# The ShadowCoord coordinate mirrors the configuration, velocity and acceleration 
# of another coordinate, without contributing to the Jacobian. This is useful for 
# implementing unidirectional behavior, where the configuration of this coordinate 
# is used to generate an action force on the virtual mechanism system, without any
# reaction force back on the original coordinate. The key difference with a `ConstCoord`
# is that the `ShadowCoord` configuration can change over time, following the
# evolution of the virtual mechanism system. 
# 
# This example shows a very simple example with a linear spring-damper pair linking
# two points free to slide along a prismatic joint. In the first simulation, the coordinate
# of each point is defines as a `FramePoint` coordinate, so the spring-damper pair acts 
# bidirectionally, and both points move towards each other. In the second simulation, 
# one of the points is defined as a `ShadowCoord` of the other point, so the spring-damper 
# pair acts unidirectionally,  and only the second point moves towards the first, while the 
# first point remains fixed.

using LinearAlgebra
using DifferentialEquations
using GLMakie
using StaticArrays

using VMRobotControl

### BUILDING THE SIMPLE 2-POINT ROBOT ###

robot = Mechanism{Float64}("2PointsPrismaticRobot")
F0 = root_frame(robot)
F1 = add_frame!(robot; id="L1_frame")
F2 = add_frame!(robot; id="L2_frame")

J = Prismatic(SVector(1., 0., 0.))

add_joint!(robot, J; parent=F0, child=F1, id="J1")
add_joint!(robot, J; parent=F0, child=F2, id="J2")

add_coordinate!(robot, FrameOrigin(F1); id="f1_centre_of_mass")
add_coordinate!(robot, FrameOrigin(F2); id="f2_centre_of_mass")

add_component!(robot, PointMass(1.0, "f1_centre_of_mass"); id="f1_mass")
add_component!(robot, PointMass(1.0, "f2_centre_of_mass"); id="f2_mass")

### FIRST SIMULATION: BIDIRECTIONAL SPRING-DAMPER ###

# Linkin the two points
vms1 = VirtualMechanismSystem("myVMS1", robot)

add_coordinate!(vms1, CoordDifference(".robot.f1_centre_of_mass", ".robot.f2_centre_of_mass"); id="bidirectional position error");

K = SMatrix{3, 3}(1., 0., 0., 0., 1., 0., 0., 0., 1.)
add_component!(vms1, LinearSpring(K, "bidirectional position error"); id="bidirectional spring");
D = SMatrix{3, 3}(5., 0., 0., 0., 5.0, 0., 0., 0., 5.)
add_component!(vms1, LinearDamper(D, "bidirectional position error"); id="bidirectional damper")

# SIMULATION
tspan = (0., 15.)
vms_compiled = compile(vms1)
q = ([-1.0, 1.0], zero_q(vms_compiled.virtual_mechanism)) 
q̇ = (zero_q̇(vms_compiled.robot), zero_q̇(vms_compiled.virtual_mechanism)) 
g = VMRobotControl.DEFAULT_GRAVITY
dcache = new_dynamics_cache(vms_compiled)
prob = get_ode_problem(dcache, g, q, q̇, tspan)
sol = solve(prob, Tsit5(), progress=true; maxiters=1e6, abstol=1e-6, reltol=1e-6)

# ANIMATION
fig = Figure(size=(700, 750))
ls = LScene(fig[1, 1]; show_axis = false)  # 3D interactive scene
cam = cam3d!(ls, camera=:perspective, center=false)  
cam.lookat[] = [-0.06, 0.07, 0.06]
cam.eyeposition[] = [-0.75, 1.0, 0.6]

plotting_kcache = Observable(new_kinematics_cache(compile(robot)))
robotsketch!(ls, plotting_kcache; scale = 0.5)
display(fig)
animate_robot_odesolution(fig, sol, plotting_kcache, "test.mp4"; fps = 25);

### SECOND SIMULATION: UNIDIRECTIONAL SPRING-DAMPER ###

vms2 = VirtualMechanismSystem("myVMS2", robot)

add_coordinate!(vms2, ShadowCoord(".robot.f1_centre_of_mass"); id="f1_shadow_coord");
add_coordinate!(vms2, CoordDifference("f1_shadow_coord", ".robot.f2_centre_of_mass"); id="unidirectional position error");

add_component!(vms2, LinearSpring(K, "unidirectional position error"); id="unidirectional spring")
add_component!(vms2, LinearDamper(D, "unidirectional position error"); id="unidirectional damper")

# SIMULATION
vms_compiled = compile(vms2)
dcache = new_dynamics_cache(vms_compiled)
prob = get_ode_problem(dcache, g, q, q̇, tspan)
sol = solve(prob, Tsit5(), progress=true; maxiters=1e6, abstol=1e-6, reltol=1e-6);

# ANIMATION
plotting_kcache = Observable(new_kinematics_cache(compile(robot)))
robotsketch!(ls, plotting_kcache; scale = 0.5)
display(fig)
animate_robot_odesolution(fig, sol, plotting_kcache, "test.mp4"; fps = 25);