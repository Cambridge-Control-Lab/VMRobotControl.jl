# # Franka Impedance Control - PD in task space
# Impedance control is a control strategy that is used to control a robot by 
# designing a force/position relationship rather than employing either position control or 
# force control independantly.
# A standard, 6DOF, end-effector impedance controller uses a PD feedback in end-effector space
# to create a desired force/position relationship. 
# We interpret this as the end-effector being connected to the world by a spring-damper system, and
# present here an implementation of this controller for a Franka Emika robot.

using DifferentialEquations
using GeometryBasics: Vec3f, Point3f
using GLMakie
using LinearAlgebra
using MeshIO
using StaticArrays
using VMRobotControl

# ## Loading/Building the Robot and Controller

# First we have to load the model of the robot, which uses the URDF file format.
# Because the meshes for the franka robot are in the DAE file format, which is not natively
# supported by Julia's MeshIO/FileIO, we have to manually register the DAE file format to be able to load
using FileIO, UUIDs
try
    FileIO.add_format(format"DAE", (), ".dae", [:DigitalAssetExchangeFormatIO => UUID("43182933-f65b-495a-9e05-4d939cea427d")])
catch
end
cfg = URDFParserConfig(;suppress_warnings=true) # This is just to hide warnings about unsupported URDF features
module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])
robot = parseURDF(joinpath(module_path, "URDFs/franka_description/urdfs/fr3.urdf"), cfg)

# Now, we begin adding gravity compensation to the robot model, as the franka robot does its own
# gravity compensation. We also add some damping to each joint to make the simulation more realistic
# so that the robot does not oscillate indefinitely.
add_gravity_compensation!(robot, VMRobotControl.DEFAULT_GRAVITY)
for i in 1:7
    add_coordinate!(robot, JointSubspace("fr3_joint$i");    id="J$i")
    add_component!(robot, LinearDamper(0.1, "J$i");         id="Joint damper $i")
end;

# Now, we build the virtual mechanism controller, which will control the position and orientation
# of the end effector of the robot. We define several coordinates, starting with the TCP position
# and orientation, and then the target position, and the position error. 
#
# A `FramePoint` coordinate is used to represent the position of the TCP, which is a point 
# in the frame of the final link of the robot. The `QuaternionAttitude` component is used to
# represent the orientation of the TCP, which is a quaternion. The `CoordDifference` component
# is used to take the difference between the target position and the current position of the
# TCP.

target_rot = AxisAngle(SVector(0., 1., 0.), Float64(π))
vms = VirtualMechanismSystem("franka_impedance_control", robot)
add_coordinate!(robot, FramePoint("fr3_link8", SVector(0., 0., 0.));            id="TCP position")
add_coordinate!(vms, QuaternionAttitude(".robot.fr3_link8", target_rot);        id="TCP orientation")
root = root_frame(vms.robot)
add_coordinate!(vms, FramePoint(".robot.$root", SVector(0.4, 0.0, 0.2));        id="Target position")
add_coordinate!(vms, CoordDifference(".robot.TCP position", "Target position"); id="Position error");

# We then add a linear spring-damper components onto the defined coordinates. The linear spring-damper
# components will act as the impedance controller, which will try to keep the end effector at a fixed
# position and orientation.
#
# We use different stiffness and damping values in different directions, to demonstrate the 
# flexibility of the impedance controller: e.g. a linear stiffness of 100 N/m in the x-direction, 
# 200 N/m in the y-direction, and 300 N/m in the z-direction, and similarly for the damping values.
K = SMatrix{3, 3}(100., 0., 0., 0., 200., 0., 0., 0., 300.)
add_component!(vms, LinearSpring(K, "Position error");           id="Linear Spring")
D = SMatrix{3, 3}(10., 0., 0., 0., 20.0, 0., 0., 0., 30.)
add_component!(vms, LinearDamper(D, "Position error");           id="Linear Damper")
K_rot = SMatrix{3, 3}(10., 0., 0., 0., 20., 0., 0., 0., 30.)
add_component!(vms, LinearSpring(K_rot, "TCP orientation");       id="Angular Spring")
D_rot = SMatrix{3, 3}(0.1, 0., 0., 0., 0.2, 0., 0., 0., 0.3)
add_component!(vms, LinearDamper(D_rot, "TCP orientation");       id="Angular Damper");

# ## Setting up the simulation
# We define a disturbance function that will apply a force of 10 N in the y-direction for the first
# 3 seconds of the simulation. We then define the `f_control` to apply this disturbance force to the
# system, as `f_control` is called once per timestep of the simulation, and can be used to apply
# external forces to the system. Function `f_setup` is used to get the coordinate ID of the TCP position
# at the beginning of the simulation, to be used in the `f_control` function.
#
# We then define the timespan, initial joint angles, joint velocities, and gravity vector for the 
# simulation. We create a dynamics cache, and an ODE problem, and solve the ODE problem using the
# Tsit5 solver from DifferentialEquations.jl.

disturbance_func(t) = mod(t, 6) < 3 ? SVector(0., 0., 0.) : SVector(0., 10.0, 0.)
f_setup(cache) = get_compiled_coordID(cache, ".robot.TCP position")
function f_control(cache, t, args, extra)
    tcp_pos_coord_id = args
    F = disturbance_func(t)
    uᵣ, uᵥ = get_u(cache)
    z = configuration(cache, tcp_pos_coord_id)
    J = jacobian(cache, tcp_pos_coord_id)
    mul!(uᵣ, J', F)
    nothing
end
tspan = (0., 12.)
q = ([0.0, 0.3, 0.0, -1.8, 0.0, π/2, 0.0], Float64[]) # Robot joint angle, vm joint angles
q̇ = ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], Float64[]) # Robot joint velocity, vm joint velocities
g = VMRobotControl.DEFAULT_GRAVITY
dcache = new_dynamics_cache(compile(vms))
prob = get_ode_problem(dcache, g, q, q̇, tspan; f_setup, f_control)
@info "Simulating franka robot with impedance control."
sol = solve(prob, Tsit5(); maxiters=1e5, abstol=1e-6, reltol=1e-6);

# ## Visualizing the simulation
# We create a figure, and a 3D scene, and set the camera position to a good view of the robot. 
# We create an observable for the time, and the kinematics cache, which will used by the function
# `animate_robot_odesolution` to update any plots that depend upon these observables.
# We then visualize the robot, and the force arrow that is applied to the end effector of the robot. 
# We then animate the simulation, and save the animation to a file.
fig = Figure(size = (720, 720), figure_padding=0)
display(fig)
ls = LScene(fig[1, 1]; show_axis=false)
cam = cam3d!(ls, camera=:perspective, center=false)
cam.lookat[] = [0.24015087703685004, -0.02303109659518269, 0.37391966173978597]
cam.eyeposition[] = [0.5360994756635347, 0.7578567808643422, 0.5303480879908374]

plotting_t = Observable(0.0)
plotting_kcache = Observable(new_kinematics_cache(compile(robot)))
robotvisualize!(ls, plotting_kcache;)

tcp_pos_id = get_compiled_coordID(plotting_kcache[], "TCP position")
tcp_pos = map(plotting_kcache) do kcache
    Point3f(configuration(kcache, tcp_pos_id))
end
force = map(t -> 0.01 * Vec3f(disturbance_func(t)), plotting_t)
arrowscale = map(f -> 0.1*(f'*f)^(0.25), force)
arrows3d!(ls, map(p -> [p], tcp_pos), map(f -> [f], force); color = :red, markerscale = arrowscale)


savepath = joinpath(module_path, "docs/src/assets/franka_impedance_control.mp4")
animate_robot_odesolution(fig, sol, plotting_kcache, savepath; t=plotting_t);

# ```@raw html
# <video controls width="100%" height="auto" autoplay loop>
# <source src="../../assets/franka_impedance_control.mp4" type="video/mp4">
# </video>
# ```