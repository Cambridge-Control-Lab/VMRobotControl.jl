# # Reaching with Obstacle Avoidance
# In this example, we will generate a controller for the sciurus17 robot to reach two targets while
#  avoiding two obstacles. The reaching controller will be a saturating-spring, linear-damper 
# virtual mechanism, with additional repulsive springs to avoid the obstacles. 
#
# The sciurus17 robot is a many-DOF robot 2 arms, a pivotting waist and 2DOF neck/head.

using DifferentialEquations
using GLMakie
using StaticArrays
using VMRobotControl

# First, we load the URDF file for the sciurus17 robot, with warnings suppressed, as the URDF file
# contains some features that are not supported by the URDFParser, but are not necessary for this example.
# We then add joint limit springs to the robot, to prevent the robot from reaching its joint limits.
# These act like `buffers' around the joint limits, which will and push back with a spring force 
# as the joint approaches its limits. We also add a linear damper to each joint to make the simulation
# more realistic, so that the robot does not oscillate indefinitely.
# Finally, we add the TCPs for the left and right arms of the robot, which will be used for the
# reaching controller.
cfg = URDFParserConfig(; suppress_warnings=true)
module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])
robot = parseURDF(joinpath(module_path, "URDFs/sciurus17_description/urdf/sciurus17.urdf"), cfg)
# Add joint limit springs
joint_limits = cfg.joint_limits
for joint_id in keys(joints(robot))
    limits = joint_limits[joint_id]
    isnothing(limits) && continue
    add_coordinate!(robot, JointSubspace(joint_id);  id="$(joint_id)_coord")
    @assert ~isnothing(limits.lower) && ~isnothing(limits.upper)
    add_deadzone_springs!(robot, 50.0, (limits.lower+0.1, limits.upper-0.1), "$(joint_id)_coord")
    add_component!(robot, LinearDamper(0.01, "$(joint_id)_coord"); id="$(joint_id)_damper")
end
add_coordinate!(robot, FramePoint("l_link7", SVector(0.,  0.08, 0.0)); id="L TCP")
add_coordinate!(robot, FramePoint("r_link7", SVector(0., -0.08, 0.0)); id="R TCP")

# Next we build the virtual mechanism controller.
# There is no virtual mechanism structure, as the controller we are building does not require any
# simulated links or joints, only springs/dampers and repulsive springs.
# We define two targets for the robot to reach, and the position error for each target.
# On top of these coordinates, we add a saturating spring-damper component, which will act as the
# reaching controller. 

vms = VirtualMechanismSystem("sciurus_reaching", robot)
add_gravity_compensation!(vms, VMRobotControl.DEFAULT_GRAVITY)
vm = vms.virtual_mechanism
add_coordinate!(vm, ReferenceCoord(Ref(SVector(0.5,  0.0, 0.4))); id="target_1_pos")
add_coordinate!(vm, ReferenceCoord(Ref(SVector(0.5, -0.1, 0.2))); id="target_2_pos")
add_coordinate!(vms, CoordDifference(".robot.L TCP", ".virtual_mechanism.target_1_pos"); id="L pos error")
add_coordinate!(vms, CoordDifference(".robot.R TCP", ".virtual_mechanism.target_2_pos"); id="R pos error")

add_component!(vms, TanhSpring("L pos error"; max_force=10.0, stiffness=1000.0); id="L spring")
add_component!(vms, LinearDamper(10.0, "L pos error"); id="L damper")
add_component!(vms, TanhSpring("R pos error"; max_force=10.0, stiffness=1000.0); id="R spring")
add_component!(vms, LinearDamper(10.0, "R pos error"); id="R damper")


# We define two obstacle positions, and for each frame in each arm attach a repulsive spring between
# the frame origin and the obstacle position. The repulsive spring will push the frame origin away
# from the obstacle, with a force that decreases as the frame origin moves away from the obstacle, 
# according to the Gaussian potential energy of the spring.
obstacles = Dict(
    "obstacle_1" => SVector(0.4,  0.1, 0.3),
    "obstacle_2" => SVector(0.3, -0.2, 0.1)
)
collision_frames = String[]
for i in 1:7
    push!(collision_frames, "l_link$i")
    push!(collision_frames, "r_link$i")
end
for (id, pos) in obstacles
    add_coordinate!(vm, FramePoint("root_frame", pos); id)
end
for id in collision_frames
    add_coordinate!(robot, FrameOrigin("$id"); id="$id frame origin")
    for obstacle in keys(obstacles)
        add_coordinate!(vms, CoordDifference(".robot.$id frame origin", ".virtual_mechanism.$obstacle"); id="$obstacle $id error")
        add_component!(vms, GaussianSpring("$obstacle $id error"; max_force=-10.0, width=0.05); id="$obstacle $id spring")
    end
end

# ## Setup simulation
# We define functions to move the targets in a circular motion, and to set the target positions and
# velocities in the `f_control` function. We then define the `f_setup` function to get the coordinate
# IDs of the target positions and velocities, to be used in the `f_control` function.
# 
# We then define the timespan, initial joint angles, joint velocities, and gravity vector for the
# simulation. We create a dynamics cache, and an ODE problem, and solve the ODE problem using the
# Tsit5 solver from DifferentialEquations.jl.
target_1_pos(t) = SVector(0.5,  0.0, 0.3) + SVector(0.2*cos(t), 0.0, 0.2*sin(t))
target_1_vel(t) = SVector(-0.2*sin(t), 0.0, 0.2*cos(t))
target_2_pos(t) = SVector(0.5, -0.1, 0.2) + SVector(0.0, 0.2*sin(t), 0.2*cos(t))
target_2_vel(t) = SVector(0.2*cos(t), 0.0, -0.2*sin(t))
function f_setup(cache)
    id1 = get_compiled_coordID(cache, ".virtual_mechanism.target_1_pos")
    id2 = get_compiled_coordID(cache, ".virtual_mechanism.target_2_pos")
    val1 = cache[id1].coord_data.val
    val2 = cache[id2].coord_data.val
    vel1 = cache[id1].coord_data.vel
    vel2 = cache[id2].coord_data.vel
    (val1, val2, vel1, vel2) 
end
function f_control(cache, t, args, extra)
    (val1, val2, vel1, vel2) = args
    val1[] = target_1_pos(t)
    vel1[] = target_1_vel(t)
    val2[] = target_2_pos(t)
    vel2[] = target_2_vel(t)
    false
end
tspan = (0., 6π)
dcache = new_dynamics_cache(compile(vms))
q = zero_q(dcache.vms)
q̇ = zero_q̇(dcache.vms)
g = VMRobotControl.DEFAULT_GRAVITY
prob = get_ode_problem(dcache, g, q, q̇, tspan; f_setup, f_control)
@info "Simulating Sciurus reaching with obstacle avoidance problem."
sol = solve(prob, Tsit5(); maxiters=1e5, abstol=1e-3, reltol=1e-3) # Low tol to speed up simulation

# ## Plotting
# We create a figure with two scenes, for two different camera angles. We plot the robot, the
# targets, the TCPs, and the obstacles in the scene.
# We use observables for the time and the kinematics cache, which will be updated in the function
# `animate_robot_odesolution`, causing any plots that depend upon these observables to be updated.
fig = Figure(; size = (2*720, 720), figure_padding=1.5, fontsize=32)
display(fig)
lscenes = (LScene(fig[1, 1]; show_axis=false), LScene(fig[1, 2]; show_axis=false))
cams = map(ls -> cam3d!(ls; center=false), lscenes)
plotting_t = Observable(0.0)
plotting_kcache = Observable(new_kinematics_cache(compile(vms)))

target_scatter_kwargs = (;
    color=:green, 
    marker=:+, 
    markersize=15, 
    label="Targets",
    transparency=true ## Avoid ugly white outline artefact on markers
)
tcp_scatter_kwargs = (;
    color=:blue, 
    marker=:x, 
    markersize=15, 
    label="TCPs",
    transparency=true ## Avoid ugly white outline artefact on markers
)

for ls in lscenes        
    ## Show robot
    robotvisualize!(ls, plotting_kcache; overdraw=false)
    
    ## Label target and TCP
    target_1_pos_id = get_compiled_coordID(plotting_kcache[], ".virtual_mechanism.target_1_pos")
    target_2_pos_id = get_compiled_coordID(plotting_kcache[], ".virtual_mechanism.target_2_pos")
    l_tcp_pos_id = get_compiled_coordID(plotting_kcache[], ".robot.L TCP")
    r_tcp_pos_id = get_compiled_coordID(plotting_kcache[], ".robot.R TCP")
    scatter!(ls, plotting_kcache, [target_1_pos_id, target_2_pos_id]; target_scatter_kwargs...)
    scatter!(ls, plotting_kcache, [l_tcp_pos_id, r_tcp_pos_id]; tcp_scatter_kwargs...)
    for (id, pos) in obstacles ## Show obstacles
        small_sphere = Sphere(Point3f(pos...), 0.05)
        large_sphere = Sphere(Point3f(pos...), 0.1)
        mesh!(ls, small_sphere; color=:magenta, transparency=true)
        poly!(ls, Point3f[]; color=:magenta, label="Obstacles") ## Just for legend entry
        mesh!(ls, large_sphere; color=:cyan, alpha=0.1, transparency=true)
        poly!(ls, Point3f[]; color=:cyan, label="Repulsive field") ## Just for legend entry
    end
end
cams[1].lookat[] = [0., 0., 0.3]
cams[1].eyeposition[] = [1.5, 0., 0.3]
cams[2].lookat[] = [0.4, 0, 0.3]
cams[2].eyeposition[] = [0.4, 1.5, 0.3]

leg = Legend(fig[1, 1], lscenes[1]; merge=true, tellwidth=false, halign=:left, valign=:top)
savepath = joinpath(module_path, "docs/src/assets/sciurus_reaching.mp4")
animate_robot_odesolution(fig, sol, plotting_kcache, savepath; t=plotting_t, f_setup, f_control, fastforward=1.0, fps=20)

# ```@raw html
# <video controls width="100%" height="auto" autoplay loop>
# <source src="assets/sciurus_reaching.mp4" type="video/mp4">
# </video>
# ```