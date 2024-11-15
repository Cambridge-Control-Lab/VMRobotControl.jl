using VMRobotControl
using StaticArrays

robot = Mechanism{Float64}("robot2L")
# "root_frame" is automatically created
add_frame!(robot, "L1_frame")
add_frame!(robot, "L2_frame")
add_frame!(robot, "EE_frame")

# Define the unit vectors to make things easier
X = SVector(1., 0., 0.)
Y = SVector(0., 1., 0.)
Z = SVector(0., 0., 1.)
# Here, we define the joint-data for each joint.
J1 = Revolute(Y, Transform(0.15*Z))
J2 = Revolute(Y, Transform(0.4*Z))
J3 = Rigid(Transform(0.4*Z))

add_joint!(robot, J1; parent="root_frame", child="L1_frame", id="J1")
add_joint!(robot, J2; parent="L1_frame",   child="L2_frame", id="J2")
add_joint!(robot, J3; parent="L2_frame",   child="EE_frame", id="JEE")

add_coordinate!(robot, FramePoint("L1_frame", 0.2*Z); id="L1_com")
add_coordinate!(robot, FramePoint("L2_frame", 0.2*Z); id="L2_com")
add_coordinate!(robot, FramePoint("EE_frame", SVector(0, 0, 0)); id="TCP")
add_coordinate!(robot, ConstCoord(SVector(0.38, 0.0, 0.64)); id="ref")
add_coordinate!(robot, CoordDifference("TCP", "ref"); id="tip_error")

add_component!(robot, PointMass(1.0, "L1_com"); id="L1_mass")
add_component!(robot, PointMass(1.0, "L2_com"); id="L2_mass")
add_component!(robot, LinearSpring(1.0, "tip_error"); id="Tip spring")
add_component!(robot, LinearDamper(1.0, "tip_error"); id="Tip damper")

m = compile(robot)

using GLMakie

# Setup cache and indices for plotting
cache = Observable(new_kinematics_cache(m))

q = Observable([0.0, 0.0])
on(q) do q
   kinematics!(cache[], 0.0, q)
   notify(cache)
end

coord_ee = get_compiled_coordID(m, "TCP")
coord_ref = get_compiled_coordID(m, "ref")
spring_extension = get_compiled_coordID(m, "tip_error")
L1_com = get_compiled_coordID(m, "L1_com")
L2_com = get_compiled_coordID(m, "L2_com")
root_frameid = get_compiled_frameID(m, "root_frame")
L1_frame = get_compiled_frameID(m, "L1_frame")
L2_frame = get_compiled_frameID(m, "L2_frame")
spring = get_compiled_componentID(m, "Tip spring")

# Define some plotting attributes

robot_sketch_attributes = (
    linewidth=2.0,
    linecolor=:black,
    scale=1.0
)

robot_line_attributes = (
    color=robot_sketch_attributes.linecolor,
    linewidth=robot_sketch_attributes.linewidth
)

coord_line_attributes = (
    color=:red, 
    linestyle=:dash
)

centre_of_mass_marker_attributes = (
    color=:blue,
    markersize=10,
    marker='⨁',
    depth_shift=-0.2f0
)

coord_marker_attributes = (
    color=:red,
    markersize=10,
    depth_shift=-0.2f0
)

text_attributes = (
    fontsize=12,
    glowwidth=2,
    glowcolor=:white,
    depth_shift=-0.2f0,
)


# Setup figure
fig = Figure(size=(390, 230))
display(fig)

# Scene 1, 2D view
ls1 = LScene(fig[1, 1])
delete!(ls1, ls1.scene.plots[1])
cam1 = cam3d!(ls1, projectiontype=:orthographic, center=false)


# Scene 2, 3D view
ls2 = LScene(fig[1, 2])
delete!(ls2, ls2.scene.plots[1])
cam = cam3d!(ls2, projectiontype=:orthographic, center=false)

# Plot robot sketch and coords
for ls in (ls1, ls2)
    robotsketch!(ls, cache; robot_sketch_attributes...)
    scatter!(ls, cache, [coord_ee, coord_ref]; coord_marker_attributes...)
    componentsketch!(ls, cache, spring; robot_sketch_attributes..., scale=0.01, linecolor=:magenta)
    # scatterlines!(ls, cache, [coord_ee, coord_ref]; coord_line_attributes...)
    scatter!(ls, cache, [L1_com, L2_com]; centre_of_mass_marker_attributes...)
end
text!(ls1, cache, [L1_com, L2_com]; text=["L1_com", "L2_com"], text_attributes..., align=(:right, :bottom), color=:blue)
text!(ls1, cache, [coord_ee, coord_ref]; text=["TCP", "Ref"], text_attributes..., color=:red,)
text!(ls1, cache, spring_extension; text="Spring", text_attributes..., color=:magenta) # Must be plotted on its own for dispatch to work :/

# Cam positions
lookat1 = Vec3f(0.2, 0.0, 0.5)
eyeposition1 = Vec3f(0.2, -0.5, 0.5)
update_cam!(ls1.scene, eyeposition1, lookat1, Vec3f(0, 0, 1))

lookat2 = Vec3f(0.1, 0.0, 0.4)
eyeposition2 = Vec3f(0.3, -0.4, 0.4)
update_cam!(ls2.scene, eyeposition2, lookat2, Vec3f(0, 0, 1))
fig

q[] = [0.0, 1*π/4]

text!(fig.scene, "Side (2D) view", position=(0.0, 1.0), fontsize=14, color=:black, space=:relative, align=(:left, :top))
text!(fig.scene, "3D view", position=(0.5, 1.0), fontsize=14, color=:black, space=:relative, align=(:left, :top))

# CairoMakie.activate!();
GLMakie.activate!();
save("./build_example.png", fig; px_per_unit=4); 

# using DifferentialEquations
# dynamics = get_ode_dynamics(new_dynamics_cache(m), VMRobotControl.DEFAULT_GRAVITY)
# tspan = (0.0, 10.0)
# u0 = SVector(0.0, 0.0, 0.0, 0.0)
# prob = ODEProblem(dynamics, u0, tspan)
# sol = solve(prob, Tsit5())
# animate_robot_odesolution(fig, sol, cache, "./figs/tmp.mp4")