# ## Rail Robot
# This example demonstrates a simple rail robot with a Rail joint and a Revolute joint.
# The resulting robot is a pendulum that can slide along a curved path.

using
    DifferentialEquations, 
    GLMakie, 
    Random,
    StaticArrays,
    VMRobotControl
using VMRobotControl.Splines: CubicSpline

# We use a cubic spline to define the path of the rail robot, and build the mechanism from 
# a Rail joint, a Revolute joint, and a Rigid joint.

spline_knots = [
    -1.0  0.0  0.1;
    -0.5  0.0  0.0;
    -0.0  0.0  0.1;
    0.5  0.0  0.0;
    1.0  0.0  0.1
]
spline = CubicSpline(spline_knots)

T1 = Transform(SVector(0.0, 0.0, 1.0), zero(Rotor{Float64}))
J1 = Rail(spline, T1)

T2 = Transform(SVector(0.0, 0.0, 0.0), zero(Rotor{Float64}))
J2 = Revolute(SVector(0.0, 1.0, 0.0), T2)

T3 = Transform(SVector(0.0, 0.0, 1.0), zero(Rotor{Float64}))
J3 = Rigid(T3)

mech = Mechanism{Float64}("2Link")
cart_frame = add_frame!(mech, "Cart")
L2_frame = add_frame!(mech, "L2")
EE_frame = add_frame!(mech, "EE")

add_joint!(mech, J1; parent="root_frame",   child=cart_frame,               id="J1")
add_joint!(mech, J2; parent=cart_frame,     child=L2_frame,                 id="J2")
add_joint!(mech, J3, parent=L2_frame,       child=EE_frame;                 id="J3")

# Then, we add a coordinate to the mechanism to represent the tip of the pendulum, and a
# coordinate to represent the position of the cart. These are used to add point masses to
# the mechanism.
add_coordinate!(mech, FrameOrigin(EE_frame);                                id="tip_pos")
add_coordinate!(mech, FrameOrigin(cart_frame);                              id="cart_pos")
add_component!(mech, PointMass(1.0, "cart_pos");                            id="cart_mass")
add_component!(mech, PointMass(1.0, "tip_pos");                             id="pendulum_mass")

# We compile the mechanism, and setup an ODE problem to simulate the dynamics of the rail robot.
m = compile(mech)

rng = MersenneTwister(1234)
sol, q, q̇, T = let 
    q = [2.0 + 1e-3*randn(rng), 0.0]
    q̇ = [0.0, 0.0]
    T = 30
    prob = get_ode_problem(new_dynamics_cache(m), VMRobotControl.DEFAULT_GRAVITY, q, q̇, T)
    sol = solve(prob, Tsit5(), maxiters=1e4, abstol=1e-6, reltol=1e-6)
    sol, q, q̇, T
end

# Finally, we animate the solution of the ODE problem to visualize the rail robot.
# We will create a vector of all the tip positions of the pendulum, to animate a trail.
fig = Figure(; size=(720, 720), figure_padding=0)
display(fig)
ls = LScene(fig[1, 1]; show_axis=false)
cam = cam3d!(ls; center=false)
cam.lookat[] = [0.0, 0.0, 0.5]
cam.eyeposition[] = [0.0, 5.0, 1.0]
q_obs = Observable(q)
N_trail_points = 60
trail_points = Observable(Vector{SVector{3, Float64}}(undef, N_trail_points))
fill!(trail_points[], SVector{3, Float64}(NaN, NaN, NaN))
trail_colors = 1:N_trail_points
trail_width = 0.5*exp.(-LinRange(0, -3, N_trail_points))
cache = Observable(new_kinematics_cache(m))
robotsketch!(ls, cache; linewidth=3)
lines!(ls, trail_points; color=trail_colors, colormap=:viridis, linewidth=trail_width)

animate_f_setup(cache) = (get_compiled_coordID(m, "tip_pos"),)
function animate_f_control(cache, t, args, extra)
    tip_pos_ID = args[1]
    pos = configuration(cache, tip_pos_ID)
    trail_points[][1:N_trail_points-1] .= trail_points[][2:N_trail_points]
    trail_points[][N_trail_points] = pos
    cam.eyeposition[] = [2.0*sin(0.2*t), 5.0, 1.0]
    update_cam!(ls.scene, cam)
    notify(trail_points)
end


module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])
savepath = joinpath(module_path, "docs/src/assets/rail_robot.mp4")
animate_robot_odesolution(fig, sol, cache, savepath; f_setup=animate_f_setup, f_control=animate_f_control)

# ```@raw html
# <video controls width="100%" height="auto" autoplay loop>
# <source src="assets/rail_robot.mp4" type="video/mp4">
# </video>
# ```
