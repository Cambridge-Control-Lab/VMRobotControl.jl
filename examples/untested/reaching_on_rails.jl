using VMRobotControl
using VMRobotControl.Splines: LoopingCubicSpline
using DifferentialEquations, GLMakie, StaticArrays

# frames
# Must be able to index from frame into vector - unique int for each frame in a mechanism
# Frames also have an associated string


using FileIO
using UUIDs
    try FileIO.add_format(format"DAE", (), ".dae", [:DigitalAssetExchangeFormatIO => UUID("43182933-f65b-495a-9e05-4d939cea427d")])
catch e
end


############################
# Build joint limit mechanism
############################

const FRANKA_RESEARCH_3_JOINT_LIMITS = [
    (-2.7437,  2.7437),
    (-1.7837,  1.7837),
    (-2.9007,  2.9007),
    (-3.0421, -0.1518),
    (-2.8065,  2.8065),
    ( 0.5445,  4.5169),
    (-3.0159,  3.0159)
]

# k is stiffness
# c is maximum damping coefficient
# w_k is the distance from the joint limit at which the spring starts to active.
# w_d is the distance from the spring limit to the point at which the damper starts to act. 
FRANKA_RESEARCH_3_JOINT_BUFFER_PARAMS =[
    (k=50.0, c=4.0, w_k=0.30,  w_d=0.40),
    (k=50.0, c=4.0, w_k=0.20,  w_d=0.20),
    (k=50.0, c=4.0, w_k=0.20,  w_d=0.20),
    (k=60.0, c=5.0, w_k=0.30,  w_d=0.30),
    (k=35.0, c=2.0, w_k=0.35,  w_d=0.35),
    (k=30.0, c=1.5, w_k=0.35,  w_d=0.35),
    (k=30.0, c=1.0, w_k=0.35,  w_d=0.35)
]

function add_buffer!(mechanism, coord, bounds; k, c, w_k, w_d)
    @assert bounds[1] < bounds[2]
    @assert w_k > 0.
    @assert w_d > 0.

    spring_bounds =      (bounds[1] + w_k,       bounds[2] - w_k)
    top_damper_lerp =    (bounds[2] - w_k - w_d, bounds[2] - w_k)
    bottom_damper_lerp = (bounds[1] + w_k,       bounds[1] + w_k + w_d)

    add_deadzone_springs!(mechanism, k, spring_bounds, coord, Float64, 1)
    damper1 = RectifiedDamper(c, coord, top_damper_lerp, false; diodic=true);
    damper2 = RectifiedDamper(c, coord, bottom_damper_lerp, true; diodic=true);
    add_component!(mechanism, damper1; id="$(coord)_damper_top")
    add_component!(mechanism, damper2; id="$(coord)_damper_bottom")
end

function add_fr3_joint_limit_buffers!(mechanism; limits=FRANKA_RESEARCH_3_JOINT_LIMITS, params=FRANKA_RESEARCH_3_JOINT_BUFFER_PARAMS)
    for i = 1:7
        l_i = limits[i]
        p_i = params[i]
        buffer = add_buffer!(mechanism, "J$(i)", l_i; p_i...)
    end
    mechanism
end

#################################################

isdefined(Main, :_robot) || (_robot = parseURDF("./URDFs/franka_description/urdfs/fr3_franka_hand.urdf", URDFParserConfig(suppress_warnings=true)))


robot = let 
    robot = deepcopy(_robot)
    add_coordinate!(robot, FramePoint("fr3_hand_tcp", SVector(0.0, 0.0, 0.05)); coordID="TCP")
    add_coordinate!(robot, JointSubspace("fr3_finger_joint1"); coordID="left_finger")
    add_coordinate!(robot, JointSubspace("fr3_finger_joint2"); coordID="right_finger")
    # Add inertance to fingers 
    add_component!(robot, LinearInerter(0.1, "left_finger"); id="left_finger_inertance")
    add_component!(robot, LinearInerter(0.1, "right_finger"); id="right_finger_inertance")
    for joint in 1:7
        add_coordinate!(robot, JointSubspace("fr3_joint$(joint)"); coordID="J$(joint)")
        add_component!(robot, LinearDamper(0.1, "J$(joint)"); id="joint$(joint)_damper")
    end
    add_gravity_compensation!(robot, VMRobotControl.DEFAULT_GRAVITY)
    add_fr3_joint_limit_buffers!(robot)
    robot
end

virtual_mechanism = let
    spline_knots = [
        -0.5  0.0  0.1;
        -0.5  0.0  0.3;
        0.0  0.0  0.3;
        0.5  0.0  0.1;
        0.5  0.0  0.3;
        0.0  0.0  0.3;
    ]
    spline = LoopingCubicSpline(spline_knots)
    T1 = Transform(SVector(0.6, 0.0, 0.0), AxisAngle(SVector(0.0, 0.0, 1.0), π/2))
    J1 = Rail(spline, T1)
    mech = Mechanism{Float64}("2Link")
    cart_frame = add_frame!(mech, "Cart")
    add_joint!(mech, J1; parent="root_frame",   child=cart_frame,   jointID="J1")
    add_coordinate!(mech, FrameOrigin(cart_frame); coordID="tip_pos")
    add_coordinate!(mech, JointSubspace("J1"); coordID="cart_distance")
    add_component!(mech, PointMass(10.0, "tip_pos"); id="cart_mass")
    add_gravity_compensation!(mech, VMRobotControl.DEFAULT_GRAVITY)
    add_component!(mech, LinearDamper(5.0, "tip_pos"); id="cart_damper")
    add_component!(mech, ForceSource(SVector(5.0), 50.0, "cart_distance"); id="cart_force")
    mech
end

vms = VirtualMechanismSystem("reaching_on_rails", robot, virtual_mechanism)
add_coordinate!(vms, CoordDifference(".virtual_mechanism.tip_pos", ".robot.TCP"); coordID="tip_err")
add_component!(vms, LinearSpring(3000.0, "tip_err"); id="spring")
add_component!(vms, LinearDamper(40.0, "tip_err"); id="damper")


m = compile(vms)


sol, q, q̇, T = let 
    # q = [2.0 + 1e-3*randn(), 0.0]
    # q̇ = [0.0, 0.0]
    q = zero_q(m)
    q̇ = zero_q̇(m)
    # x0 = vcat(vcat(q...), vcat(q̇...))
    T = 15
    prob = get_ode_problem(new_dynamics_cache(m), VMRobotControl.DEFAULT_GRAVITY, q, q̇, T)
    sol = solve(prob, Tsit5(), abstol=1e-10, reltol=1e-10)
    sol, q, q̇, T
end

lines(sol)

###############
# Animate
###############
fig = Figure()
display(fig)
ax = LScene(fig[1, 1]; show_axis=false)
q_obs = Observable(q)
cache = Observable(new_kinematics_cache(m))
robotsketch!(ax, cache; scale=0.1)
cart_pos = get_compiled_coordID(m, ".virtual_mechanism.tip_pos")
scatter!(ax, cache, cart_pos; markersize=10, color=:red, marker=:rect)
robotvisualize!(ax, cache)

animate_robot_odesolution(fig, sol, cache, "tmp.mp4")
