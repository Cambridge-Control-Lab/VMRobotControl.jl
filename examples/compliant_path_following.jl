# # Compliant Path Following
# This example demonstrates how to simulate a compliant path following control strategy for a robot
# moving along a predefined path. The robot is controlled to follow a predefined path while being
# subjected to a disturbance force. The robot is controlled by a virtual mechanism system, which
# includes a virtual track that a virtual cart moves along. A virtual spring and damper are used to
# connect the virtual cart to the robot's end effector, and a force source is used to apply a
# driving force to the virtual cart.
#
# The result is a controller which stiffly constrains the robot to the path, while behaving 
# compliantly along the direction of travel. If a force exceeding the driving force is applied to
# the robot, the cart may even reverse its direction of travel, but the linear spring will 
# ensure that the robot remains on the path.

using DifferentialEquations
using GLMakie
using LinearAlgebra

using VMRobotControl
using VMRobotControl.Splines: CubicSpline
using StaticArrays

# ## Loading/Building the Robot and Controller

# First, we define the path that the robot will follow. The path is defined as a spline, which is
# a piecewise cubic polynomial that interpolates a set of points. We construct the spline using 
# a set of points that define the path, and then create a `CubicSpline` object from these points, 
# which will pass exactly through the points.
#
# Care must be taken to ensure that the path is smooth and continuous, as the units of the system 
# state are in meters and radians: if there are large differences in scale between the points, on 
# the path, then the automatic tolerance selection in the ODE solver may struggle to perform well, 
# as in some regions of the path a small difference in `q` may cause a large difference in the
# position, requiring a small step size to accurately simulate the system. 
start = SVector(0.425, 0.15, 0.10)
s_width = 4e-2
s_length = 20e-2
spline = let w=s_width, x0=start[1], y0=start[2], L=s_length, h=start[3], Nk = 6, Nr = 6
    forward = Vector(LinRange(x0, x0+L, Nk))
    backward = Vector(reverse(forward))
    spline_points = Matrix{Float64}(undef, 0, 3);

    for i = 0:(Nr-1)
        knots = (i%2)!=1 ? forward : backward 
        spline_points = vcat(
            spline_points, 
            hcat(knots, y0 .+ w*i*ones(size(knots)), h*ones(size(knots)))
        )
    end
    CubicSpline(spline_points)
end

# Because the meshes for the franka robot are in the DAE file format, which is not natively
# supported by MeshIO/FileIO, we have to manually register the DAE file format to be able to load
# Register DAE file format to DigitalAssetExchangeFormatIO, so that the mesh files for the franka
# robot can be loaded. Most of the time, this is done automatically by the package that provides the
# file format, but in this case we have to do it manually, before we can load the URDF file.
# Then we load the URDF file, with warnings suppressed, as the URDF file contains some features
# that are not supported by the URDFParser, but are not necessary for this example.
#
# We then add gravity compensation to the robot model. As the franka robot does its own gravity
# compensation, this is considered part of the robot model, rather than part of the controller, so 
# it is added directly to `robot`, not to the virtual mechanism system.
#
# We then define a tool center point (TCP), in the end-effector frame of the robot, and add a
# coordinate to the robot model to represent the TCP. We also add a coordinate to the robot model
# for each joint, and add a tanh damper component to each joint to make the simulation more
# realistic, emulating the coulomb friction in the joints. The choice of β will affect the 
# minimum velocity at which the damper will start to act, and will therefore affect the
# number of timesteps required to simulate the system: a smaller β will require more timesteps.

using FileIO, UUIDs
try
    FileIO.add_format(format"DAE", (), ".dae", [:DigitalAssetExchangeFormatIO => UUID("43182933-f65b-495a-9e05-4d939cea427d")])
catch
end
module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])
robot = parseRSON(joinpath(module_path, "RSONs/rsons/franka_panda/pandaSurgical.rson"))
add_gravity_compensation!(robot, VMRobotControl.DEFAULT_GRAVITY)
add_coordinate!(robot, FrameOrigin("instrument_EE_frame"); id="TCP")

for (i, τ_coulomb) in zip(1:7, [5.0, 5.0, 5.0, 5.0, 3.0, 3.0, 3.0])
    β = 1e-1
    add_component!(robot, TanhDamper(τ_coulomb, β, "J$i"); id="J$(i)_damper")
end;

# Now, we build the virtual mechanism, which consists of a virtual track that a virtual cart moves
# along. The virtual track is a `Rail` joint based upon the spline that we defined earlier. The
# virtual cart is connected to the robot's end effector by a virtual spring and damper, using the 
# coordinate `CartPosition`.
# Coordinate `CartDistance` is the jointspace coordinate of the rail joint: distance the cart has
# moved along the virtual track.
# Components `CartInertance` and `CartDamper` are used to add inertance and damping to the cart, 
# and can be thought of as the mass and friction of the cart, respectively.
# Notably, using a `LinearInerter` component here is equivalent to adding a mass to the cart which
# is not affected by the force of gravity. 
vm = Mechanism{Float64}("VirtualTrack")

cart_frame = add_frame!(vm, "Cart")
add_joint!(vm, Rail(spline, zero(Transform{Float64})); 
        parent=root_frame(vm), child=cart_frame,        id="RailJoint")
# Jointspace components
add_coordinate!(vm, JointSubspace("RailJoint");         id="CartDistance")
add_coordinate!(vm, FrameOrigin(cart_frame);            id="CartPosition")
add_component!(vm, LinearInerter(1.0, "CartPosition");  id="CartInertance") # Cart mass
add_component!(vm, LinearDamper(100.0, "CartPosition"); id="CartDamper");

# Now, we will combine the robot and the virtual mechanism into a single system. To do this 
# we will create a `VirtualMechanismSystem` object, which will contain both the robot and the
# virtual mechanism. We will then add a `CoordDifference` component to the system, which will
# take the difference between the position of the virtual cart and the position of the robot's
# end effector. We will then add a spring and damper component to the system, which will constrain
# the robot to the path defined by the cart.
#
# Finally we will add a `ForceSource` component to the system, which will apply a driving force
# to the cart, to move it along the path. The force source will apply a force of 20N in the forward 
# direction, unless doing so would exceed the maximum power of 10W, in which case the magnitude of
# the force will be reduced to ensure that the power does not exceed 10W.
vms = VirtualMechanismSystem("System", robot, vm)
err_coord = CoordDifference(".virtual_mechanism.CartPosition", ".robot.TCP")
err_spring = LinearSpring(3000.0 * identity(3), "CartError")
err_damper = LinearDamper(50.0 * identity(3), "CartError")

add_coordinate!(vms, err_coord;     id="CartError")
add_component!(vms, err_spring;     id="CartErrSpring")
add_component!(vms, err_damper;     id="CartErrDamper")

max_power = 10.0
force_source = ForceSource(SVector(20.0), max_power, ".virtual_mechanism.CartDistance")
add_component!(vms, force_source;   id="Force source");

# ## Setting up the simulation
# We define a disturbance function that will apply a force of 20N in the negative x-direction 
# between 3 and 6 seconds. We then define the `f_control` function, which will apply this
# disturbance force to the system. The `f_control` function is called once per timestep of the
# simulation, and can be used to apply external forces to the system. The `f_setup` function is
# used to get the coordinate ID of the TCP position at the beginning of the simulation, to be used
# in the `f_control` function.
#
# We then define the timespan, initial joint angles, joint velocities, and gravity vector for the
# simulation. We create a dynamics cache, and an ODE problem, and solve the ODE problem using the
# Tsit5 solver from DifferentialEquations.jl.

f_setup(cache) = get_compiled_coordID(cache, ".robot.TCP")
disturbance_func(t) = 3 < t < 6 ? SVector(-20., 0., 0.) : SVector(0., 0., 0.)
function f_control(cache, t, args, extra)
    tcp_pos_coord_id = args
    F = disturbance_func(t)
    uᵣ, uᵥ = get_u(cache)
    z = configuration(cache, tcp_pos_coord_id)
    J = jacobian(cache, tcp_pos_coord_id)
    mul!(uᵣ, J', F)
    nothing
end

cache = new_dynamics_cache(compile(vms))
tspan = (0.0, 10.0)
q = ([0.0, 0.1, 0.0, -2.5, pi/2, pi/2, 0.6], [0.0])
q̇ = (zeros(7), zeros(1))
gravity = VMRobotControl.DEFAULT_GRAVITY
prob = get_ode_problem(cache, gravity, q, q̇, tspan; f_setup, f_control)
@info "Simulating compliant path following."
sol = solve(prob, Tsit5(); maxiters=2e5, abstol=1e-6, reltol=1e-6);

# ## Visualizing the simulation
# We will now visualize the simulation. We will create a `Figure` object, and add a `LScene` to it.
# We create an observable for the time, and an observable for the kinematics cache. These will
# be used by the `animate_robot_odesolution` function, which will update the observables, and 
# therefore any plots that depend upon them, as it plays back the simulation solution.

plotting_t = Observable(0.0)
plotting_kcache = Observable(new_kinematics_cache(compile(vms)))
plotting_vm_kcache = map(plotting_kcache) do k
    VMRobotControl.virtual_mechanism_cache(k)
end
cartID = get_compiled_coordID(plotting_kcache[], ".virtual_mechanism.CartPosition")

fig = Figure(; size=(720, 720), figure_padding=0)
display(fig)
ls = LScene(fig[1, 1]; show_axis=false)
cam = cam3d!(ls; center=false)
cam.eyeposition[] = [0.912748151284803, 1.0895512157983234, 0.8513286633206905]
cam.lookat[] = [0.09731484912188403, -0.18195162102725565, 0.17343471031108892];

# The cart is visualized as a red rectangle. The rail is shown by sketching the virtual mechanism,
# and the robot is visualized using the robotvisualize! function.
scatter!(ls, plotting_kcache, cartID; color=:red, marker=:rect, markersize=10)
robotvisualize!(ls, plotting_kcache)
robotsketch!(ls, plotting_vm_kcache);

# To plot the force arrow, we first get the TCP position, and then calculate the force at the TCP
# position using the disturbance function. We then plot the force arrow at the TCP position.
# The size of the arrow is scaled according to the magnitude of the force.
tcp_pos_id = get_compiled_coordID(plotting_kcache[], ".robot.TCP")
tcp_pos = map(plotting_kcache) do kcache
    Point3f(configuration(kcache, tcp_pos_id))
end
force = map(t -> 0.01 * Vec3f(disturbance_func(t)), plotting_t)
arrowsize = map(f -> 0.1*(f'*f)^(0.25), force)
arrows!(ls, map(p -> [p], tcp_pos), map(f -> [f], force); color = :red, arrowsize)

fps = 60
T = sol.t[end]
N_frames = Int(floor(fps * T))
ts = LinRange(0.0, T, N_frames)
savepath = joinpath(module_path, "docs/src/assets/compliant_path_following.mp4")
animate_robot_odesolution(fig, sol, plotting_kcache, savepath; t=plotting_t);

# ```@raw html
# <video controls width="100%" height="auto" autoplay loop>
# <source src="assets/compliant_path_following.mp4" type="video/mp4">
# </video>
# ```
