
using DifferentialEquations
using GLMakie
using LinearAlgebra

using VMRobotControl
using VMRobotControl.Splines: CubicSpline
using StaticArrays

module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])
robot = parseRSON(joinpath(module_path, "RSONs/rsons/simple2link.rson"))
empty!(dissipations(robot))
add_gravity_compensation!(robot, VMRobotControl.DEFAULT_GRAVITY)
add_coordinate!(robot, FrameOrigin("EE_frame"); id="TCP")

vms = VirtualMechanismSystem("System", deepcopy(robot))

ref_coord = ReferenceCoord(Ref(SVector(0.8, 0.0, 0.8)))
cartesian_err_coord = CoordDifference("ref", ".robot.TCP")
absolute_err_coord = CoordNorm("cartesian_err")
rest_len = ConstCoord(SVector(0.0))
absolute_err_offset = CoordDifference("absolute_err", "rest_len")
add_coordinate!(vms, ref_coord; id="ref")
add_coordinate!(vms, cartesian_err_coord;   id="cartesian_err")
add_coordinate!(vms, absolute_err_coord;    id="absolute_err")
add_coordinate!(vms, rest_len;              id="rest_len")
add_coordinate!(vms, absolute_err_offset;   id="absolute_err_offset")



## Now make 2 versions, one using the cartesian coordinates, and one using the absoulte coordinates
vms1 = deepcopy(vms)
vms2 = deepcopy(vms)
stiffness = 100.0
damping = 30.0
add_component!(vms1, LinearSpring(stiffness, "cartesian_err");     id="err_spring")
add_component!(vms1, LinearDamper(damping, "cartesian_err");     id="err_damper")

add_component!(vms2, LinearSpring(stiffness, "absolute_err_offset");     id="err_spring")
add_component!(vms2, LinearDamper(damping, "absolute_err_offset");     id="err_damper")


function my_simulate(
        vms;
        tspan = 10.0,
        q = nothing,
        q̇ = nothing,
        gravity = VMRobotControl.DEFAULT_GRAVITY
    )
    cache = new_dynamics_cache(compile(vms))
    q = isnothing(q) ? zero_q(cache) : q
    q̇ = isnothing(q̇) ? zero_q̇(cache) : q̇
    prob = get_ode_problem(cache, gravity, q, q̇, tspan)
    solve(prob, Tsit5(); maxiters=2e5, abstol=1e-6, reltol=1e-6);
end

sol1 = my_simulate(vms1; q = ([0.0, 1.57], Float64[]), q̇ = ([10.0, 0.], Float64[]))
sol2 = my_simulate(vms2; q = ([0.0, 1.57], Float64[]), q̇ = ([10.0, 0.], Float64[]))

plotting_t = Observable(0.0)
plotting_dcache_1 = Observable(new_dynamics_cache(compile(vms1)))
plotting_dcache_2 = Observable(new_dynamics_cache(compile(vms2)))

begin
fig = Figure(; size=(2*720, 720), figure_padding=0)
display(fig)
ls1 = LScene(fig[1, 1]; show_axis=false)
ls2 = LScene(fig[1, 2]; show_axis=false)
text!(ls1.scene, Point2f(0.0, 1.0); text="Cartesian Spring/Damper", align=(:left, :top), color=:black, fontsize=20, space=:relative)
text!(ls2.scene, Point2f(0.0, 1.0); text="Distance Spring/Damper", align=(:left, :top), color=:black, fontsize=20, space=:relative)

alpha = 0.5
red = RGBAf(1, 0, 0, alpha)
cyan = RGBAf(0, 1, 1, alpha)
magenta = RGBAf(1, 0, 1, alpha)


for (ls, dcache) in zip((ls1, ls2), (plotting_dcache_1, plotting_dcache_2))
    cam = cam3d!(ls; center=false)
    cam.eyeposition[] = 2.5*[0.912748151284803, 1.0895512157983234, 0.8513286633206905]
    cam.lookat[] = [0.09731484912188403, -0.18195162102725565, 0.4];

    robotsketch!(ls, dcache);
    # The cart is visualized as a red rectangle. The rail is shown by sketching the virtual mechanism,
    # and the robot is visualized using the robotvisualize! function.
    ref_ID = get_compiled_coordID(dcache[], "ref")
    tcp_ID = get_compiled_coordID(dcache[], ".robot.TCP")
    cartesian_err_ID = get_compiled_coordID(dcache[], "cartesian_err")
    err_spring_ID = get_compiled_componentID(dcache[], "err_spring")
    err_damper_ID = get_compiled_componentID(dcache[], "err_damper")
    
    scatter!(ls, dcache, [tcp_ID]; color=:blue, markersize=10)
    scatter!(ls, dcache, [ref_ID]; color=:red, markersize=10)
    
    # Now do an arrow for the force
    tcp_pos = map(dcache) do dcache
        Point3f(configuration(dcache, tcp_ID))
    end
    forces = map(dcache) do dcache
        spring_force = opspace_force(dcache, err_spring_ID)
        damper_force = opspace_force(dcache, err_damper_ID)
        @assert length(spring_force) == length(damper_force)
        if length(damper_force) == 3
            Fs = spring_force
            Fd = damper_force
            return [Fs, Fd, Fs.+Fd]
        elseif length(damper_force) == 1
            err = configuration(dcache, cartesian_err_ID)
            err_dir = err / norm(err)
            Fs = spring_force[1] * err_dir
            Fd = damper_force[1] * err_dir
            return [Fs, Fd, Fs.+Fd]
        else
            error("Unknown force length")
        end
    end
    arrows!(
        ls, 
        map(p -> [p, p, p], tcp_pos), # Position
        map(f -> (0.01*norm(f)) * f / norm(f), forces); # Direction/magnitude
        arrowsize = map(fs -> [0.05 + (0.01*norm(f)) for f in fs], forces), # Thickness
        color = [red, cyan, magenta],
    )
end
poly!(ls1, Point3f[]; color=red, label="Spring force") ## Just for legend entry
poly!(ls1, Point3f[]; color=cyan, label="Damper force") ## Just for legend entry
poly!(ls1, Point3f[]; color=magenta, label="Total force") ## Just for legend entry
leg = Legend(fig[1, 1], ls1; merge=true, tellwidth=false, halign=:left, valign=:bottom)


begin ## Animate
    _q_idxs, _q̇_idxs = state_idxs(plotting_dcache_1[])
    q_q̇_from_t_odestate = (t, x)-> (x[_q_idxs], x[_q̇_idxs])
    # Determine vector of frame times
    Tmax1 = maximum(sol1.t)
    Tmax2 = maximum(sol1.t)
    @assert Tmax1 ≈ Tmax2
    fps = 60
    N_frames = Int(floor(fps*Tmax1))
    ts = LinRange(0.0, Tmax1, N_frames)
    # Run f_setup callback, and record (which will call f_control callback)
    savepath = joinpath(module_path, "docs/src/assets/cartesian_vs_linear_springs.mp4")
    args1 = VMRobotControl.DEFAULT_F_SETUP(plotting_dcache_1)
    args2 = VMRobotControl.DEFAULT_F_SETUP(plotting_dcache_2)
    update = (frame) -> begin
        VMRobotControl._update_cache_from_framenumber!(plotting_dcache_1, frame, plotting_t, ts, sol1, q_q̇_from_t_odestate, VMRobotControl.DEFAULT_F_CONTROL, args1)
        VMRobotControl._update_cache_from_framenumber!(plotting_dcache_2, frame, plotting_t, ts, sol2, q_q̇_from_t_odestate, VMRobotControl.DEFAULT_F_CONTROL, args2)
    end
    record(update, fig, savepath, 1:length(ts), framerate=fps; visible=true)    
end
end
# ```@raw html
# <video controls width="100%" height="auto" autoplay loop>
# <source src="assets/compliant_path_following.mp4" type="video/mp4">
# </video>
# ```
