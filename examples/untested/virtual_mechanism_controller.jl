using Revise
using DifferentialEquations
using GLMakie
using VMRobotControl
using StaticArrays

# Load the mechanism
mechanism = parseRSON("../RSONs/rsons/pendulum.rson");
add_coordinate!(mechanism, FrameOrigin("EE_frame"); id="tip")
add_coordinate!(mechanism, JointSubspace("J1"); id="J1_angle")

# The virtual mechanism in this case is a copy of the original mechanism.
virtual_mechanism = deepcopy(mechanism)
for (key, c) in components(virtual_mechanism)
    if c isa Visual
        red_visual = VMRobotControl.remake(c; color = RGBAf(1.0, 0.0, 0.0, 1.0))        
        components(virtual_mechanism)[key] = red_visual
    end
end

# Add a spring to the virtual mechanism, holding J1 at zero angle
add_component!(virtual_mechanism, LinearSpring(100.0, "J1_angle"); id="J1 spring")
# Add a damper to the real mechanism
add_component!(mechanism, LinearDamper(2.0, "J1_angle"); id="damper")

vms = VirtualMechanismSystem("ControlledPendulum", mechanism, virtual_mechanism)
add_coordinate!(vms, CoordDifference(".robot.tip", ".virtual_mechanism.tip"); id="tip_err")

add_component!(vms, LinearSpring(100.0, "tip_err"); id="tip spring")
cvms = compile(vms)

# Setup the plot
f = Figure()
ax = LScene(f[1, 1])

# # Initial state
# x0 = SVector{4, Float64}(0.0, 1.0, 0.5, 0.0)

# Make the initial plot
q = zero_q(cvms)

cache = Observable(new_kinematics_cache(cvms))

robotvisualize!(ax, cache)

robot_tip = get_compiled_coordID(cvms, ".robot.tip")
virtual_tip = get_compiled_coordID(cvms, ".virtual_mechanism.tip")
scatterlines!(ax, cache, [robot_tip, virtual_tip], color=:red, linestyle=:dash)

# Now get the dynamics, and simluate
dcache = new_dynamics_cache(cvms)
prob = get_ode_problem(dcache, VMRobotControl.DEFAULT_GRAVITY, ([0.0], [1.0]), ([0.5], [0.0]), 8.0)
tols = (:abstol => 1e-6, :reltol => 1e-6)
sol = solve(prob, Tsit5(); maxiters=3e3, tols...)

# Animate the simulation
animate_robot_odesolution(f, sol, cache, "./tmp.mp4"; fps=60)