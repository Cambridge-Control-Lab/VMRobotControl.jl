# ## RNE Visualization

using
    GLMakie,
    LinearAlgebra,
    StaticArrays,
    VMRobotControl

X = SVector{3, Float64}(1.0, 0.0, 0.0)
Y = SVector{3, Float64}(0.0, 1.0, 0.0)
Z = SVector{3, Float64}(0.0, 0.0, 1.0)

T1 = zero(Transform{Float64})
J1 = Revolute(Y, T1)

T2 = Transform(Z)
J2 = Revolute(Y, T2)

T3 = Transform(Z)
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
cache = Observable(new_inverse_dynamics_cache(m))


begin
t = 0.0
q = [0.0, π/2]
q̇ = zero_q̇(m)
q̈ = zero_q̈(m)
g = SVector{3, Float64}(0.0, 0.0, -9.81)
inverse_dynamics!(cache[], t, q, q̇, q̈, g)

fig = Figure()
display(fig)
ls = LScene(fig[1, 1]; show_axis=false)
robotsketch!(ls, cache; linewidth=3)

frameIDs = get_compiled_frameID.((cache[],), frames(cache[]))
tfs = map(cache) do cache
    tfs = map(id -> get_transform(cache, id), frameIDs)
end
positions = map(tfs) do tfs
    map(tf -> Point3f(origin(tf)...), tfs)
end
fs = map(cache) do cache
    fs = map(id -> VMRobotControl.get_frame_force(cache, id), frameIDs)
end
f_scale = map(fs) do fs
    0.1/norm(maximum(fs))
end
τs = map(cache) do cache
    map(id -> VMRobotControl.get_frame_torque(cache, id), frameIDs)
end

arrows!(ls, positions, fs; lengthscale=f_scale, arrowsize=0.1, color=:red, )
arrows!(ls, positions, τs; lengthscale=f_scale, arrowsize=0.1, color=:red, )


end

using Test, Random
Revise.includet("../test/inverse_dynamics_test.jl")
@testset test_inverse_dynamics(m)

# function create_rotated_cylinder_mesh(N1, N2, r1, r2, ϕ)
#     for θ2 = LinRange(0, ϕ, N2)
#         pᶜ = Point3f(r2*cos(θ2), r2*sin(θ2), 0)
#         for θ1 = LinRange(0, 2π, N1)
#             points = Point3f()
#         end

#     end
# end



# module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])
# savepath = joinpath(module_path, "docs/src/assets/rail_robot.mp4")
# animate_robot_odesolution(fig, sol, cache, savepath; f_setup=animate_f_setup, f_control=animate_f_control);

# ```@raw html
# <video controls width="100%" height="auto" autoplay loop>
# <source src="../../assets/rail_robot.mp4" type="video/mp4">
# </video>
# ```
