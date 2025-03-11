# ## RNE Visualization

using
    GLMakie,
    LinearAlgebra,
    StaticArrays,
    VMRobotControl
# begin
#     X = SVector{3, Float64}(1.0, 0.0, 0.0)
#     Y = SVector{3, Float64}(0.0, 1.0, 0.0)
#     Z = SVector{3, Float64}(0.0, 0.0, 1.0)

#     T1 = zero(Transform{Float64})
#     J1 = Revolute(Y, T1)

#     T2 = Transform(Z)
#     J2 = Revolute(Y, T2)

#     T3 = Transform(Z)
#     J3 = Rigid(T3)

#     mech = Mechanism{Float64}("2Link")
#     cart_frame = add_frame!(mech, "Cart")
#     L2_frame = add_frame!(mech, "L2")
#     EE_frame = add_frame!(mech, "EE")

#     add_joint!(mech, J1; parent="root_frame",   child=cart_frame,               id="J1")
#     add_joint!(mech, J2; parent=cart_frame,     child=L2_frame,                 id="J2")
#     add_joint!(mech, J3, parent=L2_frame,       child=EE_frame;                 id="J3")

#     # Then, we add a coordinate to the mechanism to represent the tip of the pendulum, and a
#     # coordinate to represent the position of the cart. These are used to add point masses to
#     # the mechanism.
#     add_coordinate!(mech, FrameOrigin(EE_frame);                                id="tip_pos")
#     add_coordinate!(mech, FrameOrigin(cart_frame);                              id="cart_pos")
#     add_component!(mech, PointMass(1.0, "cart_pos");                            id="cart_mass")
#     add_component!(mech, PointMass(1.0, "tip_pos");                             id="pendulum_mass")
#     m = compile(mech)
# end
    
# We compile the mechanism, and setup an ODE problem to simulate the dynamics of the rail robot.
# m = compiled_mechanisms[5]
m = compile(parseRSON("./RSONs/rsons/trivial2link.rson"))
cache = Observable(new_inverse_dynamics_cache(m))
# @testset test_inverse_dynamics(m)

bundle = cache[]

begin


t = 0.0
q = [0.0, π/2]
q̇ = zero_q̇(m)
g = SVector{3, Float64}(0.0, 0.0, 0.0)

# rng = MersenneTwister(1234)
# t = 0.0
# q = rand!(rng, zero_q(Float64, m))
# q̇ = rand!(rng, zero_q̇(Float64, m))
# g = randn(rng, SVector{3, Float64})

u = zero_u(m)

# dcache = new_dynamics_cache(m)
# dynamics!(dcache, t, q, q̇, g, u)
# q̈ = get_q̈(dcache)
onehot_q̈ = [1.0, 0.0]
q̈ = onehot_q̈


VMRobotControl._inverse_dynamics_set_inputs!(bundle, t, q, q̇, q̈, g)
VMRobotControl._inverse_dynamics_forward_pass!(bundle)
VMRobotControl._inverse_dynamics_zero!(bundle)
VMRobotControl._inverse_dynamics_backward_pass_a!(bundle)
VMRobotControl._inverse_dynamics_backward_pass_b!(bundle)
# VMRobotControl._inverse_dynamics_backward_pass_c!(bundle)
# VMRobotControl._inverse_dynamics_backward_pass_d!(bundle)

# @show get_u(bundle)

# @show inverse_dynamics!(bundle, t, q, zero_q̇(m), onehot_q̈, 0*g)

# _test_RNE_inertance_matrix(bundle, dcache, t, q)

# q̈_out = dynamics!(dcache, t, q, q̇, g, get_u(bundle))

# @test get_u(cache[]) ≈ u atol=1e-7 rtol=1e-7
# @test q̈ ≈ q̈_out atol=1e-7 rtol=1e-7
end


begin
fig = Figure()
display(fig)
ls = LScene(fig[1, 1]; show_axis=false)
robotsketch!(ls, cache; linewidth=3, scale=0.05)
robotvisualize!(ls, cache)

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
    0.2/maximum(norm(fs))
end
τs = map(cache) do cache
    map(id -> VMRobotControl.get_frame_torque(cache, id), frameIDs)
end
τ_scale = map(τs) do τs
    0.4/maximum(norm(τs))
end


arrows!(ls, positions, fs; lengthscale=f_scale, arrowsize=map(f->*(f, 50), f_scale), color=:red, )
arrows!(ls, positions, τs; lengthscale=τ_scale, arrowsize=0.04, color=:blue, )


end

# using Test, Random
# Revise.includet("../test/inverse_dynamics_test.jl")
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
