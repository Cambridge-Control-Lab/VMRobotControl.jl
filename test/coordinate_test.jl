using Random

using VMRobotControl

using StaticArrays
using Test

function test_coordinate(compiled_coord_ids::Vector{C}, dt, cache1, cache2, cache3, q̇3) where C<:VMRobotControl.CompiledCoord
    skip_configuration = ~VMRobotControl.has_configuration(C)
    @testset "allocs" begin
        foreach(compiled_coord_ids) do coord
            skip_configuration || (cnf_allocations = @allocations configuration(cache1, coord))
            jac_allocations = @allocations jacobian(cache1, coord)
            vel_allocations = @allocations velocity(cache1, coord)
            acc_allocations = @allocations acceleration(cache1, coord)
            skip_configuration || @test cnf_allocations == 0
            @test jac_allocations == 0
            @test vel_allocations == 0
            @test acc_allocations == 0
        end
    end
    
    if ~skip_configuration
        @testset "velocity" begin
            foreach(compiled_coord_ids) do coord
                z1 = configuration(cache1, coord)
                ż2 = velocity(cache2, coord)
                z3 = configuration(cache3, coord)
                @test (z3 - z1)/(2*dt) ≈ ż2 atol=1e-7 rtol=1e-7
            end
        end
    end

    @testset "acceleration" begin
        foreach(compiled_coord_ids) do coord
            ż1 = velocity(cache1, coord)
            z̈2 = acceleration(cache2, coord)
            ż3 = velocity(cache3, coord)
            @test (ż3 - ż1)/(2*dt) ≈ z̈2 atol=1e-7 rtol=1e-7
        end
    end

    @testset "Jacobian" begin
        foreach(compiled_coord_ids) do coord
            J3 = jacobian(cache3, coord)
            ż3 = velocity(cache3, coord)
            # If J3 is a tuple
            if isa(q̇3, Tuple)
                # Virtual mechanism
                @test J3*vcat(q̇3...) ≈ ż3 atol=1e-7 rtol=1e-7
            else
                @test J3 * q̇3 ≈ ż3 atol=1e-7 rtol=1e-7
            end
        end
    end
    nothing
end

function test_coordinates(m::Union{CompiledMechanism, CompiledVirtualMechanismSystem})
    rng = MersenneTwister(1234)
    NDOF = ndof(m)
    q = rand!(zero_q(Float64, m))
    q̇ = rand!(zero_q̇(Float64, m))
    u = rand!(zero_u(m))
    t = 0.0 # Time is irrelevant to these tests as no `TimeFuncJoint`s
    gravity = VMRobotControl.DEFAULT_GRAVITY

    cache1 = new_dynamics_cache(m)
    cache2 = new_dynamics_cache(m)
    cache3 = new_dynamics_cache(m)

    dt = 2e-8
    dynamics!(cache1, t,      q,                q̇, gravity, u)
    dynamics!(cache2, t+dt,   q .+ (dt .* q̇),   q̇, gravity, u)
    dynamics!(cache3, t+2*dt, q .+ ((2*dt).*q̇), q̇, gravity, u)

    # N_frames = num_frames(m)
    # frame_selection = sample(rng, 1:N_frames, min(N_frames, 5); replace=false, ordered=true)
    
    randvec3() = randn(rng, SVector{3, Float64})

    for tsc in VMRobotControl.coordinates(m)
        for coord_vec in tsc.tup
            coord_type = eltype(coord_vec)
            @testset "$(coord_type)" test_coordinate(coord_vec, dt, cache1, cache2, cache3, q̇)
        end
    end
    # TODO maybe randomly generate coordinates and add to mechanism for more thorough testing
    # p1, p2, p3, p4 = randvec3(), randvec3(), randvec3(), randvec3()
    # @testset "FrameOrigin" begin
    #     frame_origin_coords = [FrameOrigin(i, p1) for i in frame_selection]
    #     test_coordinate(frame_origin_coords, dt, cache1, cache2, cache3, q̇)
    # end
    # @testset "FrameAngularVelocity" begin
    #     frame_angular_velocity_coords = [FrameAngularVelocity(i) for i in frame_selection]
    #     test_coordinate(frame_angular_velocity_coords, dt, cache1, cache2, cache3, q̇; skip_configuration=true)
    # end
    # @testset "QuaternionAttitude" begin
    #     target_r = rand(rng, Rotor{Float64})
    #     quat_coords = [QuaternionAttitude(i, target_r) for i in frame_selection]
    #     test_coordinate(quat_coords, dt, cache1, cache2, cache3, q̇; skip_configuration=false)
    # end
    # @testset "CartesianOperationSpace" begin
    #     cartestian_coords = [CartesianOperationSpace(i, p1, p2) for i in frame_selection]
    #     test_coordinate(cartestian_coords, dt, cache1, cache2, cache3, q̇)
    # end
    # @testset "CoordDifference" begin
    #     cartesian_diff_coords = [FrameOrigin(i) -  FrameOrigin(rand(rng, frame_selection)) for i in frame_selection]
    #     test_coordinate(cartesian_diff_coords, dt, cache1, cache2, cache3, q̇)
    # end
    # @testset "LinkFrameCoordinate" begin
    #     linkframe_coords = [
    #         begin
    #             world_frame_coord = CartesianOperationSpace(i, p1, p2) - CartesianOperationSpace(rand(rng, frame_selection), p3, p4)
    #             LinkFrameCoordinate(world_frame_coord, i)
    #         end 
    #         for i in frame_selection
    #     ]
    #     test_coordinate(linkframe_coords, dt, cache1, cache2, cache3, q̇)
    # end
    # @testset "JointSpace" begin
    #     js_coord = JointSpace{ndof(m)}()
    #     test_coordinate([js_coord], dt, cache1, cache2, cache3, q̇)
    # end
    # @testset "JointSubspace" begin
    #     joint_dict = m.rbtree.joint_dict
    #     motive_joints = [v for (k, v) in joint_dict if velocity_size(v) > 0]
    #     jss_coords = map(JointSubspace, motive_joints)
    #     test_coordinate(jss_coords, dt, cache1, cache2, cache3, q̇)
    # end
    # nothing
end
