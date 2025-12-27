function test_velocity_kinematics_vs_autodiff(m::CompiledMechanism)
    # compares velocity kinematics to autodiff.
    rng = MersenneTwister(1234)
    NDOF = ndof(m)    
    t = 0.0 # Time is irrelevant to these tests as no `TimeFuncJoint`s
    q = randn(rng, Float64, NDOF)
    q̇ = randn(rng, Float64, NDOF)
    gravity = zero(SVector{3, Float64}) 
    # Set to zero so that we can check that the acceleration is
    # equal to VPA, otherwise inertial frame acceleration (how gravity is implemented )gets in the way 
    
    cache = new_rbstates_cache(m)
    
    velocity_kinematics!(cache, t, q, q̇)
    allocs = @allocated velocity_kinematics!(cache, t, q, q̇) # Make sure is already compiled
    @test allocs == 0
    
    rbs2 = VMRobotControl.__autodiff_extended_kinematics(m.rbtree.joint_collection,
                                                         m.rbtree.walk, t, q, q̇)


    @testset "Frame \"$frame\"" for frame in frames(m)
        i = get_compiled_frameID(m, frame)
        v1, v2 = get_linear_vel(cache, i), rbs2[i].twist.linear
        @test v1 ≈ v2 atol=1e-8 rtol=1e-8
        w1, w2 = get_angular_vel(cache, i), rbs2[i].twist.angular
        @test w1 ≈ w2 atol=1e-8 rtol=1e-8
        # Velocity product accelerations
        α1, α2 = _get_linear_acc(cache, i), rbs2[i].acceleration.linear
        @test α1 ≈ α2 atol=1e-8 rtol=1e-8            
        Ω1, Ω2 = _get_angular_acc(cache, i), rbs2[i].acceleration.angular
        @test Ω1 ≈ Ω2 atol=1e-8 rtol=1e-8
    end
end

function test_velocity_kinematics_vs_finitediff(m::CompiledMechanism)
    rng = MersenneTwister(1234)
    NDOF = ndof(m)
    q_type = SVector{NDOF, Float64}
    q̇_type = SVector{NDOF, Float64}
    
    q = randn(rng, q_type)
    q̇ = randn(rng, q̇_type)
    t = 0.0 # Time is irrelevant to these tests as no `TimeFuncJoint`s
    dt = 1e-8
    gravity = SVector(0.0, 0.0, 0.0)
    
    cache = new_rbstates_cache(m)
    velocity_kinematics!(cache, t,      q,        q̇)
    cache1 = deepcopy(cache)
    velocity_kinematics!(cache, t+dt,   q+dt*q̇,   q̇)
    cache2 = deepcopy(cache)
    velocity_kinematics!(cache, t+2*dt, q+2*dt*q̇, q̇)
    cache3 = deepcopy(cache)

    @testset "Frame \"$frame\"" for frame in frames(m)
        i = get_compiled_frameID(m, frame)
        tf1, tf3 = get_transform.( (cache1, cache3), (i,))
        v1, v2, v3 = get_linear_vel.( (cache1, cache2, cache3), (i,) )
        w1, w2, w3 = get_angular_vel.( (cache1, cache2, cache3), (i,) )
        α2 = _get_linear_acc(cache2, i)
        Ω2 = _get_angular_acc(cache2, i)
        
        @test (origin(tf3) - origin(tf1))/(2*dt) ≈ v2 atol=1e-7 rtol=1e-7
        # TODO test angular velocity vs rotor derivative, somehow? Or just trust
        # Velocity product accelerations
        @test (v3 - v1)/(2*dt) ≈ α2 atol=1e-7 rtol=1e-7
        @test (w3 - w1)/(2*dt) ≈ Ω2 atol=1e-7 rtol=1e-7
    end
end
