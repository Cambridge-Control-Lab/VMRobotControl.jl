function test_dynamics(m)
    rng = MersenneTwister(1234)
    
    q = rand!(rng, zero_q(Float64, m))
    q̇ = rand!(rng, zero_q̇(Float64, m))
    u = rand!(rng, zero_u(Float64, m)) 
    time = 0.0 # Time is irrelevant to these tests as no `TimeFuncJoint`s
    dt = 1e-8
    gravity = SVector(0.0, 0.0, -9.81)
    
    cache_0dt = new_dynamics_cache(m)
    cache_1dt = new_dynamics_cache(m)
    cache_2dt = new_dynamics_cache(m)

    
    dynamics!(cache_0dt, time,      q,                 q̇, gravity, u)
    dynamics!(cache_1dt, time+dt,   q .+ (dt.*q̇),      q̇, gravity, u)
    dynamics!(cache_2dt, time+2*dt, q .+ ((2*dt).*q̇),  q̇, gravity, u)
    
    if isa(m, CompiledMechanism)
        # Test velocity kinematics from this interface. May be redundant with the tests in velocity_kinematics_test.jl
        @testset "Frame \"$frame\"" for frame in frames(m)
            i = get_compiled_frameID(m, frame)
            _test_dynamics_frame(i, dt, q̇, gravity, cache_0dt, cache_1dt, cache_2dt)
            # TODO add more tests on q̈
        end        
        @test ~any(isnan.(get_q̈(cache_0dt)))
    elseif isa(m, CompiledVirtualMechanismSystem)
        let
            q̈ = get_q̈(cache_0dt)
            @test ~any(isnan.(q̈[1]))
            @test ~any(isnan.(q̈[2]))
        end
    end

    allocs = @allocated dynamics!(cache_0dt, time, q, q̇, gravity, u)
    @test allocs == 0
end

function _test_dynamics_frame(i, dt, q̇, gravity, cache1, cache2, cache3)
    tf1, tf2, tf3 = (p -> get_transform(p, i)).( (cache1, cache2, cache3) )
    v1, v2, v3 = (p -> get_linear_vel(p, i)).( (cache1, cache2, cache3) )
    w1, w2, w3 = (p -> get_angular_vel(p, i)).( (cache1, cache2, cache3) )
    v̇2 = get_linear_vpa(cache2, i)
    ẇ2 = get_angular_vpa(cache2, i)
    Jv1 = get_linear_jacobian(cache1, i)
    Jw1 = get_angular_jacobian(cache1, i)
    
    @test (origin(tf3) - origin(tf1))/(2*dt) ≈ v2   atol=1e-7 rtol=1e-7
    @test (v3 - v1)/(2*dt) ≈ v̇2                     atol=1e-7 rtol=1e-7
    @test (w3 - w1)/(2*dt) ≈ ẇ2                     atol=1e-7 rtol=1e-7
    @test v1 ≈ Jv1*q̇                                atol=1e-7 rtol=1e-7
    @test w1 ≈ Jw1*q̇                                atol=1e-7 rtol=1e-7
end