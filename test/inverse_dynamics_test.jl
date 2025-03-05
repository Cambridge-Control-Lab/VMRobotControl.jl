function test_inverse_dynamics(m; N=20)
    rng = MersenneTwister(1234)
    
    for i = 1:N
        t = 0.0
        q = rand!(rng, zero_q(Float64, m))
        q̇ = rand!(rng, zero_q̇(Float64, m))
        q̈ = rand!(rng, zero_q̈(Float64, m))
        gravity = randn(rng, SVector{3, Float64})
        _test_inverse_dynamics(m, t, q, q̇, q̈, gravity; rng)
    end
end

function _test_inverse_dynamics(m, time, q, q̇, q̈, gravity; rng=MersenneTwister(1234))
    u = zero_u(Float64, m)
    
    icache = new_inverse_dynamics_cache(m)
    dcache = new_dynamics_cache(m)    

    ############################
    # Test 1
    # Run forward dynamics then inverse dynamics, and compare the control input
    dynamics!(dcache, time, q, q̇, gravity, u)
    inverse_dynamics!(icache, time, q, q̇, get_q̈(dcache), gravity)
    @test get_u(icache) ≈ u atol=1e-7 rtol=1e-7


    ############################
    # Test 2
    # Run inverse dynamics then forward dynamics, and compare the acceleration
    inverse_dynamics!(icache, time, q, q̇, q̈, gravity)
    dynamics!(dcache, time, q, q̇, gravity, get_u(icache))
    @test get_q̈(dcache) ≈ q̈ atol=1e-7 rtol=1e-7

    ############################
    # Test 3
    # Run inverse dynamics with a single coordinate and compare to jacobian transpose method
    VMRobotControl._inverse_dynamics_set_inputs!(icache, time, q, 0 .* q̇, 0 .* q̈, 0 * gravity)
    VMRobotControl._inverse_dynamics_forward_pass!(icache)
    for coordID in values(m.rbtree.coord_id_map)
        VMRobotControl._inverse_dynamics_zero!(icache)
        # Apply random force to the coordinate
        f = randn(rng, length(icache[coordID]))
        VMRobotControl.f_cache_view(icache, icache[coordID]) .+= f
        VMRobotControl._inverse_dynamics_backward_pass_a!(icache)
        VMRobotControl._inverse_dynamics_backward_pass_b!(icache)
        VMRobotControl._inverse_dynamics_backward_pass_c!(icache)
        VMRobotControl._inverse_dynamics_backward_pass_d!(icache)
        J = jacobian(dcache, coordID)
        @test get_u(icache) ≈ -J' * f
    end
    # if isa(m, CompiledMechanism)
    # elseif isa(m, CompiledVirtualMechanismSystem)
    # end
end
