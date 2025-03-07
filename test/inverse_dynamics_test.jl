function test_inverse_dynamics(m; N=20)
    rng = MersenneTwister(1234)
    
    for i = 1:N
        t = 0.0
        q = rand!(rng, zero_q(Float64, m))
        q̇ = rand!(rng, zero_q̇(Float64, m))
        gravity = randn(rng, SVector{3, Float64})
        _test_inverse_dynamics(m, t, q, q̇, gravity)
    end
end

function _test_inverse_dynamics(m, time, q, q̇, gravity)
    u = zero_u(Float64, m)
    
    icache = new_inverse_dynamics_cache(m)
    dcache = new_dynamics_cache(m)

    dynamics!(dcache, time, q, q̇, gravity, u)
    q̈ = get_q̈(dcache)
    inverse_dynamics!(icache, time, q, q̇, q̈, gravity)

    # u is the control input used to compute forward dynamics, and is correct
    # get_u(icache) is the result of the inverse dynamics computation, and should be equal to u
    @test get_u(icache) ≈ u atol=1e-7 rtol=1e-7

    # @show icache.cache.frame_cache.forces[6] - icache.cache.frame_cache.rbstates[6].acceleration.linear


    # if isa(m, CompiledMechanism)
    # elseif isa(m, CompiledVirtualMechanismSystem)
    # end
end
