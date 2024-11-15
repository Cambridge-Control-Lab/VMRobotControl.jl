function test_inverse_dynamics(m)
    rng = MersenneTwister(1234)
    
    # q = rand!(rng, zero_q(Float64, m))
    # q̇ = rand!(rng, zero_q̇(Float64, m))
    # u = rand!(rng, zero_u(Float64, m))
    # u = zero_u(Float64, m)
    q = [0., π/2]
    q̇ = [0., 0.]
    u = [1., 0.]
    time = 0.0 # Time is irrelevant to these tests as no `TimeFuncJoint`s
    gravity = SVector(0.0, 0.0, -10.)
    
    icache = new_inverse_dynamics_cache(m)
    dcache = new_dynamics_cache(m)

    dynamics!(dcache, time, q, q̇, gravity, u)
    q̈ = get_q̈(dcache)
    inverse_dynamics!(icache, time, q, q̇, q̈, gravity)

    @show get_u(icache)
    @show u
    @test get_u(icache) ≈ u atol=1e-7 rtol=1e-7

    # @show icache.cache.frame_cache.forces[6] - icache.cache.frame_cache.rbstates[6].acceleration.linear


    # if isa(m, CompiledMechanism)
    # elseif isa(m, CompiledVirtualMechanismSystem)
    # end
end
