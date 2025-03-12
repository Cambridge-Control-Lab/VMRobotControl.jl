function test_inverse_dynamics(m; N=3)
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
    skip_if_nan = any(isnan, get_q̈(dcache))
    skip_if_nan || @test get_q̈(dcache) ≈ q̈ atol=1e-7 rtol=1e-7

    ############################
    # Test 3
    # Run inverse dynamics with a single coordinate and compare to jacobian transpose method
    VMRobotControl._inverse_dynamics_set_inputs!(icache, time, q, 0 .* q̇, 0 .* q̈, 0 * gravity)
    VMRobotControl._inverse_dynamics_forward_pass!(icache)
    
    
    for coords in values(m.rbtree.coordinates)
        for vec in coords.tup
            for coord in vec
                _test_one_coord_vs_jacobian(icache, dcache, rng, coord)
            end
        end
    end
    
    ############################
    # Test 4
    # Test inertance matrix computed via RNE
    _test_RNE_inertance_matrix(icache, dcache, time, q)

    ############################
    # Test 5
    # Check no allocs
    allocs = @allocated inverse_dynamics!(icache, time, q, q̇, q̈, gravity)
    @test allocs == 0
end

function _test_one_coord_vs_jacobian(icache, dcache, rng, coord)
    VMRobotControl._inverse_dynamics_zero!(icache)
    # Apply random force to the coordinate
    f = randn(rng, length(coord))
    VMRobotControl.f_cache_view(icache, coord) .+= f
    VMRobotControl._inverse_dynamics_backward_pass_b!(icache)
    VMRobotControl._inverse_dynamics_backward_pass_c!(icache)
    VMRobotControl._inverse_dynamics_backward_pass_d!(icache)
    J = jacobian(dcache, coord)
    failed = !((@test get_u(icache) ≈ -J' * f) isa Test.Pass)
    failed && @info "Failed coord: $coord"
end


function _test_RNE_inertance_matrix(icache, dcache, t, q)
    N = ndof(icache)
    Mʳⁿᵉ = zeros(N, N)
    VMRobotControl._RNE_inertance_matrix!(Mʳⁿᵉ, icache, t, q)
    M = inertance_matrix!(dcache, t, q)
    @test Mʳⁿᵉ ≈ M atol=1e-7 rtol=1e-7
end
