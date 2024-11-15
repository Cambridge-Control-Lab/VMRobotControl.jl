function test_rson(mechanism)
    # Tests serialisation 
    s = serializeRSON(String, mechanism)
    mechanism_2 = parseRSONString(s)

    m1 = compile(mechanism)
    m2 = compile(mechanism_2)

    @test ndof(m1) == ndof(m2)
    @test name(m1) == name(m2)

    q = rand!(zero_q(m1))
    q̇ = rand!(zero_q̇(m1))
    u = rand!(zero_u(m1))
    t = 0.0
    g = VMRobotControl.DEFAULT_GRAVITY

    kcache1 = new_kinematics_cache(m1)
    kcache2 = new_kinematics_cache(m2)

    kinematics!(kcache1, t, q)
    kinematics!(kcache2, t, q)

    if kcache1 isa VMRobotControl.MechanismCacheBundle
        tfs_1 = kcache1.cache.frame_cache.transforms
        tfs_2 = kcache2.cache.frame_cache.transforms
        @test all(tfs_1 .== tfs_2) 
    elseif kcache1 isa VMRobotControl.VirtualMechanismSystemCacheBundle
    else
        error("Unknown cache type")
    end

    dcache1 = new_dynamics_cache(m1)
    dcache2 = new_dynamics_cache(m2)

    q̈1 = dynamics!(dcache1, t, q, q̇, g, u)
    q̈2 = dynamics!(dcache2, t, q, q̇, g, u)

    if isa(q̈1, Tuple)
        q̈1ᵣ, q̈1ᵥ = q̈1
        q̈2ᵣ, q̈2ᵥ = q̈2
        # If any of q1 is NaN, then skip this test, as NaN != NaN
        any(isnan.(q̈1ᵣ)) || @test isapprox(q̈1ᵣ, q̈2ᵣ; atol=1e-8, rtol=1e-8)
        any(isnan.(q̈1ᵥ)) || @test isapprox(q̈1ᵥ, q̈2ᵥ; atol=1e-8, rtol=1e-8)
    else
        # If any of q1 is NaN, then skip this test, as NaN != NaN
        any(isnan.(q̈1)) || @test isapprox(q̈1, q̈2; atol=1e-8, rtol=1e-8)
    end

    for coord_id in keys(coordinates(mechanism))
        cID1 = get_compiled_coordID(m1, coord_id)
        cID2 = get_compiled_coordID(m2, coord_id)
        # @test cID1 == cID2 # This is not always true
        
        if VMRobotControl.has_configuration(cID1)
            @test configuration(dcache1, cID1) ≈ configuration(dcache2, cID2)
        end
        @test velocity(dcache1, cID1) ≈ velocity(dcache2, cID2)
        @test acceleration(dcache1, cID1) ≈ acceleration(dcache2, cID2)
        if dcache1 isa VMRobotControl.MechanismCacheBundle 
            # Jacobian for VMS is a tuple so can't be compared directly
            @test jacobian(dcache1, cID1) ≈ jacobian(dcache2, cID2)
        end
    end  
end