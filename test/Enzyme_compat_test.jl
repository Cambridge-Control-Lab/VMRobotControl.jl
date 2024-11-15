function enzyme_kinematics_test(m, t, q)
    cache = new_kinematics_cache(m)
    kinematics!(cache, t, q)
    function f(cache, _t, _q)
        kinematics!(cache, _t, _q)
        return nothing
    end
    autodiff(Reverse, f, Duplicated(cache, deepcopy(cache)), Active(t), Duplicated(q, deepcopy(q)))
end

function enzyme_velocity_kinematics_test(m, t, q, q̇)
    isa(m, CompiledVirtualMechanismSystem) && return # This is not implemented
    cache = new_rbstates_cache(m)
    velocity_kinematics!(cache, t, q, q̇)
    function f(_cache, _t, _q, _q̇)
        velocity_kinematics!(_cache, _t, _q, _q̇)
        return nothing
    end
    autodiff(Reverse, f, Duplicated(cache, deepcopy(cache)), Active(t), Duplicated(q, deepcopy(q)), Duplicated(q̇, deepcopy(q̇)))
end

function enzyme_acceleration_kinematics_test(m, t, q, q̇, q̈)
    isa(m, CompiledVirtualMechanismSystem) && return # This is not implemented
    cache = new_rbstates_cache(m)
    acceleration_kinematics!(cache, t, q, q̇, q̈)
    function f(_cache, _t, _q, _q̇, _q̈)
        acceleration_kinematics!(_cache, _t, _q, _q̇, _q̈)
        return nothing
    end
    autodiff(Reverse, f, Duplicated(cache, deepcopy(cache)), Active(t), Duplicated(q, deepcopy(q)), Duplicated(q̇, deepcopy(q̇)), Duplicated(q̈, deepcopy(q̈)))
end

function enzyme_jacobians_test(m, t, q)
    isa(m, CompiledVirtualMechanismSystem) && return # This is not implemented
    cache = new_jacobians_cache(m)
    jacobians!(cache, t, q)
    function f(_cache, _t, _q)
        jacobians!(_cache, _t, _q)
        return nothing
    end
    autodiff(Reverse, f, Duplicated(cache, deepcopy(cache)), Active(t), Duplicated(q, deepcopy(q)))
end

function enzyme_precomp_test(m, t, q, q̇, gravity)
    cache = new_dynamics_cache(m)
    precompute!(cache, t, q, q̇, gravity)
    function f(_cache, _t, _q, _q̇, _gravity)
        precompute!(_cache, _t, _q, _q̇, _gravity)
        return nothing
    end
    autodiff(Reverse, f, Duplicated(cache, deepcopy(cache)), Active(t), Duplicated(q, deepcopy(q)), Duplicated(q̇, deepcopy(q̇)), Active(gravity))
end

function enzyme_inertance_matrix_test(m, t, q)
    cache = new_dynamics_cache(m)
    inertance_matrix!(cache, t, q)
    function f(_cache, _t, _q)
        M = inertance_matrix!(_cache, _t, _q)
        isa(M, Tuple) && return M[1][1, 1]
        return M[1, 1]
    end
    autodiff(Reverse, f, Duplicated(cache, deepcopy(cache)), Active(t), Duplicated(q, deepcopy(q)))
end

function enzyme_generalized_force_test(m, t, q, q̇, gravity)
    cache = new_dynamics_cache(m)
    generalized_force!(cache, t, q, q̇, gravity)
    function f(_cache, _t, _q, _q̇, _gravity)
        τ = generalized_force!(_cache, _t, _q, _q̇, _gravity)
        isa(τ, Tuple) && return τ[1][1]
        return τ[1]
    end
    autodiff(Reverse, f, Duplicated(cache, deepcopy(cache)), Active(t), Duplicated(q, deepcopy(q)), Duplicated(q̇, deepcopy(q̇)), Active(gravity))
end

function enzyme_dynamics_test(m, t, q, q̇, gravity, u)
    cache = new_dynamics_cache(m)
    dynamics!(cache, t, q, q̇, gravity, u)
    function f(_cache, _t, _q, _q̇, _gravity, _u)
        q̈ = dynamics!(_cache, _t, _q, _q̇, _gravity, _u)
        return q̈[1]
    end
    autodiff(Reverse, f, Duplicated(cache, deepcopy(cache)), Active(t), Duplicated(q, deepcopy(q)), Duplicated(q̇, deepcopy(q̇)), Active(gravity), Duplicated(u, deepcopy(u)))
end

function __enzyme_test_mechanism(m)
    t = 0.0
    q = zero_q(m)
    q̇ = zero_q̇(m)
    q̈ = zero_q̈(m)
    gravity = VMRobotControl.DEFAULT_GRAVITY
    u = zero_u(m)
    @test_nowarn enzyme_kinematics_test(m, t, q)
    @test_nowarn enzyme_velocity_kinematics_test(m, t, q, q̇)
    @test_nowarn enzyme_acceleration_kinematics_test(m, t, q, q̇, q̈)
    @test_nowarn enzyme_jacobians_test(m, t, q)
    # @test_nowarn enzyme_precomp_test(m, t, q, q̇, gravity)
    # @test_nowarn enzyme_generalized_force_test(m, t, q, q̇, gravity)
    # @test_nowarn enzyme_inertance_matrix_test(m, t, q) # This is broken right now...
    # @test_nowarn enzyme_dynamics_test(m, t, q, q̇, gravity, u)
end

function test_enzyme_compat(immut_mechanism)
    __enzyme_test_mechanism(immut_mechanism)
end