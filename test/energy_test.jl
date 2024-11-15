"""
    test_energy(immut_mechanism)

Does an euler step simulation for 1 second and checks that the energy is conserved.
"""
function test_energy(system)
    rng = MersenneTwister(1234)
    gravity = SVector(0.0, 0.0, -9.81)
    cache = new_dynamics_cache(system)
    f_dynamics! = get_ode_dynamics(cache, gravity)
    T, dt = 1.0, 1e-4
    
    q = zero_q(system)
    q̇ = zero_q̇(system)
    if system isa CompiledMechanism
        randn!(rng, q)
        randn!(rng, q̇)
        q .*= 10.
        q̇ .*= 10.
    else
        randn!(rng, q[1])
        randn!(rng, q[2])
        randn!(rng, q̇[1])
        randn!(rng, q̇[2])
        q[1] .*= 10.
        q[2] .*= 10.
        q̇[1] .*= 10.
        q̇[2] .*= 10.
    end
    q0, q̇0 = deepcopy(q), deepcopy(q̇)
    x0 = assemble_state(system, q, q̇)
    ẋ = copy(x0)

    dynamics!(cache, 0.0, q0, q̇0, gravity)
    E1 = stored_energy(cache)

    q_idxs, q̇_idxs = state_idxs(system)

    q, q̇ = let
        local x
        x = x0
        ts = 0:dt:T
        for t in ts
            f_dynamics!(ẋ, x, nothing, t)
            x = x + (ẋ * dt)
        end
        x[q_idxs], x[q̇_idxs]
    end

    # prob = ODEProblem(dynamics, x0, T, abstol=1e-12, reltol=1e-12)
    # sol = solve(prob; save_everystep=false, maxiters=1000)
    # q = sol[q_idxs(immut_mechanism), end]
    # q̇ = sol[q̇_idxs(immut_mechanism), end]
    dynamics!(cache, T, q, q̇, gravity)
    E2 = stored_energy(cache)

    dissipative = false
    if system isa CompiledMechanism
        dissipative = !isempty(dissipations(system))
    else
        dissipative = dissipative || !isempty(dissipations(system))
        dissipative = dissipative || !isempty(dissipations(system.robot))
        dissipative = dissipative || !isempty(dissipations(system.virtual_mechanism))
    end
    if dissipative
        @test E2 <= E1
    else
        tol = 1.0 + (E1 * 0.01) # 1.0J + 1% of E1, to allow for numerical errors
        @test  E1 - tol < E2 < E1 + tol
    end
end
