using LinearAlgebra
using VMRobotControl

using VMRobotControl: to_rotation_matrix, skew # For scara_tests
using VMRobotControl: transform, linear_vel, angular_vel, origin

using StaticArrays
using Test


function scara_test()
    X, Y, Z = SVector(1., 0., 0.), SVector(0., 1., 0.), SVector(0., 0., 1.)

    a₁ = Y
    a₂ = Z

    l₁ = Z
    l₂ = Z

    m₁ = 1.0
    m₂ = 2.0

    I₁ = @SMatrix [1.0 0.0 0.0;
                0.0 2.0 0.0;
                0.0 0.0 3.0]
    I₂ = @SMatrix [1.0 0.0 0.0;
                0.0 2.0 0.0;
                0.0 0.0 3.0]

    m = begin 
        mechanism = Mechanism{Float64}("mechanism")
        add_frame!(mechanism, "L1")
        add_frame!(mechanism, "L2")
        add_frame!(mechanism, "EE")
        add_joint!(mechanism, Revolute(a₁); parent=root_frame(mechanism), child="L1", id="J1")
        add_joint!(mechanism, Revolute(a₂, Transform(l₁)); parent="L1", child="L2", id="J2")
        add_joint!(mechanism, Rigid(Transform(l₂)); parent="L2", child="EE", id="EE_joint")
        
        add_coordinate!(mechanism, FrameAngularVelocity("L1"); id="L1_ω")
        add_coordinate!(mechanism, FrameAngularVelocity("L2"); id="L2_ω")

        add_coordinate!(mechanism, FrameOrigin("L1"); id="L1_pos")
        add_coordinate!(mechanism, FrameOrigin("L2"); id="L2_pos")
        add_coordinate!(mechanism, FrameOrigin("EE"); id="EE_pos")

        inertia_1 = Inertia(I₁, "L1_ω")
        inertia_2 = Inertia(I₂, "L2_ω")
        mass_1 = PointMass(m₁, "L2_pos")
        mass_2 = PointMass(m₂, "EE_pos")

        add_component!(mechanism, inertia_1; id="inertia_1")
        add_component!(mechanism, inertia_2; id="inertia_2")
        add_component!(mechanism, mass_1; id="mass_1")
        add_component!(mechanism, mass_2; id="mass_2")
        compile(mechanism)
    end

    cache = new_dynamics_cache(m)

    L1 = get_compiled_frameID(m, "L1")
    L2 = get_compiled_frameID(m, "L2")
    LEE = get_compiled_frameID(m, "EE")

    mass_1_id = get_compiled_componentID(m, "mass_1")
    mass_2_id = get_compiled_componentID(m, "mass_2")
    inertia_1_id = get_compiled_componentID(m, "inertia_1")
    inertia_2_id = get_compiled_componentID(m, "inertia_2")

    # Test setup
    t = 0.
    q = SVector(0., π/2)
    q̇ = SVector(1., 1.)
    u = SVector(0., 0.)
    gravity = SVector(0., 0., -10.)
    dynamics!(cache, t, q, q̇, gravity, u) # Computes the velocity kinematics and jacobians
    u_computed = copy(get_generalized_force(cache))
    M_computed = copy(get_inertance_matrix(cache))

    # Transform by hand
    tf₁ = Transform(Rotor(; axis=a₁, angle=q[1]))
    tf₂ = tf₁ * Transform(l₁) * Transform(Rotor(; axis=a₂, angle=q[2]))
    tfₑₑ = tf₂ * Transform(l₂)

    @test tf₁ == get_transform(cache, L1)
    @test tf₂ == get_transform(cache, L2)
    @test tfₑₑ == get_transform(cache, LEE)

    # Velocities by hand
    a₁ʷ = a₁
    a₂ʷ = rotor(tf₁) * a₂
    l₁ʷ = rotor(tf₁) * l₁
    l₂ʷ = rotor(tf₂) * l₂

    v₁ = SVector(0., 0., 0.)
    ω₁ = q̇[1] * a₁ʷ

    v₂ = cross(ω₁, l₁ʷ)
    ω₂ = ω₁ + q̇[2] * a₂ʷ

    vₑₑ = v₂ + cross(ω₂, l₂ʷ)

    @test v₁ == get_linear_vel(cache, L1)
    @test v₂ == get_linear_vel(cache, L2)
    @test vₑₑ ≈ get_linear_vel(cache, LEE)

    # Velocity product acceleration by hand
    v̇₁ = SVector(0., 0., 0.)
    ẇ₁ = SVector(0., 0., 0.)

    ȧ₁ʷ = SVector(0., 0., 0.)
    ȧ₂ʷ = cross(ω₁, a₂)

    l̇₁ʷ = cross(ω₁, l₁ʷ)
    l̇₂ʷ = cross(ω₂, l₂ʷ)

    v̇₂ = v̇₁ + cross(ω₁, l̇₁ʷ)
    ẇ₂ = ẇ₁ + q̇[2] * ȧ₂ʷ

    v̇ₑₑ = v̇₂ + cross(ẇ₂, l₂ʷ) + cross(ω₂, l̇₂ʷ)

    @test v̇₁ ≈ get_linear_vpa(cache, L1)
    @test v̇₂ ≈ get_linear_vpa(cache, L2)
    @test v̇ₑₑ ≈ get_linear_vpa(cache, LEE)
    @test ẇ₁ ≈ get_angular_vpa(cache, L1)
    @test ẇ₂ ≈ get_angular_vpa(cache, L2)

    # Opspace forces by hand
    I₁ʷ, İ₁ʷ = let 
        R = to_rotation_matrix(rotor(tf₁))
        Ṙ = skew(ω₁) * R
        I = R * I₁ * R'
        İ = Ṙ * I₁ * R' + R * I₁ * Ṙ'
        I, İ    
    end
    I₂ʷ, İ₂ʷ = let 
        R = to_rotation_matrix(rotor(tf₂))
        Ṙ = skew(ω₂) * R
        I = R * I₂ * R'
        İ = Ṙ * I₂ * R' + R * I₂ * Ṙ'
        I, İ
    end

    F₁ = m₁ * (v̇₂ - gravity)
    τ₁ = (I₁ʷ * ẇ₁ + İ₁ʷ * ω₁)
    F₂ = m₂ * (v̇ₑₑ - gravity)
    τ₂ = (I₂ʷ * ẇ₂ + İ₂ʷ * ω₂)

    @test F₁ ≈ opspace_force(cache, m[mass_1_id])
    @test F₂ ≈ opspace_force(cache, m[mass_2_id])
    @test τ₁ ≈ opspace_force(cache, m[inertia_1_id])
    @test τ₂ ≈ opspace_force(cache, m[inertia_2_id]) 

    # Generalized forces by hand
    u_mass_1 = SVector(
        dot(a₁ʷ, cross(l₁ʷ, F₁) ),
        0.
    )
    u_mass_2 = SVector(
        dot(a₁ʷ, cross(l₁ʷ + l₂ʷ, F₂) ),
        dot(a₂ʷ, cross(l₂ʷ, F₂) )
    )
    u_inertia_1 = SVector(
        dot(a₁ʷ, τ₁),
        0.
    )
    u_inertia_2 = SVector(
        dot(a₁ʷ, τ₂),
        dot(a₂ʷ, τ₂)
    )

    @test u_mass_1 ≈ generalized_force!(zeros(2), cache, m[mass_1_id]) atol=1e-8
    @test u_mass_2 ≈ generalized_force!(zeros(2), cache, m[mass_2_id]) atol=1e-8
    @test u_inertia_1 ≈ generalized_force!(zeros(2), cache, m[inertia_1_id]) atol=1e-10
    @test u_inertia_2 ≈ generalized_force!(zeros(2), cache, m[inertia_2_id]) atol=1e-10

    u = u_mass_1 + u_mass_2 + u_inertia_1 + u_inertia_2
    @test u ≈ u_computed atol=1e-8

    # Inertance matrix by hand
    M₁₁_mass_1 = norm(cross(l₁ʷ, a₁ʷ))^2 * m₁
    M₁₁_mass_2 = norm(cross(l₁ʷ + l₂ʷ, a₁ʷ))^2 * m₂
    M₁₁_inertia_1 = dot(a₁ʷ, I₁ʷ, a₁ʷ)
    M₁₁_inertia_2 = dot(a₁ʷ, I₂ʷ, a₁ʷ)

    M₂₂_mass_2 = norm(cross(l₂ʷ, a₂ʷ))^2 * m₂
    M₂₂_inertia_2 = dot(a₂ʷ, I₂ʷ, a₂ʷ)

    @test M₁₁_mass_1 ≈ inertance_matrix!(zeros(2, 2), cache, m[mass_1_id])[1, 1]
    @test M₁₁_mass_2 ≈ inertance_matrix!(zeros(2, 2), cache, m[mass_2_id])[1, 1]
    @test M₁₁_inertia_1 ≈ inertance_matrix!(zeros(2, 2), cache, m[inertia_1_id])[1, 1]
    @test M₁₁_inertia_2 ≈ inertance_matrix!(zeros(2, 2), cache, m[inertia_2_id])[1, 1]
    @test M₂₂_mass_2 ≈ inertance_matrix!(zeros(2, 2), cache, m[mass_2_id])[2, 2]
    @test M₂₂_inertia_2 ≈ inertance_matrix!(zeros(2, 2), cache, m[inertia_2_id])[2, 2]

    @test M_computed[1, 1] ≈ M₁₁_mass_1 + M₁₁_mass_2 + M₁₁_inertia_1 + M₁₁_inertia_2
    @test M_computed[2, 2] ≈ M₂₂_mass_2 + M₂₂_inertia_2 

    # Stored energy by hand
    E_mass_1 = 0.5 * m₁ * dot(v₂, v₂)
    E_inertia_1 = 0.5 * dot(ω₁, I₁ʷ, ω₁)
    E_mass_2 = 0.5 * m₂ * dot(vₑₑ, vₑₑ)
    E_inertia_2 = 0.5 * dot(ω₂, I₂ʷ, ω₂)
    PE₁ = m₁ * dot(-gravity, origin(tf₂))
    PE₂ = m₂ * dot(-gravity, origin(tfₑₑ))
    E = E_mass_1 + E_inertia_1 + PE₁ + E_mass_2 + E_inertia_2 + PE₂

    @test E_inertia_1 ≈ stored_energy(cache, m[inertia_1_id])
    @test E_inertia_2 ≈ stored_energy(cache, m[inertia_2_id])
    @test E_mass_1 + PE₁ ≈ stored_energy(cache, m[mass_1_id])
    @test E_mass_2 + PE₂ ≈ stored_energy(cache, m[mass_2_id])
    @test stored_energy(cache) ≈ E
    nothing
end

function scara_tests()
    @testset "Scara Tests" scara_test() 
end