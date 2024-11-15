using LinearAlgebra
using Random
using VMRobotControl

using VMRobotControl: AxisAngle, rotation_matrix, skew # For scara_tests
using VMRobotControl: transform, linear_vel, angular_vel, origin

using StaticArrays
using Test


function pendulum_test(a₁, l₁, m₁, I₁, t, q, q̇, u, gravity)    
    m = begin 
        mechanism = Mechanism{Float64}("mechanism")
        add_frame!(mechanism, "L1")
        add_frame!(mechanism, "L1_com")
        add_joint!(mechanism, Revolute(a₁); parent=root_frame(mechanism), child="L1", id="J1")
        add_joint!(mechanism, Rigid(Transform(l₁)); parent="L1", child="L1_com", id="J2")

        
        add_coordinate!(mechanism, FrameAngularVelocity("L1"); id="L1_ω")
        add_coordinate!(mechanism, FrameOrigin("L1"); id="L1_pos")
        add_coordinate!(mechanism, FrameOrigin("L1_com"); id="L1_com")
    
        inertia_1 = Inertia(I₁, "L1_ω")
        mass_1 = PointMass(m₁, "L1_com")
    
        add_component!(mechanism, inertia_1; id="inertia_1")
        add_component!(mechanism, mass_1; id="mass_1")
        compile(mechanism)
    end

    mass_1_id = get_compiled_componentID(m, "mass_1")
    inertia_1_id = get_compiled_componentID(m, "inertia_1")
    
    cache = new_dynamics_cache(m)
    
    L1 = get_compiled_frameID(m, "L1")
    L1_com = get_compiled_frameID(m, "L1_com")
    
    # Test setup    
    q̈_computed = copy(dynamics!(cache, t, q, q̇, gravity, u)) # Computes the velocity kinematics and jacobians
    u_computed = copy(get_generalized_force(cache))
    M_computed = copy(get_inertance_matrix(cache))
    
    # Transform by hand
    tf₁ = Transform(AxisAngle(a₁, q[1]))
    tf₁_com = tf₁ * Transform(l₁)
    
    @test tf₁ ≈ get_transform(cache, L1)
    @test tf₁_com ≈ get_transform(cache, L1_com)
    
    # Velocities by hand
    a₁ʷ = a₁
    l₁ʷ = rotor(tf₁) * l₁
    
    v₁ = SVector(0., 0., 0.)
    ω₁ = q̇[1] * a₁ʷ
    ṽ₁ = vcat(ω₁, v₁)
    
    v₁_com = v₁ + cross(ω₁, l₁ʷ)
    ω₁_com = ω₁
    ṽ₁_com = vcat(ω₁_com, v₁_com)
    
    @test v₁ ≈ get_linear_vel(cache, L1)
    @test v₁_com ≈ get_linear_vel(cache, L1_com)
    @test ω₁ ≈ get_angular_vel(cache, L1)
    @test ω₁_com ≈ get_angular_vel(cache, L1_com)
    

    # Velocity product acceleration by hand
    ȧ₁ʷ = SVector(0., 0., 0.)    
    l̇₁ʷ = cross(ω₁, l₁ʷ)
    
    v̇₁ = SVector(0., 0., 0.)
    ẇ₁ = q̇[1] * ȧ₁ʷ
    dṽ₁ = vcat(ẇ₁, v̇₁)
    
    l̈₁ʷ = cross(ẇ₁, l₁ʷ) + cross(ω₁, l̇₁ʷ)
    v̇₁_com = v̇₁ + cross(ẇ₁, l₁ʷ) + cross(ω₁, l̇₁ʷ)
    
    @test v̇₁_com ≈ l̈₁ʷ

    ẇ₁_com = ẇ₁
    dṽ₁_com = vcat(ẇ₁_com, v̇₁_com)
        
    c₁ = SVector(0., 0., 0., 0., 0., 0.)
    
    @test c₁ ≈ dṽ₁
    


    @test l̇₁ʷ ≈ v₁_com - v₁
    @test v̇₁ ≈ get_linear_vpa(cache, L1)
    @test v̇₁_com ≈ get_linear_vpa(cache, L1_com)
    @test ẇ₁ ≈ get_angular_vpa(cache, L1)

    # Opspace forces by hand
    I₁ʷ, İ₁ʷ = let 
        R = rotation_matrix(rotor(tf₁))
        Ṙ = skew(ω₁) * R
        I = R * I₁ * R'
        İ = Ṙ * I₁ * R' + R * I₁ * Ṙ'
        I, İ    
    end
        
    F₁ = m₁ * (v̇₁_com - gravity)
    τ₁ = (I₁ʷ * ẇ₁ + İ₁ʷ * ω₁)
    
    @test F₁ ≈ opspace_force(cache, m[mass_1_id])
    @test τ₁ ≈ opspace_force(cache, m[inertia_1_id])
    
    # Generalized forces by hand
    u_mass_1 = [
        -dot(a₁ʷ, cross(l₁ʷ, F₁) ),
    ]
    u_inertia_1 = [
        -dot(a₁ʷ, τ₁),
    ]
    
    @test u_mass_1 ≈ generalized_force!(zeros(1), cache, m[mass_1_id]) atol=1e-8
    @test u_inertia_1 ≈ generalized_force!(zeros(1), cache, m[inertia_1_id]) atol=1e-10

    u_components = u_mass_1 + u_inertia_1
    @test u_components ≈ u_computed atol=1e-8
    
    # Inertance matrix by hand
    M₁₁_mass_1 = norm(cross(l₁ʷ, a₁ʷ))^2 * m₁
    M₁₁_inertia_1 = dot(a₁ʷ, I₁ʷ, a₁ʷ)
        
    @test M₁₁_mass_1 ≈ inertance_matrix!(zeros(1, 1), cache, m[mass_1_id])[1, 1]
    @test M₁₁_inertia_1 ≈ inertance_matrix!(zeros(1, 1), cache, m[inertia_1_id])[1, 1]
    @test M_computed[1, 1] ≈ M₁₁_mass_1+ M₁₁_inertia_1
    

    # Stored energy by hand
    E_mass_1 = 0.5 * m₁ * dot(v₁_com, v₁_com)
    E_inertia_1 = 0.5 * dot(ω₁, I₁ʷ, ω₁)
    PE₁ = m₁ * dot(-gravity, origin(tf₁_com))
    E = E_mass_1 + E_inertia_1 + PE₁
    
    @test E_inertia_1 ≈ stored_energy(cache, m[inertia_1_id])
    @test E_mass_1 + PE₁ ≈ stored_energy(cache, m[mass_1_id])
    @test stored_energy(cache) ≈ E atol=1e-8
    
    inv(M_computed) * u_components
    nothing
end



   
function pendulum_tests()
    @testset "pendulum_tests" begin
        let
            X, Y, Z = SVector(1., 0., 0.), SVector(0., 1., 0.), SVector(0., 0., 1.)
            a₁ = Y # Axis
            l₁ = Z # Length
            m₁ = 1.0 # Mass
            I₁ = @SMatrix [1.0 0.0 0.0; # Inertia
                        0.0 2.0 0.0;
                        0.0 0.0 3.0]
            t = 0. # Time
            q = SVector(π/4) # Joint angle
            q̇ = SVector(1) # Joint velocity
            u = SVector(0) # Joint actuation torque
            gravity = SVector(0., 0., 10.)
            
            pendulum_test(a₁, l₁, m₁, I₁, t, q, q̇, u, gravity)
        end    

        Random.seed!(0)
        for i = 1:5
            a₁ = normalize(rand(SVector{3, Float64}))
            l₁ = rand(SVector{3, Float64})
            m₁ = rand()
            I₁ = let
                A = rand(SMatrix{3, 3, Float64})
                A' * A
            end
            t = rand()
            q = SVector(rand())
            q̇ = SVector(rand())
            u = SVector(rand())
            gravity = rand(SVector{3, Float64})
            pendulum_test(a₁, l₁, m₁, I₁, t, q, q̇, u, gravity)
        end
    end
end