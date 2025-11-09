using DiffResults
using ForwardDiff
using ProgressMeter
using Random: MersenneTwister

using VMRobotControl
using VMRobotControl: joint_transform, joint_twist, joint_vpa, jacobian_column, AbstractJointData
using VMRobotControl: Twist, SpatialAcceleration
using VMRobotControl.Hessians: my_hessian
using VMRobotControl.Transforms: angular_velocity, quatmul_geodual_bivector_matrix

using StaticArrays
using Test

function test_single_joint(JD::Type{<:AbstractJointData{T}}) where T
    rng = MersenneTwister(1234)
    jointdata = rand(rng, JD)
    parent_tf = rand(Transform{T})
    q = rand(rng, SVector{config_size(jointdata), T})
    q̇ = rand(rng, SVector{velocity_size(jointdata), T})    
    _test_single_joint(jointdata, parent_tf, q, q̇)
end

function _test_single_joint(jointdata::AbstractJointData{T}, parent_tf, q, q̇) where T
    # Test with a stationary parent frame
    parent_twist = zero(Twist{T})
    parent_vpa = zero(SpatialAcceleration{T})    
    t = 0.0 # Time is irrelevant as kinematics are not a func of time

    _ = joint_transform(jointdata, parent_tf, t, q)
    dr = my_hessian(q -> SVector(joint_transform(jointdata, parent_tf, t, q)), SVector(q))

    tf = DiffResults.value(dr)[1]
    J = DiffResults.derivative(dr)[1]
    H = DiffResults.hessian(dr)[1]

    twist_AD = Twist(tf, J, q̇)
    @test isnan(twist_AD) == false
    vpa_AD = SpatialAcceleration(tf, J, H, SVector(q̇))
    @test isnan(vpa_AD) == false

    @test tf ≈ joint_transform(jointdata, parent_tf, t, q)
    @test isnan(tf) == false

    twist_kinematic = joint_twist(jointdata, parent_tf, tf, parent_twist, t, q, q̇)
    @test isnan(twist_kinematic) == false
    @test twist_AD ≈ twist_kinematic
    
    vpa_kinematic = joint_vpa(jointdata, parent_tf, tf, parent_twist, twist_AD, parent_vpa, t, q, q̇, )
    @test isnan(vpa_kinematic) == false
    @test vpa_AD ≈ vpa_kinematic atol=1e-8 rtol=1e-8

    Jo_AD, Jr_AD = origin(J), quatmul_geodual_bivector_matrix(rotor(tf))*rotor(J)
    Jo_Ki, Jr_Ki = jacobian_column(jointdata, parent_tf, tf, t, q)
    Jo_AD2, Jr_AD2 = VMRobotControl.autodiff_jacobian_column(jointdata, parent_tf, tf, tf, t, q)

    @test Jr_AD[:] ≈ Jr_Ki[:]
    @test Jr_AD[:] ≈ Jr_AD2[:]
    @test Jr_Ki[:] ≈ Jr_AD2[:]
    @test isapprox(Jo_AD[:], Jo_Ki[:]; atol=1e-8)
    @test isapprox(Jo_AD[:], Jo_AD2[:]; atol=1e-8)
    @test isapprox(Jo_Ki[:], Jo_AD2[:]; atol=1e-8)

    if isa(jointdata, RevoluteData{Float64})
        axis = jointdata.axis
        axis_w = rotor(parent_tf) * rotor(jointdata.transform) * axis
        ω = axis_w .* q̇
        @test ω ≈ angular_vel(twist_AD)
        @test ω ≈ angular_vel(twist_kinematic) 

        r = rotor(tf)
        Jr = rotor(J)        
        S = VMRobotControl.Transforms.angular_velocity_prematrix(r)
        @test all(S*Jr .≈ axis_w)
    end
end


function test_double_joint(::Type{JD1}, ::Type{JD2}) where {T, JD1 <: AbstractJointData{T}, JD2 <: AbstractJointData{T}}
    rng = MersenneTwister(1234)
    scalar_type = T
    jointdata1 = rand(rng, JD1)
    jointdata2 = rand(rng, JD2)
    
    q = rand(rng, SVector{2, scalar_type})
    q̇ = rand(rng, SVector{2, scalar_type})
    t = 0.0 # Time is irrelevant to these tests as no `TimeFuncJoint`s

    q1, q2 = SVector(q[1]), SVector(q[2])
    q̇1, q̇2 = SVector(q̇[1]), SVector(q̇[2])

    tf0 = zero(Transform{T})
    twist0 = zero(Twist{T})
    vpa0 = zero(SpatialAcceleration{T})

    tf1 = joint_transform(jointdata1, tf0, t, q1)
    twist1 = joint_twist(jointdata1, tf0, tf1, twist0, t, q1, q̇1)
    vpa1 = joint_vpa(jointdata1, tf0, tf1, twist0, twist1, vpa0, t, q1, q̇1)
    
    tf2 = joint_transform(jointdata2, tf1, t, q2)
    twist2 = joint_twist(jointdata2, tf1, tf2, twist1, t, q2, q̇2)
    vpa2 = joint_vpa(jointdata2, tf1, tf2, twist1, twist2, vpa1, t, q2, q̇2)
    
    dr = my_hessian(q) do q
        _tf1 = joint_transform(jointdata1, zero(Transform{T}), t, SVector(q[1]))
        _tf2 = joint_transform(jointdata2, _tf1, t, SVector(q[2]))
        SVector(_tf2)
    end

    tf_auto = DiffResults.value(dr)[1]
    J_auto = DiffResults.derivative(dr)[1]
    H_auto = DiffResults.hessian(dr)[1]
    twist_auto = Twist(tf_auto, J_auto, q̇)
    vpa_auto = SpatialAcceleration(tf_auto, J_auto, H_auto, q̇)

    @test tf_auto ≈ tf2 atol=1e-8 rtol=1e-8
    @test twist_auto ≈ twist2 atol=1e-8 rtol=1e-8
    @test vpa_auto ≈ vpa2 atol=1e-8 rtol=1e-8

    Jv2, Jw2 = jacobian_column(jointdata2, tf1, tf2, t, q2)
    # Jo3, Jr3 = VMRobotControl.autodiff_jacobian_column(jointdata2, tf1, tf2, q[joint2.idx])

    Jv_auto = origin(J_auto)[:, 2]
    Jw_auto = quatmul_geodual_bivector_matrix(rotor(tf_auto)) * rotor(J_auto)[:, 2]
    @test  Jv_auto ≈ Jv2[:] atol=1e-8 rtol=1e-8
    @test  Jw_auto ≈ Jw2[:] atol=1e-8 rtol=1e-8
end

function test_single_joint_mechanism(::Type{JD}) where {T, JD<:AbstractJointData{T}}
    rng = MersenneTwister(1234)
    jointdata = rand(rng, JD)

    mech = Mechanism{T}("$(nameof(JD))_test_mechanism")
    child_frame = add_frame!(mech, "child_frame")
    add_joint!(mech, jointdata; parent=root_frame(mech), child=child_frame, id="J1")
    m = compile(mech)
    fid = get_compiled_frameID(m, child_frame)

    q = rand(rng, T, config_size(jointdata))
    q̇ = rand(rng, T, velocity_size(jointdata))
    t = rand(rng, T)
    gravity = SVector(0.0, 0.0, 0.0)
    # Zero gravity, so that we can check that the velocity product acceleration 
    # is zero (gravity is implemented as a base frame acceleration)

    cache = new_dynamics_cache(m)
    precompute!(cache, t, q, q̇, gravity) # Computes velocity kinematics and Jacobians
    
    Jo = get_linear_jacobian(cache, fid)
    Jw = get_angular_jacobian(cache, fid)
    v = get_linear_vel(cache, fid)
    w = get_angular_vel(cache, fid)
    
    @test Jo*q̇ ≈ v
    @test Jw*q̇ ≈ w    
    
    v̇ = get_linear_vpa(cache, fid)
    ẇ = get_angular_vpa(cache, fid)

    # Velocity product acceleration is the acceleration when q̈=0. Therefore
    # we can check that it is true by finding d/dt (v) using AD.
    v̇ẇ_AD = ForwardDiff.derivative(t) do t_AD
        q_AD = q + q̇ * (t_AD - t)
        @assert q_AD ≈ q
        AD_cache = new_rbstates_cache(m, eltype(t_AD))
        velocity_kinematics!(AD_cache, t_AD, q_AD, q̇)
        vcat(get_linear_vel(AD_cache, fid), get_angular_vel(AD_cache, fid))
    end

    v̇_AD, ẇ_AD = v̇ẇ_AD[SVector(1, 2, 3)], v̇ẇ_AD[SVector(4, 5, 6)]

    @test v̇ ≈ v̇_AD
    @test ẇ ≈ ẇ_AD
 
    nothing
end

function test_double_joint_mechanism(::Type{JD1}, ::Type{JD2}) where {T, JD1<:AbstractJointData{T}, JD2<:AbstractJointData{T}}
    rng = MersenneTwister(1234)
    jointdata1 = rand(rng, JD1)
    jointdata2 = rand(rng, JD2)

    mech = Mechanism{T}("$(nameof(JD1))_ $(nameof(JD2))_test_mechanism")
    L1_frame = add_frame!(mech, "L1_frame")
    L2_frame = add_frame!(mech, "L2_frame")
    add_joint!(mech, jointdata1; parent=root_frame(mech), child=L1_frame, id="J1")
    add_joint!(mech, jointdata2; parent=L1_frame, child=L2_frame, id="J2")
    m = compile(mech)
    L1 = get_compiled_frameID(m, L1_frame)
    L2 = get_compiled_frameID(m, L2_frame)

    q = rand(rng, SVector{config_size(mech), T})
    q̇ = rand(rng, SVector{velocity_size(mech), T})
    t = rand(T)
    gravity = SVector(0.0, 0.0, 0.0)

    # Zero gravity, so that we can check that the velocity product acceleration
    # is zero (gravity is implemented as a base frame acceleration)

    cache_1 = new_dynamics_cache(m)
    precompute!(cache_1, t, q, q̇, gravity)
    
    for frame in (L1, L2)
        Jv = get_linear_jacobian(cache_1, frame)
        Jw = get_angular_jacobian(cache_1, frame)
        v = get_linear_vel(cache_1, frame)
        w = get_angular_vel(cache_1, frame)
    
        @test Jv*q̇ ≈ v
        @test Jw*q̇ ≈ w    

        v̇ = get_linear_vpa(cache_1, frame)
        ẇ = get_angular_vpa(cache_1, frame)

        # Velocity product acceleration is the acceleration when q̈=0. Therefore
        # we can check that it is true by finding d/dt (v) using AD.
        v̇ẇ_AD = let
            ForwardDiff.derivative(t) do t_AD
                q_AD = q + q̇ * (t_AD - t)
                @assert q_AD ≈ q
                AD_cache = new_rbstates_cache(m, eltype(t_AD))
                velocity_kinematics!(AD_cache, t_AD, q_AD, q̇)
                vcat(get_linear_vel(AD_cache, frame), get_angular_vel(AD_cache, frame))
            end
        end

        v̇_AD, ẇ_AD = v̇ẇ_AD[SVector(1, 2, 3)], v̇ẇ_AD[SVector(4, 5, 6)]

        @test v̇ ≈ v̇_AD
        @test ẇ ≈ ẇ_AD
    end
    
    nothing
end

function single_joint_tests(mobile_jointtypes)
    @testset "Single Joint Tests" begin
        @showprogress dt=1.0 desc="Single Joint Tests" for jtype in mobile_jointtypes
            @testset "$jtype" begin
                test_single_joint(jtype)
            end
        end
    end
end

function single_joint_mechanism_tests(all_jointtypes)
    @testset "Single Joint Mechanism Tests" begin
        @showprogress dt=1.0 desc="Single Joint Mechanism Tests" for jointtype in all_jointtypes
            @testset "$(nameof(jointtype))" begin
                test_single_joint_mechanism(jointtype)
            end
        end
    end
end


# Test derivatives for a mechanism formed of two joints chained together
# This should capture errors in VPA, which can't be seen by a single joint, 
# as the base velocity is zero.
function double_joint_tests(mobile_jointtypes)
    @testset "Double Joint Tests"  begin
        @showprogress dt=1.0 desc="Double Joint Tests" for jtype1 in mobile_jointtypes
            for jtype2 in mobile_jointtypes
                @testset "$jtype1, $jtype2" begin
                    test_double_joint(jtype1, jtype2)
                end
            end
        end
    end
end

function double_joint_mechanism_tests(all_jointtypes)
    @testset "Double joint mechanism tests" begin
        @showprogress dt=1.0 desc="Double Joint Mechanism Tests" for JD1 in all_jointtypes
            for JD2 in all_jointtypes
                @testset "$(nameof(JD1)), $(nameof(JD2))" begin
                    test_double_joint_mechanism(JD1, JD2)
                end
            end
        end
    end
end

mobile_jointtypes = [
    RevoluteData{Float64},
    PrismaticData{Float64},
    HelicalData{Float64},
    RailData{Float64, VMRobotControl.CubicSpline{3, Float64}},
    SphericalData{Float64}
]
all_jointtypes = [
    RevoluteData{Float64},
    PrismaticData{Float64},
    RailData{Float64, VMRobotControl.CubicSpline{3, Float64}},
    Rigid{Float64}
]

# single_joint_tests(mobile_jointtypes);
# single_joint_mechanism_tests(all_jointtypes);
# double_joint_tests(mobile_jointtypes);
# double_joint_mechanism_tests(all_jointtypes);
