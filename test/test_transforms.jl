using VMRobotControl.Transforms
using StaticArrays 
using Random: MersenneTwister
using Test
using LinearAlgebra: dot, normalize, norm
using AllocCheck

begin
function test_transforms()
    rng = MersenneTwister(1234)
    # Test correctness of translations
    p1 = rand(rng, SVector{3, Float64})
    translation = rand(rng, SVector{3, Float64})
    tf1 = Transform(translation)
    @test tf1 * p1 ≈ p1 + translation

    # Test rotations
    axis = normalize(rand(rng, SVector{3, Float64}))
    angle = rand(rng, Float64)
    r1 = AxisAngle(axis, angle)
    tf2 = Transform(r1)
    @test tf2 * p1 ≈ r1 * p1
    @test dot(tf2 * p1, axis) ≈ dot(p1, axis)
    p_perp = p1 - dot(p1, axis) * axis
    tfp_perp = (tf2 * p1) - dot(tf2 * p1, axis) * axis
    @test dot(normalize(p_perp), normalize(tfp_perp)) ≈ cos(angle)

    # Test homogeneous transforms
    tf3 = Transform(translation, r1)
    # Rotate then translate
    @test tf3 * p1 ≈ tf1 * tf2 * p1
    @test tf3 * tf2 ≈ Transform(translation, r1*r1)

    # Test inverse transforms
    @test inv(tf1) * tf1 ≈ zero(typeof(tf1)) atol = 1e-15
    @test tf1 * inv(tf1) ≈ zero(typeof(tf1)) atol = 1e-15

    @test inv(tf2) * tf2 ≈ zero(typeof(tf2)) atol = 1e-15
    @test tf2 * inv(tf2) ≈ zero(typeof(tf2)) atol = 1e-15

    @test inv(tf3) * tf3 ≈ zero(typeof(tf3)) atol = 1e-15
    @test tf3 * inv(tf3) ≈ zero(typeof(tf3)) atol = 1e-15

    # Test no allocations
    @test 0 == @allocations tf2 * tf3
    @test 0 == @allocations tf3 * p1
    @test 0 == @allocations AxisAngle(axis, angle) * p1
end
test_transforms()
end
