"""
    Twist{T}(linear::SVector{3, T}, angular::SVector{3, T})

A twist represents the velocity of a rigid body/frame in 3D space. 
It is composed of a linear velocity and an angular velocity.
"""
struct Twist{T}
    linear::SVector{3, T}
    angular::SVector{3, T}
end

Twist{T2}(w::Twist{T1}) where {T1, T2} = Twist(SVector{3, T2}(w.linear), SVector{3, T2}(w.angular))

linear_vel(w::Twist) = w.linear
angular_vel(w::Twist) = w.angular
Base.zero(::Type{Twist{T}}) where T = Twist{T}(zero(SVector{3, T}), zero(SVector{3, T}))
Base.zero(::Twist{T}) where T = Twist{T}(zero(SVector{3, T}), zero(SVector{3, T}))

@inline Base.:+(w1::Twist, w2::Twist) = Twist(linear_vel(w1) + linear_vel(w2), angular_vel(w1) + angular_vel(w2))
@inline Base.:-(w1::Twist, w2::Twist) = Twist(linear_vel(w1) - linear_vel(w2), angular_vel(w1) - angular_vel(w2))

function Twist(tf::Transform, J::TransformJacobian, q̇)
    q̇_svec = SVector{ndof(J)}(q̇)
    linear_velocity = origin(J)*q̇_svec
    r = rotor(tf)

    # ṙ = SVector(rotor(J)*q̇)
    # angular_velocity = angular_velocity(r, ṙ)

    Jω = angular_velocity_prematrix(r) * rotor(J)
    angular_velocity = Jω*q̇_svec
    Twist(linear_velocity,  angular_velocity)
end

function Base.isapprox(a::Twist{T}, b::Twist{T}; kwargs...) where T
    isapprox(linear_vel(a), linear_vel(b); kwargs...) && isapprox(angular_vel(a), angular_vel(b); kwargs...)
end

function Base.isnan(w::Twist)
    return any(isnan, linear_vel(w)) || any(isnan, angular_vel(w))
end

Base.:*(r::Rotor, w::Twist) = Twist(r*linear_vel(w), r*angular_vel(w))
