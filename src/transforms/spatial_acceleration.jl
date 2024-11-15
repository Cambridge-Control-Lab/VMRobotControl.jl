"""
    SpatialAcceleration{T}

A spatial acceleration represents the acceleration of a rigid body/frame in 3D space.

It is composed of a linear acceleration and an angular acceleration.
"""
struct SpatialAcceleration{T}
    linear::SVector{3, T}
    angular::SVector{3, T}
end

SpatialAcceleration{T2}(a::SpatialAcceleration{T1}) where {T1, T2} = SpatialAcceleration(SVector{3, T2}(a.linear), SVector{3, T2}(a.angular))

linear_acc(vpa::SpatialAcceleration) = vpa.linear
angular_acc(vpa::SpatialAcceleration) = vpa.angular
Base.zero(::Type{SpatialAcceleration{T}}) where T = SpatialAcceleration{T}(zero(SVector{3, T}), zero(SVector{3, T}))
Base.zero(::SpatialAcceleration{T}) where T = SpatialAcceleration{T}(zero(SVector{3, T}), zero(SVector{3, T}))


function Base.:+(a1::SpatialAcceleration, a2::SpatialAcceleration) 
    SpatialAcceleration(linear_acc(a1) + linear_acc(a2), angular_acc(a1) + angular_acc(a2))
end

function Base.:-(a1::SpatialAcceleration, a2::SpatialAcceleration) 
    SpatialAcceleration(linear_acc(a1) - linear_acc(a2), angular_acc(a1) - angular_acc(a2))
end

# From a jacobian and hessian
function SpatialAcceleration(tf::Transform, J::TransformJacobian, H::TransformHessian, q̇)
    q̇_svec = SVector{ndof(J)}(q̇)
    vpa_linear = hessian_quadratic_form(origin(H), q̇_svec)
    S = angular_velocity_prematrix(rotor(tf))
    ṙ = rotor(J)*q̇_svec
    Ṡ = angular_velocity_prematrix_derivative(ṙ)
    r̈ = hessian_quadratic_form(rotor(H), q̇_svec)
    vpa_angular = Ṡ*ṙ + S*r̈
    SpatialAcceleration(vpa_linear, vpa_angular)
end

function Base.isapprox(a::SpatialAcceleration{T}, b::SpatialAcceleration{T}; kwargs...) where T
    isapprox(linear_acc(a), linear_acc(b); kwargs...) && isapprox(angular_acc(a), angular_acc(b); kwargs...)
end

function Base.isnan(a::SpatialAcceleration)
    return any(isnan, linear_acc(a)) || any(isnan, angular_acc(a))
end

Base.:*(r::Rotor, w::SpatialAcceleration) = SpatialAcceleration(r*linear_acc(w), r*angular_acc(w))
