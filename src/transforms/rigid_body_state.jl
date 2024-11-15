struct RigidBodyState{T}
    transform::Transform{T}
    twist::Twist{T}
    acceleration::SpatialAcceleration{T}
end

transform(b::RigidBodyState) = b.transform
twist(b::RigidBodyState) = b.twist
acceleration(b::RigidBodyState) = b.acceleration
origin(b::RigidBodyState) = origin(transform(b))
rotor(b::RigidBodyState) = rotor(transform(b))
linear_vel(b::RigidBodyState) = linear_vel(twist(b))
angular_vel(b::RigidBodyState) = angular_vel(twist(b))
linear_acc(b::RigidBodyState) = linear_acc(acceleration(b))
angular_acc(b::RigidBodyState) = angular_acc(acceleration(b))

function Base.zero(::Type{RigidBodyState{T}}) where T 
    RigidBodyState{T}(zero(Transform{T}), zero(Twist{T}), zero(SpatialAcceleration{T}))
end

function Base.isapprox(a::RigidBodyState{T}, b::RigidBodyState{T}; kwargs...) where T
    isapprox(a.transform, b.transform; kwargs...) & isapprox(a.twist, b.twist; kwargs...) & isapprox(a.acceleration, b.acceleration; kwargs...)
end
