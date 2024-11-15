#####################
# Rigid body transform
#####################
"""
    Transform{T}(origin::SVector{3, T}, rotor::Rotor{T})

A rigid body transform defined by an origin and a rotor (quaternion). 

Can be constructed from either an origin and a rotor, or just an origin, or just a rotor.
"""
struct Transform{T}
    origin::SVector{3, T}
    rotor::Rotor{T}
end
# Constructors
Transform(R::Rotor{T}) where T = Transform{T}(zero(SVector{3, T}), R)
Transform(o::SVector{3, T}) where T = Transform{T}(o, zero(Rotor{T}))
Transform{T}(tf::Transform{T}) where T = tf
Transform{T2}(tf::Transform{T1}) where {T1, T2} = Transform{T2}(SVector{3, T2}(tf.origin), Rotor{T2}(tf.rotor))


Base.:zero(::Type{Transform{T}}) where T = Transform{T}(zero(SVector{3, T}), zero(Rotor{T}))
Base.:zero(::Transform{T}) where T = Transform{T}(zero(SVector{3, T}), zero(Rotor{T}))
Random.rand(rng::AbstractRNG, ::Random.SamplerType{Transform{T}}) where T = Transform(rand(rng, SVector{3, T}), rand(rng, Rotor{T}))

# Accessors
origin(t::Transform) = t.origin
rotor(t::Transform) = t.rotor

function transform(T::Transform, p::SVector{3})
    origin(T) + rotor(T)*p
end

function transform(tf1::Transform{T1}, tf2::Transform{T2}) where {T1, T2}
    T_out = promote_type(T1, T2)
    o1, o2 = origin(tf1), origin(tf2)
    R1, R2 = rotor(tf1), rotor(tf2)
    offset::SVector{3, T_out} = R1 * o2
    o = o1 + offset
    r = R1*R2
    Transform(o, r)
end

@inline Base.:*(T::Transform, p::SVector{3}) = transform(T, p)
@inline Base.:*(tf1::Transform{T1}, tf2::Transform{T2}) where {T1, T2} = transform(tf1, tf2)

function Base.inv(tf::Transform{T}) where T
    r⁻¹, o = inv(rotor(tf)), origin(tf)
    Transform{T}(r⁻¹*(-o), r⁻¹)
end

function Base.isapprox(a::Transform{T}, b::Transform{T}; kwargs...) where T
    isapprox(rotor(a), rotor(b); kwargs...) && isapprox(origin(a), origin(b); kwargs...)
end

function Base.isnan(tf::Transform)
    return any(isnan, origin(tf)) || isnan(rotor(tf))
end

#####################
# Derivatives
#####################

struct TransformJacobian{T, N, L1, L2}
    J_origin::SMatrix{3, N, T, L1}
    J_rotor::SMatrix{4, N, T, L2}
end

ndof(::Type{TransformJacobian{T, N, L1, L2}}) where {T, N, L1, L2} = N
ndof(J::TransformJacobian) = ndof(typeof(J))

# struct TransformJacobianAngular{T, N, L}
#     J_linear::MMatrix{3, N, T, L}
#     J_angular::MMatrix{3, N, T, L}
# end

struct TransformJacobianAngular{T, N, L}
    J_linear::Matrix{T}
    J_angular::Matrix{T}
    function TransformJacobianAngular{T, N, L}() where {T,N,L}
        @assert L == 3*N
        J_linear = Matrix(undef, 3, N)
        J_angular = Matrix(undef, 3, N)
        new{T, N, L}(J_linear, J_angular)
    end
end

function TransformJacobianAngular{T, N}() where {T, N}
    L = 3 * N
    J_linear = Matrix(undef, 3, N)
    J_angular = Matrix(undef, 3, N)
    new{T, N, L}(J_linear, J_angular)
end

ndof(::Type{TransformJacobianAngular{T, N, L}}) where {T, N, L} = N
ndof(J::TransformJacobianAngular) = ndof(typeof(J))



# Base.eltype(::Type{TransformJacobian{T, N, L1, L2}}) where {T, N, L1, L2} = T
# Base.eltype(::Type{TransformJacobianAngular{T, N, L}}) where {T, N, L} = T
# Base.eltype(::TJ) where TJ<: TransformJacobian = eltype(TJ) 
# Base.eltype(::TJA) where TJA<: TransformJacobianAngular = eltype(TJA)

origin(t::TransformJacobianAngular) = throw(ErrorException("Don't use origin, use 'linear'"))
rotor(t::TransformJacobianAngular) = throw(ErrorException("Don't use rotor, use 'angular'"))
# rotor(t::TransformJacobianAngular) = t.J_angular

linear(J::TransformJacobianAngular) = J.J_linear
angular(J::TransformJacobianAngular) = J.J_angular

linear_jacobian_smatrix(J::TransformJacobianAngular{T, N, L}) where {T, N, L} = SMatrix{3, N, T, L}(J.J_linear)
angular_jacobian_smatrix(J::TransformJacobianAngular{T, N, L}) where {T, N, L} = SMatrix{3, N, T, L}(J.J_angular)

# function Base.zero(::Type{TransformJacobianAngular{T, N, L}}) where {T, N, L}
#     TransformJacobianAngular(zero(MMatrix{3, N, T, L}), zero(MMatrix{3, N, T, L}))
# end

function Base.zero(::Type{TransformJacobianAngular{T, N, L}}) where {T, N, L}
    ret = TransformJacobianAngular{T, N, L}()
    fill!(ret.J_linear, zero(T))
    fill!(ret.J_angular, zero(T))
    ret
end


struct TransformHessian{T, N, L1, L2}
    H_origin::SArray{Tuple{3, N, N}, T, 3, L1}
    H_rotor::SArray{Tuple{4, N, N}, T, 3, L2}
end

Base.eltype(::Type{TransformHessian{T, N, L1, L2}}) where {T, N, L1, L2} = T
Base.eltype(tfh::TransformHessian) = eltype(typeof(tfh))
origin(t::TransformJacobian) = t.J_origin
rotor(t::TransformJacobian) = t.J_rotor
origin(t::TransformHessian) = t.H_origin
rotor(t::TransformHessian) = t.H_rotor
