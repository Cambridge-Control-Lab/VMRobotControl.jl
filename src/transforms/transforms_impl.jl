# VMRobotControl-specific Transform Jacobian and Hessian types
# Core Transform and Rotor types come from RigidBodyTransforms.jl

import RigidBodyTransforms: Transform, Rotor, origin, rotor

#####################
# Transform Jacobians and Hessians
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
