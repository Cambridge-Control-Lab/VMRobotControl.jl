module Transforms

export Rotor, rotor_to_svector
export AxisAngle, XRotor, YRotor, ZRotor
export bivector, scalar, rotation_matrix, rotation_matrix_derivative
export rotate
export EulerVector
export angular_velocity, angular_velocity_prematrix, angular_velocity_prematrix_derivative
export quaternion_derivative, quaternion_derivative_propagation, quaternion_derivative_propagation_derivative
export AxisAngleDerivative
export quatmul_geodual_bivector_matrix
export skew

export Transform, Twist, SpatialAcceleration
export TransformJacobian, TransformJacobianAngular
export TransformHessian
export Twist
export SpatialAcceleration
export RigidBodyState

export origin, rotor
export linear_vel, angular_vel
export linear_acc, angular_acc
export linear, angular
export linear_jacobian_smatrix, angular_jacobian_smatrix
export transform, twist, acceleration, spatialacceleration

# import Base: zero, *, +, -, ^, inv
using LinearAlgebra: cross, normalize

using ..Hessians: hessian_quadratic_form
using LinearAlgebra: norm, I, cross, diagm, det
using Random: Random, AbstractRNG, rand
using StaticArrays
using StaticArrays: SVector


include("./rotors.jl")
include("./transforms_impl.jl")
include("./twists.jl")
include("./spatial_acceleration.jl")
include("./rigid_body_state.jl")
include("./transform_utils.jl")


#############################
# Define eltype for all types

let types = [:Rotor, :Transform, :Twist, :SpatialAcceleration, :RigidBodyState]
    # Define shortcut functions to call without first evaluating precomp e.g.
    # opspaceConfiguration(m, q, q̇, t, c::OperationSpace)
    for type in types
        @eval begin
            Base.eltype(::$type{T}) where T = T
            Base.eltype(::Type{$type{T}}) where T = T
        end
    end
end

end