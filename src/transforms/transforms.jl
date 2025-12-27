module Transforms

# Import core types and functions from RigidBodyTransforms
using RigidBodyTransforms
using RigidBodyTransforms:
    Rotor, Transform,
    XRotor, YRotor, ZRotor,
    scalar, bivector, origin, rotor,
    to_rotation_matrix, to_axis_angle

# Re-export core RigidBodyTransforms types and functions
export Rotor, Transform
export XRotor, YRotor, ZRotor
export scalar, bivector, origin, rotor
export to_rotation_matrix, to_axis_angle

# Export VMRobotControl-specific extensions
export rotor_to_svector
export rotation_matrix_derivative
export EulerVector, EulerVectorDerivative
export AxisAngleDerivative
export angular_velocity, angular_velocity_prematrix, angular_velocity_prematrix_derivative
export quaternion_derivative, quaternion_derivative_propagation, quaternion_derivative_propagation_derivative
export quatmul_geodual_bivector_matrix
export skew

# Export VMRobotControl-specific types
export Twist, SpatialAcceleration
export TransformJacobian, TransformJacobianAngular
export TransformHessian
export RigidBodyState

# Export accessor functions
export linear_vel, angular_vel
export linear_acc, angular_acc
export linear, angular
export linear_jacobian_smatrix, angular_jacobian_smatrix
export transform, twist, acceleration, spatialacceleration

# Imports from other VMRobotControl modules
using ..Hessians: hessian_quadratic_form
using LinearAlgebra: norm, I, cross, diagm, det
using Random: Random, AbstractRNG, rand
using StaticArrays
using StaticArrays: SVector

# Include VMRobotControl-specific implementation files
include("./rotors_extensions.jl")
include("./transforms_impl.jl")
include("./twists.jl")
include("./spatial_acceleration.jl")
include("./rigid_body_state.jl")
include("./transform_utils.jl")

#############################
# Define eltype for VMRobotControl-specific types only
# (Rotor and Transform eltype already defined in RigidBodyTransforms)

let types = [:Twist, :SpatialAcceleration, :RigidBodyState]
    for type in types
        @eval begin
            Base.eltype(::$type{T}) where T = T
            Base.eltype(::Type{$type{T}}) where T = T
        end
    end
end

end
