# VMRobotControl-specific extensions for Rotor and Transform types
# Core types come from RigidBodyTransforms.jl

using StaticArrays: SVector, SMatrix, @SMatrix
using LinearAlgebra: norm
import RigidBodyTransforms
import RigidBodyTransforms: Rotor, scalar, bivector, to_rotation_matrix, angular_velocity_prematrix

#######################################
# Utility Functions
#######################################

"""
    rotor_to_svector(r::Rotor{T}) where T

Convert a Rotor to SVector{4,T} with [scalar, bivector...] ordering.
"""
rotor_to_svector(r::Rotor{T}) where T = SVector{4, T}(scalar(r), bivector(r)...)

"""
    angular_velocity(r::Rotor, ṙ::SVector{4})

Compute angular velocity from rotor and its time derivative.
"""
function angular_velocity(r::Rotor, ṙ::SVector{4})
    S = angular_velocity_prematrix(r)
    ω = S * ṙ
    return ω
end

#######################################
# Derivatives
#######################################

"""
    AxisAngleDerivative(axis, angle::T) where T

Partial derivative of rotor w.r.t. angle, evaluated at (axis, angle).
Returns the derivative as an SVector{4}.
"""
function AxisAngleDerivative(axis, angle::T) where T
    b =   0.5 * cos(angle/2) * axis
    w = - 0.5 * sin(angle/2)
    rotor_to_svector(Rotor(b, w))::SVector{4, T}
end

"""
    rotation_matrix_derivative(r::Rotor, ṙ::SVector{4})

Time derivative of rotation matrix given rotor and its time derivative.
"""
function rotation_matrix_derivative(r::Rotor, ṙ::SVector{4})
    ω = angular_velocity(r, ṙ)
    R = to_rotation_matrix(r)
    rotation_matrix_derivative(R, ω)
end

"""
    rotation_matrix_derivative(r::Rotor, ω::SVector{3})

Time derivative of rotation matrix given rotor and angular velocity.
"""
function rotation_matrix_derivative(r::Rotor, ω::SVector{3})
    R = to_rotation_matrix(r)
    rotation_matrix_derivative(R, ω)
end

"""
    rotation_matrix_derivative(R::SMatrix{3, 3}, ω::SVector{3})

Time derivative of rotation matrix given angular velocity.
Returns Ṙ = [ω]× * R where [ω]× is the skew-symmetric matrix of ω.
"""
function rotation_matrix_derivative(R::SMatrix{3, 3}, ω::SVector{3})
    Sω = @SMatrix [ 0.0  -ω[3]  ω[2];
                    ω[3]  0.0  -ω[1];
                   -ω[2]  ω[1]  0.0 ]
    Ṙ = Sω * R
    return Ṙ
end

#######################################
# Euler Vector Representation
#######################################

"""
    EulerVector(v::SVector{3})

Represents a rotation on the plane with normal vector v, by angle norm(v).
This is an alternative rotation representation where the axis is v/norm(v)
and the angle is norm(v).
"""
struct EulerVector{T}
    v::SVector{3, T}
end

"""
    Rotor(ev::EulerVector)

Convert an EulerVector to a Rotor.
"""
function Rotor(ev::EulerVector)
    mag = norm(ev.v)
    axis, angle = ev.v/mag, mag
    Rotor(; axis=axis, angle=angle)  # Use RBT's keyword constructor
end

"""
    EulerVectorDerivative(v̇::SVector{3})

Time derivative of an EulerVector.
"""
struct EulerVectorDerivative{T}
    v̇::SVector{3, T}
end

"""
    Rotor(ev::EulerVector, evd::EulerVectorDerivative)

Convert EulerVector and its derivative to Rotor representation.
Note: This function is incomplete (TODO in original code).
"""
function Rotor(ev::EulerVector, evd::EulerVectorDerivative)
    v, v̇ = ev.v, evd.v̇
    m = norm(v)
    ṁ = 2 * v .* v̇
    a = v/m
    ȧ = (v̇*m - v *ṁ)/m^2

    # TODO: Complete implementation
    error("EulerVectorDerivative to Rotor conversion not yet implemented")
end
