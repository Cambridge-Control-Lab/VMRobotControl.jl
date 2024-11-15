@inline function quatmultiply(u, v)
    u₁, u₂, u₃, u₄ = u
    v₁, v₂, v₃, v₄ = v
    SVector{4, Base.promote_eltype(u, v)}(
        @fastmath (
            u₁*v₁ - u₂*v₂ - u₃*v₃ - u₄*v₄,
            u₁*v₂ + u₂*v₁ + u₃*v₄ - u₄*v₃,
            u₁*v₃ - u₂*v₄ + u₃*v₁ + u₄*v₂,
            u₁*v₄ + u₂*v₃ - u₃*v₂ + u₄*v₁
        )
    )
end

function quatmul_matrix(u::SVector{4})
    "Returns matrix equivalent U such that quatmultiply(u, v) = U*v"
    @SMatrix [u[1]  -u[2]  -u[3]  -u[4];
              u[2]   u[1]  -u[4]   u[3];
              u[3]   u[4]   u[1]  -u[2];
              u[4]  -u[3]   u[2]   u[1] ]
end

function quatmul_matrix_reverse(u::SVector{4})
    "Returns matrix equivalent U such that quatmultiply(u, v) = U*v
    
    we do w = quatmultiply(u, v)

    v is constant

    we want dw/du

    dw/du = 
    "
    @SMatrix [u[1]   u[2]   u[3]   u[4];
             -u[2]   u[1]   u[4]  -u[3];
             -u[3]  -u[4]   u[1]   u[2];
             -u[4]   u[3]  -u[2]   u[1] ]
end

#######################
# Rotor
#######################

struct Rotor{T}
    # Consists of a scalar and bivector part of a 3-Space in geometric algebra
    # Represents a rotation in 3D space. 
    # When applied to another rotor, performs a single quaternion multiplication e.g. q₂ q₁
    # When applied to a point, multiplies on each side, which performs the rotation e.g. q p q⁻¹
    scalar::T
    bivector::SVector{3, T}
    function Rotor{T}(scalar::T, bivector::SVector{3, T}, normalize=Val(true)) where T
        s̄, v̄ = scalar, bivector
        if isa(normalize, Val{true})
            svec = SVector{4, T}(scalar, bivector...)
            magnitude_squared = svec'*svec
            if isapprox(magnitude_squared, one(magnitude_squared))
                # Do nothing
            else
                magnitude = sqrt(magnitude_squared) # Sqrt is expensive, only do if needed
                recip = one(T)/magnitude
                s̄, v̄ = scalar*recip, bivector.*recip
            end
        else
            @assert isa(normalize, Val{false})
        end
        new{T}(s̄, v̄)
    end
end

# Accessors
bivector(r::Rotor) = r.bivector
scalar(r::Rotor) = r.scalar

#######################################
# Constructors
#######################################

# Infer type of Rotor from scalar and bivector
function Rotor(scalar::Real, bivector::SVector{3}, normalize::Val=Val(true))
    T1, T2 = typeof(scalar), eltype(bivector)
    Rotor{promote_type(T1, T2)}(scalar, bivector, normalize)
end
# Accept arguments in either order
Rotor(bivector::SVector{3}, scalar::Real, normalize::Val=Val(true)) = Rotor(scalar, bivector, normalize)

# Convert to different type (does nothing if already correct type)
function Rotor{T2}(r::Rotor{T1}, normalize::Val=Val(true)) where {T1, T2}
    _scalar = convert(T2, r.scalar)
    _bivector = convert(SVector{3, T2}, r.bivector)    
    Rotor{T2}(_scalar, _bivector, normalize)
end

# Construct from an SVector
function Rotor(v::SVector{4, T}, normalize::Val=Val(true)) where T 
    Rotor(v[1], v[SVector{3, Int}(2, 3, 4)], normalize) # Normalizes by default
end

# Zero rotations
Base.zero(::Type{Rotor{T}}) where T = Rotor(one(T), zero(SVector{3, T}))
Base.zero(::Type{Rotor}) = zero(Rotor{Float64})

# AxisAngle constructors for a Rotor. Rotates around axis by angle, following right hand rule.
function AxisAngle(axis, angle, normalize::Val=Val(false))
    s, c = sincos(angle/2)
    Rotor(c, s * axis, normalize)
end

function AxisAngle(rotor::Rotor)
    η, ϵ = scalar(rotor), bivector(rotor)
    s, c = norm(ϵ), η
    c > 1.0 && isapprox(c, 1.0) && (c = 1.0) # Handle numerical errors
    angle = 2 * asin(s)
    @assert isapprox(2 * acos(c), angle; atol=1e-3)  "$(2*acos(c)), $(angle)"
    axis = ϵ / s
    axis, angle
end

# Random rotation made from AxisAngle
function Random.rand(rng::AbstractRNG, ::Random.SamplerType{Rotor{T}}) where T
    angle = T(2π) * rand(rng, T)
    yaw = T(2π) * rand(rng, T)
    pitch = T(π) * rand(rng, T) - T(π/2)
    x, y, z = cos(yaw)*cos(pitch), sin(yaw)*cos(pitch), sin(pitch)
    axis = SVector{3, T}(x, y, z)
    AxisAngle(axis, angle, Val(true))
end

# Shortut for rotors about unit vectors
XRotor(a) = AxisAngle(SVector(1.0, 0.0, 0.0), a)
YRotor(a) = AxisAngle(SVector(0.0, 1.0, 0.0), a)
ZRotor(a) = AxisAngle(SVector(0.0, 0.0, 1.0), a)

# From rotation matrix
function Rotor(a::StaticMatrix{3, 3, T}; validate_input::Val{B}=Val{true}()) where {T, B}
    # Credit to:
    # https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
    # Which provided equivalent C++ code
    # TODO check a is a rotation matrix
    if validate_input == Val{true}()
        @assert isapprox(det(a), one(T)) "Not a rotation matrix, determinant is not 1: $a, det(a) = $(det(a))"
        Z, O = zero(T), one(T)
        @assert isapprox(a'*a, SMatrix{3, 3, T, 9}(O, Z, Z, Z, O, Z, Z, Z, O)) "Not a rotation matrix, not orthogonal: $a, a'*a = $(a'*a)"
    elseif validate_input == Val{false}()
    else
        error("Invalid validate_input value")
    end
    trace = a[1, 1] + a[2, 2] + a[3, 3]
    if trace > 0
        s = 0.5 / sqrt(trace+ 1.0);
        w = 0.25 / s;
        x = ( a[3, 2] - a[2, 3] ) * s;
        y = ( a[1, 3] - a[3, 1] ) * s;
        z = ( a[2, 1] - a[1, 2] ) * s;
    else 
        if (a[1, 1] > a[2, 2]) && (a[1, 1] > a[3, 3])
            s = 2.0 * sqrt( 1.0 + a[1, 1] - a[2, 2] - a[3, 3]);
            w = (a[3, 2] - a[2, 3] ) / s;
            x = 0.25 * s;
            y = (a[1, 2] + a[2, 1] ) / s;
            z = (a[1, 3] + a[3, 1] ) / s;
        elseif (a[2, 2] > a[3, 3]) 
            s = 2.0 * sqrt( 1.0 + a[2, 2] - a[1, 1] - a[3, 3]);
            w = (a[1, 3] - a[3, 1] ) / s;
            x = (a[1, 2] + a[2, 1] ) / s;
            y = 0.25 * s;
            z = (a[2, 3] + a[3, 2] ) / s;
        else 
            s = 2.0 * sqrt( 1.0 + a[3, 3] - a[1, 1] - a[2, 2] );
            w = (a[2, 1] - a[1, 2] ) / s;
            x = (a[1, 3] + a[3, 1] ) / s;
            y = (a[2, 3] + a[3, 2] ) / s;
            z = 0.25 * s;
        end
    end
    Rotor(T(w), SVector{3, T}(x, y, z))
end


#######################################
# Conversions
#######################################

# Conversion to/from SVector
# TODO bump version due to changing interface here.
StaticArrays.SVector(r::Rotor) = error("SVector(::Rotor) API deprecated, please use `rotor_to_svector` instead.")
# rotor_to_svector(r::Rotor) = vcat(scalar(r), bivector(r))
rotor_to_svector(r::Rotor{T}) where T = SVector{4, T}(scalar(r), bivector(r)...)
quatmul_matrix(r::Rotor) = quatmul_matrix(rotor_to_svector(r))

const X = @SVector [1.0, 0.0, 0.0]
const Y = @SVector [0.0, 1.0, 0.0]
const Z = @SVector [0.0, 0.0, 1.0]
rotation_matrix(rotor) = hcat(rotor*X, rotor*Y, rotor*Z)

#######################################
# Operators
#######################################

@inline
function rotate(r::Rotor{T1}, p::SVector{3, T2}) where {T1, T2}
    """rotates p by r to give z"""
    zero_T2::T2 = zero(T2)
    p̂ = SVector{4, T2}(zero_T2, p[1], p[2], p[3])
    v⁻¹ = rotor_to_svector(inv(r))
    v = rotor_to_svector(r)
    ẑ = quatmultiply(quatmultiply(v, p̂), v⁻¹)
    z = ẑ[SVector{3, Int}(2, 3, 4)] 
    z
end

@inline 
function rotate(R1::Rotor, R2::Rotor, normalize::VAL) where VAL<:Union{Val{true}, Val{false}}
    " R1 * R2 returns the Rotor equivalent to applying R2 then R1"
    v1 = vcat(scalar(R1), bivector(R1))
    v2 = vcat(scalar(R2), bivector(R2))
    y = quatmultiply(v1, v2)
    Rotor(y[1], y[SVector{3, Int}(2, 3, 4)], normalize)
end

@inline rotate(R1::Rotor, R2::Rotor) = rotate(R1, R2, Val{true}())

@inline Base.inv(r::Rotor{T}) where T = Rotor{T}(scalar(r), -bivector(r), Val{false}())
# Overload multiply to mean "apply transformation"
@inline Base.:*(r::Rotor{T1}, p::SVector{3, T2}) where {T1, T2} = rotate(r, p)
@inline Base.:*(R1::Rotor, R2::Rotor) = rotate(R1, R2)
Base.:^(R::Rotor, p::Integer) = Base.power_by_squaring(R,p)



#######################################
# Angular velocity from rotor and its derivative

# TODO Fix type strangeness here... we are using a rotor to represent a rotor derivative (hence not normalizing)
function angular_velocity(r::Rotor, ṙ::SVector)
    ṙ_rotor = Rotor(ṙ, Val(false)) # don't normalize
    ω = -2 * bivector( rotate(r, ṙ_rotor, Val(false))  ) # don't normalize after rotating
    ω
end
# angular_velocity(r::Rotor, ṙ::SVector) = -2*(quatmultiply(rotor_to_svector(r),  ṙ)[SVector{3, Int}(2, 3, 4)])

"""
    angular_velocity_prematrix(r::Rotor)

Return S, which satisfies S*ṙ = ω, where omega is the angular velocity of the 
frame.
"""
function angular_velocity_prematrix(r::Rotor)
    # 
    u = rotor_to_svector(r)
    -2.0 * @SMatrix [u[2]  -u[1]   u[4]  -u[3];
                     u[3]  -u[4]  -u[1]   u[2];
                     u[4]   u[3]  -u[2]  -u[1]]
end

"""
    angular_velocity_prematrix_derivative(ṙ::SVector)

Return Ṡ, time derivative of `angular_velocity_prematrix(r)`
"""
function angular_velocity_prematrix_derivative(ṙ::SVector{4})
    # TODO Fix type strangeness here... we are using a rotor to represent a rotor derivative (hence not normalizing)
    angular_velocity_prematrix(Rotor(ṙ, Val(false)))  # Is linear so derivative is easy
end

function quaternion_derivative(rotor::Rotor, ω::SVector{3})
    s = scalar(rotor)
    B = bivector(rotor)
    dscalar = -1//2 * (ω[1]*B[1] + ω[2]*B[2] + ω[3]*B[3])
    dBx = 1//2 * (ω[1]*s + ω[2]*B[3] - ω[3]*B[2])
    dBy = 1//2 * (ω[2]*s + ω[3]*B[1] - ω[1]*B[3])
    dBz = 1//2 * (ω[3]*s + ω[1]*B[2] - ω[2]*B[1])
    Rotor(dscalar, SVector(dBx, dBy, dBz), Val{false}()) # Don't normalize, is a velocity
end

"""
    quaternion_derivative_propagation(r::Rotor)

Returns the matrix E which satisfies E*ω = ṙ, where ω is the angular velocity of rotor r
"""
function quaternion_derivative_propagation(r::Rotor)
    η, ϵ = scalar(r), bivector(r)
    1//2 * @SMatrix [
        -ϵ[1]   -ϵ[2]   -ϵ[3];
         η       ϵ[3]   -ϵ[2];
        -ϵ[3]    η       ϵ[1];
         ϵ[2]   -ϵ[1]    η
    ]
end

"""
    quaternion_derivative_propagation_derivative(r::Rotor, ω)

Returns the matrix Ė, time derivative of [`quaternion_derivative_propagation(r)`](@ref)
"""
function quaternion_derivative_propagation_derivative(r::Rotor, ω::SVector{3})
    ṙ = quaternion_derivative(r, ω)
    ṅ, ė = scalar(ṙ), bivector(ṙ)
    quaternion_derivative_propagation(Rotor(SVector(ṅ, ė...), Val(false)))
end

#######################################
# Derivatives
#######################################

"Partial derivative of rotor w.r.t. angle"
function AxisAngleDerivative(axis, angle::T) where T
    b =   0.5 * cos(angle/2) * axis
    w = - 0.5 * sin(angle/2)
    rotor_to_svector(Rotor(b, w))::SVector{4, T}
end

function rotation_matrix_derivative(r::Rotor, ṙ::SVector{4})
    ω = angular_velocity(r, ṙ)
    R = rotation_matrix(r)
    rotation_matrix_derivative(R, ω)
end

function rotation_matrix_derivative(r::Rotor, ω::SVector{3})
    R = rotation_matrix(r)
    rotation_matrix_derivative(R, ω)
end

function rotation_matrix_derivative(R::SMatrix{3, 3}, ω::SVector{3})
    Sω = @SMatrix [ 0.0  -ω[3]  ω[2];
                    ω[3]  0.0  -ω[1];
                   -ω[2]  ω[1]  0.0 ]
    Ṙ = Sω * R
    return Ṙ
end


# Conversion from EulerVector
"""
    EulerVector(v::SVector{3})

Represents a rotation on the plane with normal vector v, by angle norm(v).
"""
struct EulerVector{T}
    v::SVector{3, T}
end

function Rotor(ev::EulerVector)
    mag = norm(ev.v)
    axis, angle = ev.v/mag,  mag
    AxisAngle(axis, angle, Val(false))
end

struct EulerVectorDerivative{T}
    v̇::SVector{3, T}
end

function Rotor(ev::EulerVector, evd::EulerVectorDerivative)
    v, v̇ = ev.v, evd.v̇
    m = norm(v)
    ṁ = 2 * v .* v̇
    a = v/m
    ȧ = (v̇*m - v *ṁ)/m^2
    
    # TODO 
end

###

function Base.isapprox(a::Rotor{T}, b::Rotor{T}; kwargs...) where T
    # We determine rotors to be equal if they rotate a vector equally
    # A quick test is to try check that the effect on the basis vectors in the 
    # same. This should be OK
    X, Y, Z = SVector(1.0, 0.0, 0.0), SVector(0.0, 1.0, 0.0), SVector(0.0, 0.0, 1.0)
    check(v) = isapprox(a*v, b*v; kwargs...)
    check(X) && check(Y) && check(Z)
end

function Base.isnan(r::Rotor)
    return isnan(scalar(r)) || any(isnan, bivector(r))
end

############################################################
