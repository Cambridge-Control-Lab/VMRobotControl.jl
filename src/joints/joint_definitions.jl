"""
AbstractJointData{T}

An instance of AbstractJointData represents the data needed to compute the 
transform between two unspecified frames of reference. When added to a mechanism
the frames are specified.

T is the scalar type of the joint (typically Float64), and can be extracted with
valuetype(::AbstractJoint).

A joint can be added to a mechanism using the [`add_joint!`](@ref) function, which 
requires the joint, the parent frame, and the child frame, creating a 
[`MechanismJoint`](@ref)`.

# See also 
Joint data types: [`Rigid`](@ref), [`Revolute`](@ref), [`Prismatic`](@ref), [`Rail`](@ref)
['Helical'](@ref)

# Developer Notes
Each joint type must implement the following methods:
- `_joint_relative_transform` (See [`joint_relative_transform`](@ref))
- `_joint_relative_twist` (See [`joint_relative_twist`](@ref))
- `_joint_relative_vpa` (See [`joint_relative_vpa`](@ref))
- `_joint_relative_acceleration` (See [`joint_relative_acceleration`](@ref))
- `_jacobian_columns` (See [`jacobian_column`](@ref))

"""
abstract type AbstractJointData{T} end
valuetype(::Type{J}) where {T, J <: AbstractJointData{T}} = T
valuetype(::J) where {T, J <: AbstractJointData{T}} = T
jointtype(j::AbstractJointData) = typeof(j) # Pretty display fallback

#############################
# Zero Degrees-of-Freedom joints
#############################

"""
    Rigid(::Transform)

A rigid joint, defined simply by a transformation from the child frame to the 
parent frame.

See also [`JointData`](@ref)
"""
struct Rigid{T} <: AbstractJointData{T}
    transform::Transform{T}
end

function Random.rand(rng::AbstractRNG, ::Random.SamplerType{Rigid{T}}) where T
    Rigid(rand(rng, Transform{T}))
end


"""
    ReferenceJoint(::Transform)

A ReferenceJoint does not exhibit dynamics, but can it's transform can be
mutated, so that it may act as a motion source. The transform is stored as A
RefValue, which you can set/getcan like: `joint.transform[]`.

Be careful when changing the transform---if you are doing many simulations you
may need to deepcopy to avoid aliasing.

See also [`JointData`](@ref)
"""
struct ReferenceJoint{T} <: AbstractJointData{T}
    transform::Base.RefValue{Transform{T}}
end
ReferenceJoint(tf::Transform{T}) where T = ReferenceJoint{T}(Ref(tf))

"""
    TimeFuncJoint(f_tf::Function, f_twist::Function, f_vpa::Function)

A TimeFuncJoint represents a joint which has a transform which is a function
of time: `f_tf(t)`. The derivatives `f_twist` and `f_vpa` must be provided, and
checked, by the user.

See also [`JointData`](@ref)
"""
struct TimeFuncJoint{T, F1, F2, F3} <: AbstractJointData{T}
    f_tf::F1
    f_twist::F2
    f_vpa::F3
end
TimeFuncJoint(f_tf, f_twist, f_vpa) = TimeFuncJoint{Float64, typeof(f_tf), typeof(f_twist), typeof(f_vpa)}(f_tf, f_twist, f_vpa)

#############################
# Single Degree-of-Freedom joints
#############################

"""
    Revolute(axis::SVector{3})
    Revolute(axis::SVector{3}, tf::Transform)

Represents a revolute/rotating joint between two frames. The transform from the
child frame to the parent frame is `tf * Transform(Rotor(; axis=axis, angle=q))`.

`axis` must be normalized: `norm(axis)≈1.0`.

See also [`JointData`](@ref)
"""
function Revolute end

struct RevoluteData{T} <: AbstractJointData{T}
    axis::SVector{3, T}
    transform::Transform{T}
    function RevoluteData(a::SVector{3, T}, tf::Transform{T}) where T # TODO Breaking - add {T} here
        @assert norm(a) ≈ one(T) "Rotary joint axis '$a' must be normalized. `norm(a)=`$norm(a)."
        new{T}(a, tf)
    end
end

Revolute(axis::SVector{3, T}, tf::Transform{T}) where T = RevoluteData(axis, tf)
Revolute(axis::SVector{3, T}) where T = RevoluteData(axis, zero(Transform{T}))

function Random.rand(rng::AbstractRNG, ::Random.SamplerType{RevoluteData{T}}) where T
    dir = rand(rng, SVector{3, T})
    a = dir / norm(dir)
    RevoluteData(a, rand(rng, Transform{T}))
end

"""
    Prismatic(axis::SVector{3})
    Prismatic(axis::SVector{3}, tf::Transform)

Represents a prismatic/sliding joint between two frames. The transform from the
child frame to the parent frame is `tf * Transform(q*axis).` The axis does not
have to be normalized.

See also [`JointData`](@ref)
"""
function Prismatic end

struct PrismaticData{T} <: AbstractJointData{T}
    axis::SVector{3, T}
    transform::Transform{T}
end

Prismatic(axis::SVector{3, T}, tf::Transform{T}) where T = PrismaticData(axis, tf)
Prismatic(axis::SVector{3, T}) where T = PrismaticData(axis, zero(Transform{T}))

function Random.rand(rng::AbstractRNG, ::Random.SamplerType{PrismaticData{T}}) where T
    PrismaticData(rand(rng, SVector{3, T}), rand(rng, Transform{T}))
end

"""
    Helical(axis::SVector{3})
    Helical(axis::SVector{3}, tf::Transform)

Represents a helical/screw joint between two frames. The transform from the
child frame to the parent frame is `tf * Transform(q*axis).` The axis must
be normalized.

The screw motion is is defined by a translation `q*lead*axis`, and a
rotation by q around `axis`, as the lead of the screw is the distance 
travelled per full rotation.

To change from a counter-clockwise to a clockwise helix, flip the direction
of the axis, and use a negative lead.

As a result the "pitch" of the screw is |axis|, the lengthwise
distance travelled for one rotation.

See also [`JointData`](@ref)
"""
function Helical end

struct HelicalData{T} <: AbstractJointData{T}
    axis::SVector{3, T}
    lead::T
    transform::Transform{T}
    function HelicalData{T}(axis::SVector{3, T}, lead::T, transform::Transform{T}) where T
        @assert norm(axis) ≈ one(T) "Helical joint axis '$a' must be normalized. `norm(a)=`$norm(a)."
        new{T}(axis, lead, transform)
    end
end

Helical(axis::SVector{3, T}, lead::T, tf::Transform{T}) where T = HelicalData{T}(axis, lead, tf)
Helical(axis::SVector{3, T}, lead::T) where T = HelicalData{T}(axis, zero(Transform{T}))

function Random.rand(rng::AbstractRNG, ::Random.SamplerType{HelicalData{T}}) where T
    Helical(normalize(rand(rng, SVector{3, T})), rand(rng, T), rand(rng, Transform{T}))
end

"""
    Rail(spline, tf::Transform)
    Rail(spline)

Represents a prismatic/sliding joint where the joint slides along a bezier 
spline. The child frame is not rotated relative to the parent frame, (unless tf
causes a rotation). 

The origin of the parent frame for joint variable `q` is 
`tf * spline_position(spline, q)`.

See also [`Prismatic`](@ref), [`Revolute`](@ref)
"""
function Rail end

const SplineUnion{T} = Union{CubicSpline{3, T}, LoopingCubicSpline{3, T}}
struct RailData{T, S<:SplineUnion{T}} <: AbstractJointData{T}
    spline::S
    scaling::T
    transform::Transform{T}
end

function Rail(spline::SplineUnion{T}, transform::Transform{T}) where T
    max_t = Splines.n_knots(spline) - 1
    @assert max_t > 0
    scaling = Splines.integrate_length(spline) / max_t # Scale to roughly meters
    RailData{T, typeof(spline)}(spline, scaling, transform)
end
Rail(spline::SplineUnion{T}) where T = Rail(spline, zero(Transform{T}))

function Random.rand(rng::AbstractRNG, ::Random.SamplerType{RailData{T, S}}) where {S, T}
    Rail(rand(rng, S), rand(rng, Transform{T}))
end

"""
    Spherical()
    Spherical(tf::Transform)

Represents a spherical joint between two frames. 

See also [`Revolute`](@ref)
"""
function Spherical end

struct SphericalData{T} <: AbstractJointData{T}
    transform::Transform{T}
end

Spherical(::Type{T}) where T = SphericalData{T}(zero(Transform{T}))

function Random.rand(rng::AbstractRNG, ::Random.SamplerType{SphericalData{T}}) where T
    SphericalData(rand(rng, Transform{T}))
end

################################################################################

config_size(j::AbstractJointData) = config_size(typeof(j))
velocity_size(j::AbstractJointData) = velocity_size(typeof(j))

config_size(::Type{<:Rigid}) = 0
velocity_size(::Type{<:Rigid}) = 0

config_size(::Type{<:ReferenceJoint}) = 0
velocity_size(::Type{<:ReferenceJoint}) = 0

config_size(::Type{<:TimeFuncJoint}) = 0
velocity_size(::Type{<:TimeFuncJoint}) = 0

config_size(::Type{<:RevoluteData}) = 1
velocity_size(::Type{<:RevoluteData}) = 1

config_size(::Type{<:HelicalData}) = 1
velocity_size(::Type{<:HelicalData}) = 1

config_size(::Type{<:PrismaticData}) = 1
velocity_size(::Type{<:PrismaticData}) = 1

config_size(::Type{<:RailData}) = 1
velocity_size(::Type{<:RailData}) = 1

config_size(::Type{<:SphericalData}) = 3
velocity_size(::Type{<:SphericalData}) = 3

jointtype(::Rigid) = "Rigid"
jointtype(::ReferenceJoint) = "Reference"
jointtype(::TimeFuncJoint) = "TimeFunc"
jointtype(::RevoluteData) = "Revolute"
jointtype(::PrismaticData) = "Prismatic"
jointtype(::HelicalData) = "Helical"
jointtype(::RailData) = "Rail"
jointtype(::SphericalData) = "Spherical"

#####################
# Transforms
#####################

function joint_transform(joint, tf_parent, t, q)
    tfʲ = joint_relative_transform(joint, t, q)
    child_frame_transform(tf_parent, tfʲ)
end

@inline child_frame_transform(tf_parent, tf_relative) = tf_parent * tf_relative

"""
    joint_relative_transform(joint, t, q)

Returns the transform from the child frame to the parent frame of a joint configuration 
and time.

The transform is represented as a [`Transform`](@ref) object

`q` is the joint configuration *for the individual joint* (i.e. not for the whole mechanism)
as an SVector of the appropriate size.

This function is used to ensure correct type signature for all joints and arguments, while 
implementation for each [`AbstractJointData`](@ref) is done in a method of 
`joint_relative_transform`.
"""
function joint_relative_transform(joint::AbstractJointData, t::Real, qⱼ::SVector)
    T = promote_type(valuetype(joint), eltype(qⱼ), typeof(t))
    tf = _joint_relative_transform(joint, t, qⱼ)
    Transform{T}(tf)
end
@inline _joint_relative_transform(joint::Rigid,             t, qⱼ::SVector{0}) = joint.transform
@inline _joint_relative_transform(joint::ReferenceJoint,    t, qⱼ::SVector{0}) = joint.transform[]
@inline _joint_relative_transform(joint::TimeFuncJoint,     t, qⱼ::SVector{0}) = joint.f_tf(t)
@inline _joint_relative_transform(joint::RevoluteData,      t, qⱼ::SVector{1}) = joint.transform * Transform(Rotor(; axis=joint.axis, angle=qⱼ[1], normalize=Val(false)))
@inline _joint_relative_transform(joint::PrismaticData,     t, qⱼ::SVector{1}) = joint.transform * Transform(qⱼ[1] .* joint.axis)
@inline _joint_relative_transform(joint::HelicalData,       t, qⱼ::SVector{1}) = joint.transform * (Rotor(; axis=joint.axis, angle=qⱼ[1]) * Transform((qⱼ[1] * joint.lead)  .* joint.axis))
@inline _joint_relative_transform(joint::RailData,          t, qⱼ::SVector{1}) = joint.transform * Transform(spline_position(qⱼ[1] / joint.scaling, joint.spline))
@inline function _joint_relative_transform(joint::SphericalData, t, q::SVector{3})
    Q = norm(q)
    sinc = Q ≈ zero(Q) ? one(Q) : sin(Q) / Q
    rotor = Rotor(cos(Q), sinc * q)
    return joint.transform * rotor
end

######################
# Twist
######################

"""
    child_frame_twist(tf_parent, tf_child, twist_parent, joint_relative_twist)

Compute the twist of the child frame, from the twist of the parent frame, the
transforms (from each frame to the world frame) and the joint relative twist: 
i.e. the twist of the child frame represented in the parent frame.
"""
function child_frame_twist(tfᵖ, tfᶜ, wᵖ, joint_relative_twist)
    rigid_joint_twist(tfᵖ, tfᶜ, wᵖ) +  rotor(tfᵖ) * joint_relative_twist
end

"""
    rigid_joint_twist(tf_parent, tf_child, twist_parent)

Compute the twist of the child frame from the transforms and the twist of the 
parent frame. Assumes that the frames are rigidly connected.
"""
function rigid_joint_twist(tfₚ::Transform, tf::Transform, wₚ::Twist)
    δo = origin(tf) - origin(tfₚ)
    δv = cross(angular_vel(wₚ), δo)
    wₚ + Twist(δv, zero(δv))
end

function joint_twist(joint, tfᵖ, tfᶜ, wᵖ, q, q̇, t)
    jrt = joint_relative_twist(joint, q, q̇, t)
    child_frame_twist(tfᵖ, tfᶜ, wᵖ, jrt)
end


"""
    joint_relative_twist(joint, t, q, q̇)

Returns the twist of the child frame of a joint represented in the parent frame.

The twist is represented as a [`Twist`](@ref) object. 

`q` is the joint configuration *for the individual joint* (i.e. not for the whole mechanism)
as an SVector of the appropriate size.

`q̇` is the joint velocity *for the individual joint* (i.e. not for the whole mechanism)
as an SVector of the appropriate size.

"""
function joint_relative_twist(joint::AbstractJointData, t::Real, q::SVector, q̇::SVector)
    T = promote_type(valuetype(joint), eltype(q), eltype(q̇), typeof(t))
    w = _joint_relative_twist(joint, t, q, q̇)
    Twist{T}(w)
end

@inline _joint_relative_twist(j::Rigid, t, q, q̇) = zero(Twist{eltype(q)})
@inline _joint_relative_twist(j::ReferenceJoint, t, q, q̇) = zero(Twist{eltype(q)})
@inline _joint_relative_twist(j::TimeFuncJoint, t, q, q̇) = j.f_twist(t)

@inline 
function _joint_relative_twist(joint::RevoluteData, t, q, q̇)
    axis = rotor(joint.transform) * joint.axis # Axis in parent frame
    δω = axis * q̇[1]
    Twist(zero(δω), δω)
end
@inline
function _joint_relative_twist(joint::PrismaticData, t, q, q̇)
    axis = rotor(joint.transform) * joint.axis
    δv = axis * q̇[1]
    Twist(δv, zero(δv))
end
@inline
function _joint_relative_twist(joint::HelicalData, t, q, q̇)
    axis = rotor(joint.transform) * joint.axis
    δv = axis * joint.lead * q̇[1] # Translational
    δω = axis * q̇[1] # Rotational
    Twist(δv, δω)
end
@inline
function _joint_relative_twist(joint::RailData, t, q, q̇)
    tangent = rotor(joint.transform) * spline_derivative(q[1]/joint.scaling, joint.spline)/joint.scaling
    δv = tangent * q̇[1]
    Twist(δv, zero(δv))
end

@inline
function _joint_relative_twist(joint::SphericalData, t, q::SVector{3}, q̇::SVector{3})
    Q = norm(q)
    sinc = Q ≈ zero(Q) ? one(Q) : sin(Q) / Q
    rot = Rotor(cos(Q), sinc * q)

    Q̇ = (q' * q̇) / Q
    sinċ = Q ≈ zero(Q) ? zero(Q) : Q̇ * (cos(Q) * Q - sin(Q)) / (Q^2)
    drot_dt = Rotor(-sin(Q) * Q̇, q * sinċ + sinc * q̇, Val{false}())

    δω = rotor(joint.transform) * (angular_velocity_prematrix(rot) * rotor_to_svector(drot_dt))
    # δω = rotor(joint.transform) * angular_velocity(rot, rotor_to_svector(drot_dt))
    Twist(zero(δω), δω)
end

######################
# Velocity Product Acceleration
######################

function joint_vpa end

function joint_vpa(joint, tfᵖ::Transform, tfᶜ::Transform, wᵖ::Twist, wᶜ::Twist, vpaᵖ, t, q, q̇)
    jrv = joint_relative_vpa(joint, t, q, q̇)
    child_frame_acceleration(tfᵖ, tfᶜ, wᵖ, wᶜ, vpaᵖ, jrv)
end

function child_frame_acceleration(tfᵖ::Transform, tfᶜ::Transform, wᵖ::Twist, wᶜ::Twist, ẇᵖ, joint_relative_acceleration)
    vpaᵣ = rigid_joint_vpa(tfᵖ, tfᶜ, wᵖ, wᶜ, ẇᵖ)
    vpaⱼ = rotor(tfᵖ) * joint_relative_acceleration
    vpaᵣ + vpaⱼ
end

function rigid_joint_vpa(tfᵖ::Transform, tfᶜ::Transform, wᵖ::Twist, wᶜ::Twist, vpaᵖ)
    δo = origin(tfᶜ) - origin(tfᵖ)
    δv = linear_vel(wᶜ) - linear_vel(wᵖ)
    δω = angular_vel(wᶜ) - angular_vel(wᵖ)
    
    ωᵖ = angular_vel(wᵖ)
    dωᵖ = angular_acc(vpaᵖ)

    δv̇ = cross(dωᵖ, δo) + 2.0*cross(ωᵖ, δv) - cross(ωᵖ, cross(ωᵖ, δo))
    δdω = cross(ωᵖ, δω)
    vpaᵖ + SpatialAcceleration(δv̇, δdω)
end

"""
    joint_relative_vpa(joint, t, q, q̇)

Returns the velocity-product-acceleration of the child frame of a joint
represented in the parent frame.

The function `joint_vpa` applies the correction due to velocity of the
parent frame.

The acceleration is represented as a [`SpatialAcceleration`](@ref) object

Most joint types have no velocity-product-acceleration, and return zero. 

# What is velocity-product-acceleration?
Velocity product acceleration is the acceleration of the child frame relative to
the parent frame due to the velocity of the joint variable, q̇, assuming 
q̈=0, and that the parent frame is stationary.
As an example, consider the linear velocity as ȯ = J(q)q̇, then the derivative with time is 
ö = J(q)q̈ + J̇(q)q̇. Here J(q) is the jacobian of the joint. Thus J̇(q) ̇q is the 
velocity product acceleration.
"""
function joint_relative_vpa(joint, t, q, q̇,)
    T = promote_type(valuetype(joint), eltype(q), eltype(q̇), typeof(t))
    ẇ = _joint_relative_vpa(joint, t, q, q̇)
    SpatialAcceleration{T}(ẇ)
end
@inline _joint_relative_vpa(j::Union{Rigid, ReferenceJoint}, t, q, q̇) = zero(SpatialAcceleration{eltype(q)})
@inline _joint_relative_vpa(j::TimeFuncJoint, q, q̇, t) = j.f_vpa(t)
@inline _joint_relative_vpa(j::Union{RevoluteData, PrismaticData, HelicalData}, t, q, q̇) = zero(SpatialAcceleration{eltype(q)})
@inline function _joint_relative_vpa(joint::RailData, t, q, q̇)
    dtangent = rotor(joint.transform) * spline_second_derivative(q[1]/joint.scaling, joint.spline) / (joint.scaling^2) 
    δv̇ = dtangent*q̇[1]^2
    SpatialAcceleration(δv̇, zero(δv̇))
end
@inline function _joint_relative_vpa(j::SphericalData, t, q, q̇)
    Q = norm(q)
    Q̇ = (q' * q̇) / Q
    Q̈ = (q̇' * q̇) / Q - (q' * q̇ ) * Q̇ / (Q^2)

    sinc = Q ≈ zero(Q) ? one(Q) : sin(Q) / Q
    sinċ = Q ≈ zero(Q) ? zero(Q) : Q̇ * (cos(Q) * Q - sin(Q)) / (Q^2)
    sinc̈ = Q ≈ zero(Q) ? zero(Q) : Q̈ * (cos(Q) * Q - sin(Q)) / (Q^2) + Q̇^2 * (-Q^2 * sin(Q) - 2*Q*cos(Q) + 2*sin(Q))/(Q^3) 

    rot = Rotor(cos(Q), sinc * q)
    drot_dt = Rotor(-sin(Q) * Q̇, q * sinċ + sinc * q̇, Val{false}())
    ddrot_dt = Rotor(
        -sin(Q)*Q̈ - cos(Q)*Q̇^2, q * sinc̈ + 2*q̇*sinċ,
        Val{false}()
    )
    δω = rotor(j.transform) * (angular_velocity_prematrix(rot) * rotor_to_svector(drot_dt))
    δω̇ = to_rotation_matrix(rotor(j.transform)) * angular_velocity_prematrix(rot) * rotor_to_svector(ddrot_dt) + angular_velocity_prematrix(drot_dt) * rotor_to_svector(drot_dt)

    SpatialAcceleration(zero(δω̇), δω̇)
end


######################
# Joint Acceleration
######################

"""
    joint_relative_acceleration(joint, q, q̈, t)

Returns the acceleration of the child frame relative to the parent frame due to
the acceleration of the joint variable, q̈.

Does NOT include velocity-product-accelerations terms like centrifugal/coriolis
acceleration.

"""
function joint_relative_acceleration(joint, t, q, q̈)
    T = promote_type(valuetype(joint), eltype(q), eltype(q̈), typeof(t))
    ẇ = _joint_relative_acceleration(joint, t, q, q̈)
    SpatialAcceleration{T}(ẇ)
end

_joint_relative_acceleration(jd::AbstractJointData, t, q, q̈) = error("Joint relative acceleration is not implemented for joint type $(typeof(jd))")
# These joints have no joint variable, so no acceleration due to q̈
@inline _joint_relative_acceleration(j::Union{Rigid, ReferenceJoint, TimeFuncJoint}, t, q, q̈) = zero(SpatialAcceleration{eltype(q)})
@inline function _joint_relative_acceleration(j::RevoluteData, t, q, q̈)
    axis = rotor(j.transform) * j.axis # Axis in parent frame
    dω = axis * q̈[1]
    SpatialAcceleration(zero(dω), dω)
end
@inline function _joint_relative_acceleration(j::PrismaticData, t, q, q̈)
    axis = rotor(j.transform) * j.axis
    v̇ = axis * q̈[1]
    SpatialAcceleration(v̇, zero(v̇))
end
@inline function _joint_relative_acceleration(j::HelicalData, t, q, q̈)
    axis = rotor(j.transform) * j.axis
    v̇ = axis * j.lead * q̈[1]
    ω̇ = axis * q̈[1]
    SpatialAcceleration(v̇, ω̇)
end
@inline function _joint_relative_acceleration(j::RailData, t, q, q̈)
    tangent = rotor(j.transform) * spline_derivative(q[1]/j.scaling, j.spline)/j.scaling
    δv = tangent * q̈[1]
    SpatialAcceleration(δv, zero(δv))
end

######################

"""
    child_rbstate(state_parent, state_relative)

Computes the rigid body state of a child from from the parent state, and the 
relative state.
`state_parent` is the state of the parent rigid body (transform, twist, vpa).
`state_relative` contains the relative transform/twist/vpa between the child 
frame and parent frame.
"""
function child_rbstate(rbᵖ::RigidBodyState, rb_relative::RigidBodyState)
    tfᵖ, tfʳ = rbᵖ.transform, rb_relative.transform
    wᵖ, wʳ = rbᵖ.twist, rb_relative.twist
    vpaᵖ, vpaʳ = rbᵖ.acceleration, rb_relative.acceleration

    tfᶜ = child_frame_transform(tfᵖ, tfʳ)
    wᶜ = child_frame_twist(tfᵖ, tfᶜ, wᵖ, wʳ)
    vpaᶜ = child_frame_acceleration(tfᵖ, tfᶜ, wᵖ, wᶜ, vpaᵖ, vpaʳ)
    return RigidBodyState(tfᶜ, wᶜ, vpaᶜ)
end

######################
# Jacobian column
######################
"""
    jacobian_column(joint, tf_parent, tf_final, q_i)

Returns the jacobian columns `Jo` and `Jw` for `joint` such that if all other 
joints where stationary, the linear velocity of a frame with transform tf_final 
would be `Jo*q̇` and the angular velocity would be `Jw*q̇`
"""
function jacobian_column end

function jacobian_column(joint, tfₚ::Transform, final_tf::Transform, t, q::SVector)
    T = promote_type(valuetype(joint), eltype(tfₚ), eltype(q), typeof(t))
    Jo_cols, Jw_cols = _jacobian_columns(joint, tfₚ, final_tf, t, q)
    Nj = length(q)  
    SMatrix{3, Nj, T}(Jo_cols), SMatrix{3, Nj, T}(Jw_cols)
end


function _jacobian_columns(joint::RevoluteData, tfₚ::Transform, final_tf::Transform, t, q)
    # Don't normalize as error here doesn't propagate, like in the kinematics
    # Do the rotor-rotor multiplication first, as this way round is cheaper
    rₚ, rₜ = rotor(tfₚ), rotor(joint.transform)
    r = rotate(rₚ, rₜ, Val{false}()) 
    axis = r * joint.axis
    # Compute lever of jacobian i.e. vector from joint axis to jacobian frame
    δo = origin(final_tf) - tfₚ*origin(joint.transform)
    # Cross axis with lever to get column
    Jo_col = cross(axis, δo)
    Jw_col = axis
    Jo_col, Jw_col
end

function _jacobian_columns(joint::PrismaticData, tfₚ::Transform, final_tf::Transform, t, q)
    axis = rotor(tfₚ) * rotor(joint.transform) * joint.axis
    Jo_col = axis
    Jo_col, zero(Jo_col)
end

function _jacobian_columns(joint::HelicalData, tfₚ::Transform, final_tf::Transform, t, q)
    axis = rotor(tfₚ) * rotor(joint.transform) * joint.axis
    Jo_col = axis * joint.lead
    Jw_col = axis
    Jo_col, Jw_col
end

function _jacobian_columns(joint::RailData, tfₚ::Transform, final_tf::Transform, t, q::SVector{1})
    dspline = spline_derivative(q[1]/joint.scaling, joint.spline)/joint.scaling
    axis = rotor(tfₚ) * rotor(joint.transform) * dspline
    Jo_col = axis
    Jo_col, zero(Jo_col)
end

function _jacobian_columns(joint::SphericalData, tfₚ::Transform, final_tf::Transform, t, q::SVector{3})
    Q = norm(q)
    sinc = Q ≈ zero(Q) ? one(Q) : sin(Q) / Q
    rot = Rotor(cos(Q), sinc * q)

    JQ = (q') / Q
    Jsinc = Q ≈ zero(Q) ? zero(JQ) : JQ * (cos(Q) * Q - sin(Q)) / (Q^2)
    Jscalar = -sin(Q)*JQ
    Jbivector = q * Jsinc + sinc * SMatrix{3, 3}(1, 0, 0, 0, 1, 0, 0, 0, 1)

    R = to_rotation_matrix(rotor(tfₚ) * rotor(joint.transform))
    Jw_cols = R * angular_velocity_prematrix(rot) * vcat(Jscalar, Jbivector)
    Jo_cols = zero(Jw_cols)
    Jo_cols, Jw_cols
end

"""
    autodiff_jacobian_column(joint1dof, tf_parent, tf_child, tf_final, qi)

Used to efficiently compute jacobians using automatic differentiation.

Uses precomputed transforms to compute the derivative of a final transform 
`tf_final` w.r.t. a single joint config, qi, by using knowledge that the other 
transforms are not a function of qi
"""
function autodiff_jacobian_column(jointdata, tf_parent, tf_child, tf_final, t, q_joint::SVector)
    factorized_kinematics_closure = let jointdata=jointdata, tf_parent=tf_parent, tf_child=tf_child, tf_final=tf_final
        q -> begin
            tf_final_to_child = inv(tf_child) * tf_final
            tf_out = tf_parent * joint_relative_transform(jointdata, t, q) * tf_final_to_child 
            tf_out
        end
    end
    diffresult = Hessians.jacobian(factorized_kinematics_closure, q_joint)
    if length(q_joint) == 1
        T = eltype(q_joint)
        Jo_col = SVector{3, T}(origin(jacobian(diffresult)))
        Jr_col = SVector{4, T}(rotor(jacobian(diffresult)))
        Jw_col = quatmul_geodual_bivector_matrix(rotor(tf_final)) * Jr_col
        return Jo_col, Jw_col
    else
        T, N = eltype(q_joint), length(q_joint)
        Jo_cols = SMatrix{3, N, T}(origin(jacobian(diffresult)))
        Jr_cols = SMatrix{4, N, T}(rotor(jacobian(diffresult)))
        Jw_cols = quatmul_geodual_bivector_matrix(rotor(tf_final)) * Jr_cols
        return Jo_cols, Jw_cols
    end
end

# Uncomment this function to allow autodiff for the jacobian column of a joint.
# Be warned that sometimes this doesn't play well with taking a gradient through
# the dyanmics...
# function jacobian_column(joint::JointData, tfₚ::Transform, tf::Transform, final_tf::Transform, q_i::T, t) where T
#     autodiff_jacobian_column(joint, tfₚ, tf, final_tf, q_i, t)
# end

######################
# Project to joint
######################

function _project_forces_to_jointspace(j::RevoluteData, tf_p::Transform{T}, tf_c::Transform{T}, f_p::S, f_c::S, τ_p::S, τ_c::S) where {T, S<: SVector{3, T}} 
    # The torque is expressed in the world frame at the origin of the child frame. 
    # To get the axis in the same rotational frame, we first get it in the parent frame,
    # then rotate it to the world frame

    # The minus sign is due to the fact that the torque is expressed on the right side of the 
    # equation 
    axis = rotor(tf_p) * rotor(j.transform) * j.axis
    u = -dot(axis, τ_c)
    return SVector(u)
end

function _project_forces_to_jointspace(j::PrismaticData, tf_p::Transform{T}, tf_c::Transform{T}, f_p::S, f_c::S, τ_p::S, τ_c::S) where {T, S<: SVector{3, T}} 
    # Rotate axis to world frame, then dot with force
    axis = rotor(tf_c) * j.axis
    u = -dot(axis, f_c)
    return SVector(u)
end

function _project_forces_to_jointspace(j::HelicalData, tf_p::Transform{T}, tf_c::Transform{T}, f_p::S, f_c::S, τ_p::S, τ_c::S) where {T, S<: SVector{3, T}} 
    # Combination of revolute/prismatic 
    axis = rotor(tf_p) * rotor(j.transform) * j.axis
    u = -dot(axis, τ_c) - j.lead * dot(axis, f_c)
    return SVector(u)
end

function _project_forces_to_jointspace(j::RailData, tf_p::Transform{T}, tf_c::Transform{T}, f_p::S, f_c::S, τ_p::S, τ_c::S) where {T, S<: SVector{3, T}} 
    # Rotate axis to world frame, then dot with force
    # TODO test
    dspline = spline_derivative(q[1]/joint.scaling, joint.spline)/joint.scaling
    axis = rotor(tfₚ) * rotor(joint.transform) * dspline
    u = -dot(axis, f_c)
    return SVector(u)
end
