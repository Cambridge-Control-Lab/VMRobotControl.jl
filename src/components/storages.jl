"""
    Storage{T} <: ComponentData{T}

Supertype for components that act as potential energy storages, with float type `T`.

These must support implement `stored_energy` and `generalized_force!`, typically by 
implementing `_stored_energy` and `_opspace_force` functions, respectively.
"""
abstract type Storage{T} <: ComponentData{T} end

############################
# LinearSpring
############################

"""
    LinearSpring(stiffness, coord::OperationSpace)

A linear spring acting on the provided coordinate. The spring stiffness 
`stiffness` can be any type that supports multiplication with the coordinate's 
configuration, but should be positive definite to ensure it's energy is bounded
below, and should not cause allocations when multiplied (i.e. use a Float64 or 
an SMatrix).
"""
@kwdef struct LinearSpring{T, K, C} <: Storage{T}
    stiffness::K
    coord::C
    function LinearSpring(stiffness::K, coord::C) where {K, C}
        (isposdef(stiffness) || iszero(stiffness)) || @warn "Expected spring stiffness to be positive definite (or zero): '$(stiffness)'"
        isa(stiffness, Array) && @error "Warning, used an Array (probably a matrix) as a stiffness. This will cause allocations. Use a StaticArray (SMatrix) instead."
        new{eltype(K), K, C}(stiffness, coord)
    end
end

# Constructors
LinearSpring(stiffness, p::String, c::String) = LinearSpring(stiffness, RotatedCoord(p, c))

function _stored_energy(cache::CacheBundle, storage::LinearSpring)
    k, coord = storage.stiffness, storage.coord
    z = _configuration(cache, coord)
    0.5 * z' * k * z
end

function _opspace_force(cache::CacheBundle, storage::LinearSpring)
    k, coord = storage.stiffness, storage.coord
    z = _configuration(cache, coord)
    k * z
end

Base.isapprox(a::LinearSpring, b::LinearSpring) = (a.coord == b.coord) & (a.stiffness ≈ b.stiffness)

function Base.show(io::IO, ::MIME"text/plain", c::LinearSpring)
    print(io, "LinearSpring(stiffness ")
    print(io, c.stiffness)
    print(io, ", coord $(c.coord))")
end

############################
# TanhSpring
############################

"""
    TanhSpring(coord::OperationSpace; stiffness, max_force)

A spring which applies a force of magnitude tanh(|extension|) acting on the 
provided coordinate. The spring can be configured in terms of 2 of 3 variables:
either a `width` a `max_force`, or a `stiffness`, which are each scalar.

The force is determined as:
    F = kβ tanh(|z|/β) z/|z|
where `z` is the coordinate's configuration, `β` is the width of the spring, and
`k`` is the stiffness. The max force σ is `kβ`.

This is useful as the tanh function saturates to a constant force for large 
extensions, which can prevent the robot from applying too much force.
"""
@kwdef struct TanhSpring{T<:Real, C} <: Storage{T}
    stiffness::T
    width::T
    coord::C
    function TanhSpring(stiffness::T, width::T, c::C) where {T, C}
        stiffness > 0 || @warn "Expected tanh spring stiffness to be positive: '$(stiffness)'"
        new{T, C}(stiffness, width, c)
    end
end

function TanhSpring(c::CoordID; stiffness=nothing, max_force=nothing, width=nothing)
    @assert sum([isnothing(stiffness), isnothing(max_force), isnothing(width)]) == 1 "Expected exactly two of 'stiffness', 'max_force', 'width'."
    if isnothing(stiffness)
        _stiffness = max_force/width
        return TanhSpring(_stiffness, width, c)
    elseif isnothing(max_force)
        return TanhSpring(stiffness, width, c)
    elseif isnothing(width)
        _width = max_force/stiffness
        return TanhSpring(stiffness, _width, c)
    end
    error("Unknown error. $stiffness, $max_force, $width")
end

function logcosh(x)
    # More numberically stable, as cosh on its own can overflow
    # exponent always has real part <= 0
    s = abs(x)
    p = exp(-2 * s)
    return s + log1p(p) - log(2)
end

function _stored_energy(cache::CacheBundle, s::TanhSpring)
    k, β = s.stiffness, s.width
    z = _configuration(cache, s.coord)
    e = norm(z)
    return (k*β^2) * logcosh(e/β)
end

function _opspace_force(cache::CacheBundle, s::TanhSpring)
    k, β = s.stiffness, s.width
    z = _configuration(cache, s.coord)
    e, dir = norm(z), normalize(z)
    F = (e == zero(e) ? zero(dir) : k*β*tanh(e/β) * dir)
    F
end

function Base.isapprox(a::TanhSpring, b::TanhSpring)
    ret1 = a.coord == b.coord
    ret2 = a.stiffness ≈ b.stiffness
    ret3 = a.width ≈ b.width
    return ret1 & ret2 & ret3
end

#########################################
# GravityCompensator
#########################################

"""
    GravityCompensator(mass::T, coord::OperationSpace{3}, gravity::SVector{3, T})
    GravityCompensator(;mass, coord, gravity=DEFAULT_GRAVITY)

An energy storage acting in the opposite direction to the vector `gravity`.
Can be used to compensate for the potential energy of a pointmass on the same
coordinate, without affecting inertia.

Note that the stored energy is potentially unbounded for robots with unbounded
workspace, but is reported as the potential energy of the mass at the current
configuration.

See also [`add_gravity_compensator!`](@ref), [`add_gravity_compensation!`](@ref).
"""
@kwdef struct GravityCompensator{T, C} <: Storage{T}
    mass::T
    coord::C
    gravity::SVector{3, T}=DEFAULT_GRAVITY
end
# TODO refactor to store only weight, as separate mass and gravity are not
# needed.

GravityCompensator(gc::GravityCompensator, gravity) = remake(gc; gravity=gravity)
# GravityCompensator(;mass::T, coord, gravity=DEFAULT_GRAVITY) where T = GravityCompensator(mass, coord, SVector{3, T}(gravity))
# GravityCompensator(mass::T, coord) where T = GravityCompensator(mass, coord, SVector{3, T}(DEFAULT_GRAVITY))

function _opspace_force(cache::CacheBundle, gc::GravityCompensator)
    m, g = gc.mass, gc.gravity
    m * g
end

function _stored_energy(cache::CacheBundle, gc::GravityCompensator)
    m, g = gc.mass, gc.gravity
    z = _configuration(cache, gc.coord)
    m * dot(g, z)
end

function Base.isapprox(a::GravityCompensator, b::GravityCompensator; kwargs...)
    (a.coord == b.coord) & isapprox(a.mass, b.mass; kwargs...)
end

"""
    add_gravity_compensator!(mechanism, point_mass, gravity; id=nothing)

Add a gravity compensator to componsate for `point_mass`, when subject to `gravity`.
"""
function add_gravity_compensator!(mechanism, point_mass, gravity; id=nothing)
    gc = GravityCompensator(point_mass, gravity) # Convert to GravityCompensator if not already (e.g. PointMass)
    id = isnothing(id) ? "GravityCompensator_$(hash(point_mass)))" : id
    add_component!(mechanism, gc; id=id)
end

"""
    add_gravity_compensation!(system, gravity)

Add gravity compensators for all point masses.

If system is a `VirtualMechanismSystem`, then only point masses in the robot are compensated,
and the `GravityCompensator` components are added to the system, so that they are treated as 
part of the controller, not part of the robot.  
"""
function add_gravity_compensation!(system, gravity;)
    if system isa Mechanism
        mechanism = system
        for (name, component) in generic_components(mechanism)
            if component isa PointMass
                gc = GravityCompensator(component.mass, component.coord, gravity)
                id = "GravityCompensator_$(hash(component))"
                add_component!(mechanism, gc; id)
            end
        end
    elseif system isa VirtualMechanismSystem
        for (name, component) in generic_components(system.robot)
            if component isa PointMass
                gc = GravityCompensator(component.mass, ".robot.$(component.coord)", gravity)
                id = "GravityCompensator_$(hash(component)))"
                add_component!(system, gc; id)
            end
        end    
    else
        error("Expected a Mechanism or VirtualMechanismSystem, got $(typeof(system))")
    end
end

############################
# GaussianSpring
############################

"""
    GaussianSpring(σ, stiffness, coordid)
    GaussianSpring(coordid; stiffness, max_force, width)

A spring with a Gaussian energy profile, which is useful for acting as a
localized attractive/repulsive force (repulsive when stiffness is negative).

The energy function is a gaussian with std deviation `σ`, scaled so that the
gradient of it's derivative (the force) is `k` at |z|=0:
    E = -kσ² exp( -|z|²/(2σ²) )
where `z` is the coordinate's configuration.

When calling with keyword arguments, specify any two, as this fully defines 
the Gaussian energy function.
"""
@kwdef struct GaussianSpring{T<:Real, C} <: Storage{T}
    σ::T
    stiffness::T
    coord::C
    function GaussianSpring(σ::T, stiffness::T, c::C) where {T, C}
        # Note stiffness can be negative for repulsive field
        new{T, C}(σ, stiffness, c)
    end
end

# Max force is at x = ±σ
gaussian_spring_max_force(k, σ) = gaussian_spring_force(σ, k, σ) 
gaussian_spring_stiffness(σ, F_max) = F_max * exp(0.5)/ σ
gaussian_spring_std(F_max, k) = F_max * exp(0.5) / k

function GaussianSpring(c; stiffness=nothing, max_force=nothing, width=nothing)
    Nargs = 3 - (isnothing(stiffness) + isnothing(max_force) + isnothing(width))
    Nargs != 2 && (error("Call GaussianSpring(coord; stiffness, max_force, width) with exactly two"*
    " keyword-arguments, as any two specify the third."))
    if isnothing(stiffness)
        σ, k = width, gaussian_spring_stiffness(width, max_force)
    elseif isnothing(max_force)
        σ, k = width, stiffness
    elseif isnothing(width)
        σ, k = gaussian_spring_std(max_force, stiffness), stiffness
    else
        error("Unknown error")
    end
    GaussianSpring(σ, k, c)
end

# function GaussianSpring(c::C; width, max_force) where C<: OperationSpace
#     stiffness = gaussian_spring_stiffness(width, max_force)
#     GaussianSpring(width, stiffness, c)
# end


@inline gaussian_spring_energy(x, k, σ) = -k*σ^2*exp( -x^2/(2σ^2) )
@inline gaussian_spring_force(x, k, σ) = -(x/(σ^2)) * gaussian_spring_energy(x, k, σ)

function _stored_energy(cache::CacheBundle, s::GaussianSpring)
    k, σ = s.stiffness, s.σ
    z = _configuration(cache, s.coord)
    e = norm(z)
    return gaussian_spring_energy(e, k, σ)
end

function _opspace_force(cache::CacheBundle, s::GaussianSpring)
    k, σ = s.stiffness, s.σ
    z = _configuration(cache, s.coord)
    e, dir = norm(z), normalize(z)
    F_mag = gaussian_spring_force(e, k, σ)
    F = (e == zero(e) ? zero(dir) : F_mag * dir)
    F
end

function Base.isapprox(a::GaussianSpring, b::GaussianSpring)
    ret1 = a.coord == b.coord
    ret2 = a.stiffness ≈ b.stiffness
    ret3 = a.σ ≈ b.σ
    return ret1 & ret2 & ret3
end

############################
# ReLUSpring
############################

@kwdef struct RectifiedSpring{T<:Real, K, C} <: Storage{T}
    stiffness::K
    coord::C
    flipped::Bool
    function RectifiedSpring(stiffness::K, c::C, flipped::Bool) where {K, C}
        stiffness > 0 || @warn "Expected ReLU spring stiffness to be positive: '$(stiffness)'"
        new{eltype(K), K, C}(stiffness, c, flipped)
    end
end

const ReLUSpring{T, C} = RectifiedSpring{T, C}
ReLUSpring(args...;kwargs...) = RectifiedSpring(args...; kwargs...)

"""
add_deadzone_springs!(mechanism, stiffness, bounds, coord_id)

    Creates a spring with a deadzone, that acts with stiffness `stiffness` outside of 
    bounds `bounds`, e.g. (-1.0, 1.0), on the coordinate `coord_id`. 
    
    The deadzone is implemented as two ReLU springs on two new coordinates, which are
    offset from the original coordinate by the bounds. This is because the ReLU spring 
    is centered at zero, so the new coordinates are used to shift the 'zero point' to
    match the bounds.
"""
function add_deadzone_springs!(mechanism, stiffness, bounds, coord_id)
    lb, ub = bounds
    lb_coord_id = add_coordinate!(mechanism, ConstCoord(lb); id="lb_$coord_id")
    ub_coord_id = add_coordinate!(mechanism, ConstCoord(ub); id="ub_$coord_id")
    ext_lower = add_coordinate!(mechanism, CoordDifference(coord_id, lb_coord_id); id="ext_lower_$coord_id")
    ext_upper = add_coordinate!(mechanism, CoordDifference(coord_id, ub_coord_id); id="ext_upper_$coord_id") 
    s_lower = ReLUSpring(stiffness, ext_lower, true) # true ⟹ Flip direction of rectification
    s_upper = ReLUSpring(stiffness, ext_upper, false) 
    add_component!(mechanism, s_lower; id="$(coord_id)_spring_lower")
    add_component!(mechanism, s_upper; id="$(coord_id)_spring_upper")
    mechanism
end

function add_deadzone_springs!(mechanism, stiffness, bounds, coord_id, T, N)
    # Legacy function signature, ignores last two arguments
    add_deadzone_springs!(mechanism, stiffness, bounds, coord_id)
end

function DeadzoneSpring(stiffness, coord::CoordinateData, bounds::Tuple{T, T}) where T
    error("Use `add_deadzone_springs!` to add deadzone springs to a mechanism.")
end

function ReLU_extension(cache::CacheBundle, s::ReLUSpring)
    z = _configuration(cache, s.coord)
    T = eltype(cache)
    if s.flipped
        #         | 
        #  -------+------
        #       * |     
        #     *   |     
        #   *     |     
        return min.(z, zero(T))
    else
        #     |     *
        #     |   *
        #     | *
        #  ---+------
        #     | 
        return max.(z, zero(T)) 
    end
end

function _stored_energy(cache::CacheBundle, s::ReLUSpring)
    k = s.stiffness
    e = ReLU_extension(cache, s)
    return 0.5 * dot(e, k, e)
end

function _opspace_force(cache::CacheBundle, s::ReLUSpring)
    k = s.stiffness
    e = ReLU_extension(cache, s)
    k * e
end

function Base.isapprox(a::ReLUSpring, b::ReLUSpring)
    ret1 = a.coord == b.coord
    ret2 = a.stiffness ≈ b.stiffness
    ret3 = a.flipped == b.flipped
    return ret1 & ret2 & ret3
end



"""
    ForceSource(force_max, power_max, coord)

A power source which applies force  up to `force_max` in the direction of `force_max` maximum
with power `power_max`. The force is applied at `coord`. 

If power max is a scalar, then it represents a maximum power flow into the system. If it is a tuple
of two scalars, then the first element bounds power flow into the system, and the second element
bounds the power flow out of the system.
"""
struct ForceSource{T, C<:CoordID, N, P} <: Storage{T}
    force_max::SVector{N, T}
    power_max::P
    coord::C
    function ForceSource(force_max::SVector, power_max::P, coord::CoordID) where P
        @assert (P<:Real) || (P <: Tuple{<:Real, <:Real}) "Expected power_max to be a scalar or a tuple of two scalars: '$(power_max)'"
        if P <: Tuple
            @assert power_max[1] >= 0 "Expected power_max[1] to be non-negative: '$(power_max[1])'"
            @assert power_max[2] <= 0 "Expected power_max[2] to be non-positive: '$(power_max[2])'"
        end
        T = eltype(force_max)
        C = typeof(coord)
        N = length(force_max)
        new{T, C, N, P}(force_max, power_max, coord)
    end
    function ForceSource(; force_max, power_max, coord) # kwdef constructor
        ForceSource(force_max, power_max, coord)
    end
end

function _opspace_force(cache::CacheBundle, fs::ForceSource)
    F = fs.force_max
    Ė = F' * velocity(cache, fs.coord)
    
    ret = if fs.power_max isa Real
        # Clamp if supplied power is too high
        Ė > fs.power_max ? F * fs.power_max / Ė : F
    else
        if Ė > fs.power_max[1]
            # Clamp if supplied power is too high
            F * fs.power_max[1] / Ė
        elseif Ė < fs.power_max[2]
            # Clamp if extracted power is too high
            F * fs.power_max[2] / Ė
        else
            F
        end
    end
    return -ret # Negate here as the applied force is the negative of opspace_force
end

function _stored_energy(cache::CacheBundle, fs::ForceSource)
    Inf
end


"""
    PowerSource(force_max, power_max, coord)

A power source which applies a force up to `force_max` in the direction that will inject power into
the system up to `power_max`. Cannot extract power from the system. The force is applied at `coord`.
"""
struct PowerSource{T, C<:CoordID, N, P<:Real} <: Storage{T}
    force_max::SVector{N, T}
    power_max::P
    coord::C
    function PowerSource(force_max::SVector, power_max::P, coord::CoordID) where P
        T = eltype(force_max)
        C = typeof(coord)
        N = length(force_max)
        new{T, C, N, P}(force_max, power_max, coord)
    end
    function PowerSource(; force_max, power_max, coord) # kwdef constructor
        PowerSource(force_max, power_max, coord)
    end
end

function _opspace_force(cache::CacheBundle, ps::PowerSource)
    v = velocity(cache, ps.coord)
    F = ps.force_max
    F = dot(v, ps.force_max) >= 0 ? F : -F    
    Ė = dot(F, v)
    # Clamp if supplied power is too high
    F_limited =  Ė > ps.power_max ? F * ps.power_max / Ė : F
    return - F_limited # Negate here as the applied force is the negative of opspace_force
end

function _stored_energy(cache::CacheBundle, ps::PowerSource)
    Inf
end
