"""
components which do not fall cleanly into an inertance, storage or dissipation.
Must define:
inertance_matrix
generalized_force
stored_energy
"""

"""
    GenericComponent{T} <: ComponentData{T}

Supertype for any components which do not fall cleanly into an inertance, storage or dissipation.

Implement `inertance_matrix!`, `generalized_force!`, and `stored_energy`.
"""
abstract type GenericComponent{T} <: ComponentData{T} end

const DEFAULT_GRAVITY = SVector(0.0, 0.0, -9.81)

"""
    PointMass(mass, coord_id)

A point mass with mass `mass` at coordinate `coord`. The coord should be a 3
dimensional in the global frame, such as a [`FrameOrigin`](@ref) or a [`FramePoint`](@ref).

The point mass is subject to gravity. This can be compensated using a [`GravityCompensator`](@ref)
component, or avoided by using a [`LinearInerter`](@ref) instead of a point mass.
"""
@kwdef struct PointMass{T, C<:CoordID} <: GenericComponent{T}
    mass::T
    coord::C
end

function _inertance_matrix(cache::MechanismCacheBundle, pm::PointMass)
    J = _jacobian(cache, pm.coord)
    J' * pm.mass * J
end

function inertance_matrix!(M::Matrix, cache::MechanismCacheBundle, pm::PointMass)
    J = _jacobian(cache, pm.coord)
    Mw = get_inertance_matrix_workspace(cache)
    custom_transpose_mul_add!(M, J, pm.mass)
    M
end

# TODO clarify sign convention on opspace force: is this force in the direction of ż or against it
function _opspace_force(cache::MechanismCacheBundle, pm::PointMass)
    acc = _acceleration(cache, pm.coord) # Velocity product acceleration
    m = pm.mass
    g::SVector{3} = get_gravity(cache)[]

    F_coriolis = acc * m # Centrifugal/Coriolis forces
    F_gravity = - m * g
    F_coriolis + F_gravity
end

function _stored_energy(cache::MechanismCacheBundle, pm::PointMass)
    z = _configuration(cache, pm.coord)
    ż = _velocity(cache, pm.coord)
    g = get_gravity(cache)[]
    KE = 0.5 * pm.mass * dot(ż, ż)
    PE = pm.mass * dot(-g, z)
    KE + PE
end

# Defined here because PointMass is defined later than Gravity Compensator
GravityCompensator(pm::PointMass, gravity=DEFAULT_GRAVITY) = GravityCompensator(pm.mass, pm.coord, gravity)
