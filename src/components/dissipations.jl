"""
    Dissipation{T} <: ComponentData{T}

Supertype for all dissipative components, which cannot store energy and only
dissipate it. 

These components support `generalized_force!`, typically by implementing
`_opspace_force`. 
"""
abstract type Dissipation{T} <: ComponentData{T} end

######################
# LinearDamper
######################
"""
    LinearDamper(damping, coord::OperationSpace)

A linear damper acting on the provided coordinate. The damping coefficient 
`damping` can be any type that supports multiplication with the coordinate's 
velocity, but should be positive definite to ensure it is dissipative, and
should not cause allocations when multiplied (i.e. use a Float64 or an SMatrix).
"""
@kwdef struct LinearDamper{T, K, C} <: Dissipation{T}
    damping::K
    coord::C
    function LinearDamper(damping::K, coord::C) where {K, C}
        # (isposdef(damping) || iszero(damping)) || @warn "Expected damper damping to be positive definite: '$(damping)'"
        isa(damping, Array) && @error "Warning, used an Array (probably a matrix) as a damper coefficient. This will cause allocations. Use a StaticArray (SMatrix) instead."
        new{eltype(K), K, C}(damping, coord)
    end
end

# Constructors
LinearDamper(damping, p::String, c::String) = LinearDamper(damping, RotatedCoord(p, c))
LinearDamper(damping, p::Int, c::Int) = LinearDamper(damping, RotatedCoord(p, c))

function _opspace_force(cache::CacheBundle, d::LinearDamper, )
    c, coord, = d.damping, d.coord
    ż = _velocity(cache, coord)
    c * ż
end

Base.isapprox(a::LinearDamper, b::LinearDamper) = (a.coord == b.coord) & (a.damping ≈ b.damping)

######################
# RectifiedDamper
######################

"""
    RectifiedDamper(damping, coord::OperationSpace, bounds, flipped, diodic)

A linear damper acting on the provided coordinate only in a certain position 
range. If position `z` is greater than ub, full damping is applied, if it is
between the lower bound and upper bound, partial damping is applied, and if it 
is less than the lower bound, no damping is applied.

If `flipped` is true, this is inverted and the damper is applied when `z` is 
less than the lower bound, and not applied when `z` is greater than the upper.

If `diodic` is true, the damper is applied only when the velocity is positive
(or negative if `flipped` is true).
"""
@kwdef struct RectifiedDamper{T, K, C, B} <: Dissipation{T}
    damping::K
    coord::C
    bounds::Tuple{B, B}
    flipped::Bool
    diodic::Bool
    function RectifiedDamper(damping::K, coord::C, bounds::Tuple{B, B}, flipped::Bool, diodic::Bool) where {K, C<:CoordID, B}
        isposdef(damping) || @warn "Expected damper damping to be positive definite: '$(damping)'"
        all(bounds[1] .< bounds[2]) || throw(ErrorException("Expected lower bound(s) to be less than upper bound(s): $(bounds[1]), $(bounds[2])"))
        new{eltype(K), K, C, B}(damping, coord, bounds, flipped, diodic)
    end
end

RectifiedDamper(d, c, b, f; diodic=false) = RectifiedDamper(d, c, b, f, diodic)

function rectified_damper_force(z, ż, c, bounds, flipped, diodic)
    T = eltype(z)
    lb, ub = bounds
    # Damps when z is greater than ub.
    # z > ub ⟹ coeff=1.0,   z < lb ⟹ coeff=0.0
    coeffs = @. max(zero(T), min(one(T), (z - lb)/(ub - lb)))
    if flipped
        # Damps when z is less than lb.
        # z < lb ⟹ coeff=1.0,   z > ub ⟹ coeff=0.0
        coeffs = one(T) .- coeffs
    end
    F = c * coeffs .* ż
    if diodic 
        F = flipped ? min.(F, zero(T)) : max.(F, zero(T))
    end
    F
end

function _opspace_force(cache::CacheBundle, d::RectifiedDamper{T}) where T
    c, coord, = d.damping, d.coord
    z = _configuration(cache, coord)
    ż = _velocity(cache, coord)
    rectified_damper_force(z, ż, c, d.bounds, d.flipped, d.diodic)
end

######################
# DiodicDamper
######################

"""
    DiodicDamper(damping, coord::OperationSpace, flipped)

A that acts only when the velocity is positive (or negative if `flipped` is 
true).
"""
@kwdef struct DiodicDamper{T, K, C} <: Dissipation{T}
    damping::K
    coord::C
    flipped::Bool
    function DiodicDamper(damping::K, coord::C, flipped::Bool) where {K, C<:CoordID}
        isposdef(damping) || @warn "Expected damper damping to be positive definite: '$(damping)'"
        new{eltype(K), K, C}(damping, coord, flipped)
    end
end

function _opspace_force(cache::CacheBundle, d::DiodicDamper{T}) where T
    c, coord, = d.damping, d.coord
    ż = _velocity(cache, coord)
    
    F = c * ż
    F = d.flipped ? min.(F, zero(T)) : max.(F, zero(T))
    F
end


######################
# Tanh Damper
######################

"""
    TanhDamper(max_force, width, coord::OperationSpace)

A damper that applies a constant damping force above a certain velocity, using a tanh function.
The `width` parameter determines the shape of the tanh, at 2*width, the damping force is ~95% of the 
maximum, and at 3*width, the damping force is ~99.5% of the maximum.
"""
@kwdef struct TanhDamper{T, F, W, C} <: Dissipation{T}
    max_force::F
    width::T
    coord::C
    function TanhDamper(max_force::F, width::W, coord::C) where {F, W, C}
        max_force >= zero(max_force) || error("Expected max force to be positive: '$(max_force)'")
        width >= zero(width) || error("Expected width to be positive: '$(width)'")
        T = eltype(max_force)
        @assert W == T
        new{eltype(F), F, T, C}(max_force, width, coord)
    end
end

function _opspace_force(cache::CacheBundle, d::TanhDamper{T}) where T
    F, β, coord, = d.max_force, d.width, d.coord
    z = _velocity(cache, coord)
    v, dir = norm(z), normalize(z)
    F = (v == zero(v) ? zero(dir) : F*tanh(v/β) * dir)
    F
end
