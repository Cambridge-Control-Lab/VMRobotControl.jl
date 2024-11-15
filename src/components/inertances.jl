"""
    Inertance{T} <: ComponentData{T}

Supertype for all inertance components, which store kinetic energy and induce
inertances and generalized forces in the mechanism.

These components support `stored_energy`, `inertance_matrix!`, and `generalized_force!`,
typically by implementing `_stored_energy`, `inertance_matrix!`, and `_opspace_force`.
"""
abstract type Inertance{T} <: ComponentData{T} end

############################
# LinearInerter
############################

"""
    LinearInerter(inertance, coord)

A linear inertance with inertance matrix `inertance` across coordinate `coord`. 

The `inertance` matrix should be positive definite, and should multiply with
the velocity vector to produce a force vector, or the jacobian to produce an
inertance matrix. It should not cause allocation when multiplied (i.e use 
`SMatrix` or `Float64`).

A linear inerter will have the same inerting effect as a point mass if the
inertance equals the mass, and the coordinate is the center of mass, but is 
not subject to gravity.
"""
@kwdef struct LinearInerter{T, K, C} <: Inertance{T}
    inertance::K
    coord::C
    function LinearInerter(inertance::K, coord::C) where {K, C}
        isposdef(inertance) || @warn "Expected inertance to be positive definite: '$(inertance)'"
        new{eltype(K), K, C}(inertance, coord)
    end
end

# Legacy
function LinearInerter(inertance, p::String, c::String)
    coord = RotatedCoord(p, c)
    LinearInerter(inertance, coord)
end

function _stored_energy(cache::MechanismCacheBundle, inr::LinearInerter)
    m, coord = inr.inertance, inr.coord
    v = _velocity(cache, coord)
    0.5 * v' * m * v
end

# function _inertance_matrix(cache::MechanismCacheBundle, inr::LinearInerter)
#     m, coord = inr.inertance, inr.coord
#     J = _jacobian(cache, coord)
#     J' * m * J
# end

function inertance_matrix!(M::Matrix, cache::MechanismCacheBundle, inr::LinearInerter)
    m, coord = inr.inertance, inr.coord
    J = _jacobian(cache, coord)
    # # TODO try turbo with enzyme
    # @turbo for i = axes(M, 1), j = axes(M, 2)
    #     M_ij = zero(eltype(M))
    #     for k = axes(J, 1)
    #         M_ij += J[k, i] * m * J[k, j]
    #     end
    #     M[i, j] += M_ij
    # end
    mul!(M, J', J, m, true) # M = M + J' * m * J # TODO check this with SMatrix for m...
    nothing
end

function _opspace_force(cache::MechanismCacheBundle, inr::LinearInerter)
    m, coord = inr.inertance, inr.coord
    vpa = _acceleration(cache, coord)
    m * vpa
end

function Base.isapprox(a::LinearInerter, b::LinearInerter)
    (a.coord == b.coord) & (a.inertance ≈ b.inertance)
end

function Base.show(io::IO, ::MIME"text/plain", c::LinearInerter)
    print(io, "LinearInerter(inertance ")
    print(io, c.inertance)
    print(io, ", $(c.coord)")
end

###################
# Inertia
###################
"""
    Inertia(inertia::SMatrix{3, 3}, coord)

A rotational inertia. Coord must be a [`FrameAngularVelocity`](@ref) coordinate.

See also [`add_inertia!`](@ref).
"""
@kwdef struct Inertia{T, CID<:CoordID} <: Inertance{T}
    inertia::SMatrix{3, 3, T, 9}
    coord::CID
end

"""
    add_inertia!(mechanism, frameID, inertia; id, coordID=nothing)

Add an inertia to a mechanism. A FrameAngularVelocity coordinate is added to the
mechanism, on the frame with ID `frameID`, and is given the ID `coordID` if 
provided. 

The inertia is defined by a 3x3 matrix `inertia`.
"""
function add_inertia!(mechanism, frameID::String, inertia::SMatrix{3, 3}; id, coordID=nothing)
    T = eltype(mechanism)
    avel = FrameAngularVelocity(frameID)
    isnothing(coordID) && (coordID = "__FrameAngularVelocity_"*frameID)
    add_coordinate!(mechanism, avel; id=coordID)
    add_component!(mechanism, Inertia(inertia, coordID); id)
    mechanism
end

_get_inertia_rotor(cache, inr::Inertia) = rotor(get_transform(cache, coordinate(cache, inr.coord).coord_data.frameID))

function change_inertia_frame(r::Rotor, I::SMatrix{3, 3})
    R = Transforms.rotation_matrix(r)
    R*I*R'
end

function rotating_inertia_derivative(r::Rotor, ω::SVector{3}, I::SMatrix{3, 3})
    R = Transforms.rotation_matrix(r)
    Ṙ = rotation_matrix_derivative(R, ω)
    Ṙ*I*R' + R*I*Ṙ'
end

function _stored_energy(cache::MechanismCacheBundle, inr::Inertia)
    r = _get_inertia_rotor(cache, inr)
    ω = _velocity(cache, inr.coord)
    Iʷ = change_inertia_frame(r, inr.inertia) # Inertia in world frame
    0.5 * dot(ω, Iʷ, ω)
end

# function _inertance_matrix(cache::MechanismCacheBundle, inr::Inertia)
#     r = _get_inertia_rotor(cache, inr)
#     J = _jacobian(cache, inr.coord)
#     Iʷ = change_inertia_frame(r, inr.inertia) # Inertia in world frame
#     inertance_matrix = J' * Iʷ * J
#     return inertance_matrix
# end

function inertance_matrix!(M::Matrix, cache::MechanismCacheBundle, inr::Inertia)
    r = _get_inertia_rotor(cache, inr)
    J = _jacobian(cache, inr.coord)
    Iʷ = change_inertia_frame(r, inr.inertia) # Inertia in world frame
    Mw = get_inertance_matrix_workspace(cache)
    
    @assert ~any(isnan, J) "Jacobian has NaNs: $J"

    if size(Mw, 1) >= 3 # Then we can use Mw as a cache
        A = view(Mw, 1:3, 1:size(J,2))
        
        # First do A = Iʷ * J
        # Naive matmul as Iʷ is an SMatrix
        for i = axes(A, 1), j = axes(A, 2)
            A_ij = zero(eltype(M))
            for k = axes(Iʷ, 2)
                A_ij += Iʷ[i, k] * J[k, j]
            end
            A[i, j] = A_ij
        end    
        # Then do M += J' * A
        for i = axes(M, 1), j = axes(M, 2) 
            M_ij = zero(eltype(M))
            for k = axes(J, 1)
                M_ij += J[k, i] * A[k, j]
            end
            # We know the result is symmetric, so we can add both at once
            # However, when i == j, we will double count
            M[i, j] += M_ij
        end
    else    
        # Do it all in one go, this gets slow for large N
        _1_to_3 = SVector{3, Int}(1, 2, 3)
        for i = axes(M, 1), j = axes(M, 2)
            M_ij = zero(eltype(M))
            for k = axes(J, 1)
                M_ij += J[k, i] * (dot(Iʷ[k, _1_to_3], J[_1_to_3, j]))
            end
            M[i, j] += M_ij
        end
    end

    # Check symmetry
    # TODO assess performance of this
    for i = 1:size(M, 1)
        for j = (i+1):size(M, 2)
            if M[i, j] != M[j, i]
                if isapprox(M[i, j], M[j, i]; atol=1e-8, rtol=1e-8)
                    M[i, j] = M[j, i]
                else
                    error("Inertance matrix is not symmetric: $(M[i, j]) != $(M[j, i])")
                end
            end
        end
    end
    M
end

function _opspace_force(cache::MechanismCacheBundle, inr::Inertia)
    r = _get_inertia_rotor(cache, inr)
    ω = _velocity(cache, inr.coord)
    ẇ = _acceleration(cache, inr.coord)
    Iʷ = change_inertia_frame(r, inr.inertia) # Inertia in world frame
    İʷ = rotating_inertia_derivative(r, ω, inr.inertia) # Rate of change of world frame inertia
    Iʷ*ẇ + İʷ*ω
end
