"""
    CoordinateData

An abstract type representing an operation space coordinate.

See also: [`configuration`](@ref), [`velocity`](@ref), [`jacobian`](@ref) and [`acceleration`](@ref)
"""
abstract type CoordinateData end
Base.length(::Type{<:CoordinateData}) = error("CoordinateData does not have a length")
Base.length(c::CoordinateData) = length(typeof(c))
Base.eltype(::Type{<:CoordinateData}) = error("CoordinateData does not have an eltype")
Base.eltype(c::CoordinateData) = eltype(typeof(c))

cache_size(c::CoordinateData) = cache_size(typeof(c))
remaker_of(c::CoordinateData) = parameterless_type(c)

struct CompiledCoord{C<:CoordinateData}
    coord_data::C
    cache_idxs::UnitRange{Int}
end

# Base.show(io::IO, ::Type{<:CompiledCoord{C}}) where C = print(io, "CompiledCoord{$C}")

"""
    CompiledCoordID{C}

A unique identifier for a coordinte in a compiled mechanism/virtual mechanism system.
"""
struct CompiledCoordID{C} <: AbstractCompiledMechanismIndex
    depth::Int64
    idx::TypeStableIdx{CompiledCoord{C}}
    function CompiledCoordID(depth, idx::TypeStableIdx{CompiledCoord{C}}) where C
        new{C}(depth, idx)
    end
end

Base.show(io::IO, ::Type{CompiledCoordID{C}}) where C = print(io, "CompiledCoordID{$C}")

# Virtual Mechanism System Coord Index 
struct VMSCoordID{C, S} <: AbstractCompiledVirtualMechanismSystemIndex
    idx::CompiledCoordID{C}
    VMSCoordID{ON_ROBOT}(idx::CompiledCoordID{C}) where C = new{C, ON_ROBOT}(idx)
    VMSCoordID{ON_VIRTUAL_MECHANISM}(idx::CompiledCoordID{C}) where C = new{C, ON_VIRTUAL_MECHANISM}(idx)
    VMSCoordID{ON_SYSTEM}(idx::CompiledCoordID{C}) where C = new{C, ON_SYSTEM}(idx)
end
_vms_location(::VMSCoordID{T, S}) where {T, S} = Val{S}()

Base.show(io::IO, ::Type{<:VMSCoordID{C, S}}) where {C, S} = print(io, "VMSCoordID{$C, $S}")


const CoordID = Union{String, CompiledCoordID, VMSCoordID}

Base.eltype(::Type{<:CompiledCoord{C}}) where C = eltype(C)
Base.getindex(c::Vector{<:TypeStableCollection}, idx::CompiledCoordID) = getindex(getindex(c, idx.depth), idx.idx)
Base.setindex!(c::Vector{<:TypeStableCollection}, val, idx::CompiledCoordID) = setindex!(getindex(c, idx.depth), val, idx.idx)

coord_type(::String) = missing
coord_type(::CompiledCoord{C}) where C = C
coord_type(::CompiledCoordID{C}) where C = C
coord_type(::VMSCoordID{C, S}) where {C, S} = C
coord_type(::Type{String}) = missing
coord_type(::Type{<:CompiledCoord{C}}) where C = C
coord_type(::Type{<:CompiledCoordID{C}}) where C = C
coord_type(::Type{<:VMSCoordID{C, S}}) where {C, S} = C

const MetaCoord = Union{CompiledCoord, CompiledCoordID, VMSCoordID}

Base.length(c::Type{<:MetaCoord}) = length(coord_type(c))
Base.length(c::MetaCoord) = length(coord_type(c))
Base.eltype(::Type{C}) where C <:MetaCoord = eltype(coord_type(C))
cache_size(c::MetaCoord) = cache_size(coord_type(c))
has_configuration(c::Type{<:MetaCoord}) = has_configuration(coord_type(c))
has_configuration(c::MetaCoord) =         has_configuration(coord_type(c))


# Coordinate cache access
# Configuration, velocity, velocity-product acceleration and jacobian cache access
# TODO move forwarding to same place as all other cachebundle forwarding functions.
@inline z_cache_view(bundle::CacheBundle, c::CompiledCoord) = z_cache_view(bundle.cache.coord_cache, c)
@inline ż_cache_view(bundle::CacheBundle, c::CompiledCoord) = ż_cache_view(bundle.cache.coord_cache, c)
@inline α_cache_view(bundle::CacheBundle, c::CompiledCoord) = α_cache_view(bundle.cache.coord_cache, c)
@inline f_cache_view(bundle::CacheBundle, c::CompiledCoord) = f_cache_view(bundle.cache.coord_cache, c)
@inline J_cache_view(bundle::CacheBundle, c::CompiledCoord) = J_cache_view(bundle.cache.coord_cache, c)

z_cache_view(cache::AbstractCoordinateCache, c::CompiledCoord) = view(cache.z, c.cache_idxs)
ż_cache_view(cache::AbstractCoordinateCache, c::CompiledCoord) = view(cache.ż, c.cache_idxs)
α_cache_view(cache::AbstractCoordinateCache, c::CompiledCoord) = view(cache.z̈, c.cache_idxs)
J_cache_view(cache::AbstractCoordinateCache, c::CompiledCoord) = view(cache.J, c.cache_idxs, :)
f_cache_view(cache::AbstractCoordinateCache, c::CompiledCoord) = view(cache.f, c.cache_idxs)
