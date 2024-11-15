abstract type AbstractCoordinateCache{T} end

struct CoordKinematicsCache{T} <: AbstractCoordinateCache{T}
    z::Vector{T}
    function CoordKinematicsCache{T}(Nc) where T
        new{T}(Vector{T}(undef, Nc))
    end
end

struct CoordJacobiansCache{T} <: AbstractCoordinateCache{T}
    z::Vector{T}
    J::Matrix{T}
    function CoordJacobiansCache{T}(Nc, NDOF) where T
        new{T}(
            Vector{T}(undef, Nc),
            Matrix{T}(undef, Nc, NDOF)
        )
    end
end

struct CoordRBStatesCache{T} <: AbstractCoordinateCache{T}
    z::Vector{T}
    ż::Vector{T}
    z̈::Vector{T}
    function CoordRBStatesCache{T}(Nc) where T
        new{T}(
            Vector{T}(undef, Nc),
            Vector{T}(undef, Nc),
            Vector{T}(undef, Nc)
        )
    end
end

struct CoordRBStatesJacobianCache{T} <: AbstractCoordinateCache{T}
    z::Vector{T}
    ż::Vector{T}
    z̈::Vector{T}
    J::Matrix{T}
    function CoordRBStatesJacobianCache{T}(Nc, NDOF) where T
        new{T}(
            Vector{T}(undef, Nc),
            Vector{T}(undef, Nc),
            Vector{T}(undef, Nc),
            Matrix{T}(undef, Nc, NDOF)
        )
    end
end

struct CoordRBStatesWrenchesCache{T} <: AbstractCoordinateCache{T}
    z::Vector{T}
    ż::Vector{T}
    z̈::Vector{T}
    f::Vector{T}
    function CoordRBStatesWrenchesCache{T}(Nc) where T
        new{T}(
            Vector{T}(undef, Nc),
            Vector{T}(undef, Nc),
            Vector{T}(undef, Nc),
            Vector{T}(undef, Nc)
        )
    end
end


cache_size(c::AbstractCoordinateCache) = length(c.z)
