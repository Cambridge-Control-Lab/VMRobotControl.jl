##############################################
# Computation helpers for collections of components
##############################################

valuetype(tsc::TypeStableCollection) = mapreduce(valuetype, promote_type, tsc)
stored_energy(::C, ::TypeStableCollection{Tuple{}}) where C<:MechanismCacheBundle = zero(eltype(C))

function is_approx_hermitian(A::AbstractMatrix)
    indsm, indsn = axes(A)
    if indsm != indsn
        return false
    end
    for i = indsn, j = i:last(indsn)
        if A[i,j] != adjoint(A[j,i])
            return false
        end
    end
    return true
end

function inertance_matrix!(M::Matrix, cache::MechanismCacheBundle, tsc::TypeStableCollection)
    @assert eltype(M) == eltype(cache)
    @assert size(M) == (ndof(cache), ndof(cache))
    foreach(c->inertance_matrix!(M, cache, c), tsc)
    M
end

function generalized_force!(τ::Vector, cache::MechanismCacheBundle, tsc::TypeStableCollection)
    @assert length(τ) == ndof(cache)
    @assert eltype(τ) == eltype(cache)
    foreach(c->generalized_force!(τ, cache, c), tsc)
    τ
end


function generalized_force!(τ::Tuple, cache::VirtualMechanismSystemCacheBundle, tsc::TypeStableCollection)
    @assert eltype(τ[1]) == eltype(cache)
    @assert length(τ[1]) == ndof(cache.vms.robot)
    @assert eltype(τ[2]) == eltype(cache)
    @assert length(τ[2]) == ndof(cache.vms.virtual_mechanism)
    foreach(c->generalized_force!(τ, cache, c), tsc)   
end


function stored_energy(cache::CacheBundle, tsc::TypeStableCollection)
    init = zero(eltype(cache))
    mapreduce(c->stored_energy(cache, c), +, tsc; init)
end

_reassign_joints(cc::TypeStableCollection, jrd) = map(c -> _reassign_joints(c, jrd), cc)
_reassign_frames(cc::TypeStableCollection, frd) = map(c -> _reassign_frames(c, frd), cc)
_reassign_coords(cc::TypeStableCollection, crd) = map(c -> _reassign_coords(c, crd), cc)