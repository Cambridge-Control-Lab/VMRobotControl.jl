module Hessians
"""
A module for computing the jacobians/hessians of functions working with SVectors
"""
# Exports
export hessian, hessian_vector_product, hessian_quadratic_form, extract_jacobian

# Usings
using StaticArrays
using ForwardDiff: ForwardDiff
using ForwardDiff: DiffResults, valtype, partials, Tag

const ForwardDiffStaticArraysExt = Base.get_extension(ForwardDiff, :ForwardDiffStaticArraysExt)
const dualize = ForwardDiffStaticArraysExt.dualize

# Imports
import ForwardDiff: value, hessian

################################################################################

struct HessiansTagType end
HessiansTag(::AbstractArray{T}) where T  = typeof(Tag(HessiansTagType(), T))

@generated function extract_jacobian(::Type{T}, ydual::StaticArray, x::S) where {T,S<:StaticArray}
    M, N = length(ydual), length(x)
    shape = size(ydual)
    result = Expr(:tuple, [:(partials(T, ydual[$i], $j)) for i in 1:M, j in 1:N]...)
    return quote
        $(Expr(:meta, :inline))
        V = StaticArrays.similar_type(S, valtype(eltype($ydual)), Size($shape..., $N))
        return V($result)
    end
end

tagvaltype(::Type{ForwardDiff.Tag{F, V}}) where {F, V} = V

function jacobian(f::F, x::SVector) where F
    T = HessiansTag(x)
    d1 = dualize(T, x)
    yd1 = f(d1)

    # TODO debug intermitent problem with dual types being misordered
    # 
    #    I really don't know what happened with this but the sympton was julia
    # freezing (I think during type inference), or crashing, or giving a 
    # stack overflow. I think that this has dissapeared because I no longer
    # use ForwardDiff to compute jacobia, and therefore don't nest Dual types
    # any more.
    #
    # T2 = ForwardDiff.tagtype(eltype(typeof(yd1)))    
    # println(T)
    # println(ForwardDiff.tagcount(T))
    # println(T2)
    # println(ForwardDiff.tagcount(T2))
    # T2_alt = ForwardDiff.tagtype(tagvaltype(T))
    # println(T2_alt)
    # println(ForwardDiff.tagcount(T2_alt))
    # if ForwardDiff.tagcount(T) < ForwardDiff.tagcount(T2)
    #     throw(ErrorException("BAD"))
    # else
    #     throw(ErrorException("Maybe you fixed it?"))
    # end
    
    val = value(T, yd1)
    grad = extract_jacobian(T, yd1, x)
    result = DiffResults.ImmutableDiffResult(val, (grad, ))
    return result
end

function value(::Type{T}, ys::AbstractArray) where T
    map(y -> value(T, y), ys)
end

function my_hessian(f::F, x::SVector) where F
    T = HessiansTag(x)
    d1 = dualize(T, x)
    d2 = dualize(T, d1)
    yd2 = f(d2)
    yd1 = value(T, yd2)
    val = value(T, yd1)
    dgrad = extract_jacobian(T, yd2, x)
    grad = value(T, dgrad)
    hess = extract_jacobian(T, dgrad, x)
    result = DiffResults.DiffResult(val, (grad, hess))
    return result
end

##########################

function hessian_vector_product(A::SArray{Tuple{N, M, M}, T1}, 
    x::SVector{M, T2}) where {N, M, T1, T2}
    # Utility useful for multiplication with hessians. E.g. in robotics, compute
    # the velocity-product vector from the hessian of the hamiltonian (i.e.
    # the christoffell symbols)
    # vₖ,ᵢ = Σⱼ Aₖᵢⱼ xⱼ

    # B = sum(reshape(x, (1, 2, 1)) .* A; dims=2)
    # C = sum(reshape(x, (1, 1, 2)) .* B, dims=3)
    # SVector{N}(C)
    out = A[:, :, 1] * x[1]
    for i = 2:M
        out += A[:, :, i] * x[i]
    end
    out
end


function hessian_quadratic_form(A::SArray{Tuple{N, M, M}, T1}, 
                                x::SVector{M, T2}) where {N, M, T1, T2}
    # Utility useful for multiplication with hessians. E.g. in robotics, compute
    # the velocity-product vector from the hessian of the hamiltonian (i.e.
    # the christoffell symbols)
    # vₖ = ΣᵢΣⱼ Aₖᵢⱼ xᵢ xⱼ
    
    # B = sum(reshape(x, (1, 2, 1)) .* A; dims=2)
    # C = sum(reshape(x, (1, 1, 2)) .* B, dims=3)
    # SVector{N}(C)
    
    StaticArrays.sacollect(
        SVector{N, promote_type(T1, T2)},
        (sum(A[k, i, j] * x[i] * x[j] for i in 1:M, j in 1:M) for k = 1:N)
    )
end

##########################


# hessian(v -> SVector(v[1]^3, v[2]^2), SVector(1.0, 1.0))

end