module TransformsHessiansExt

using ..Transforms
using ..Hessians

using ForwardDiff: Dual
using StaticArrays

#################################
# ForwardDiff/Hessians interface for transforms 
#################################

function Hessians.value(::Type{T}, r::Rotor) where T
    Rotor(Hessians.value(T, bivector(r)), Hessians.value(T, scalar(r)), Val(false)) # DONT NORMALIZE
end
function Hessians.value(::Type{T}, tf::Transform) where T
    Transform(Hessians.value(T, origin(tf)), Hessians.value(T, rotor(tf)))
end
function Hessians.value(::Type{T}, tf::TransformJacobian) where T
    TransformJacobian(Hessians.value(T, origin(tf)), Hessians.value(T, rotor(tf)))
end
function Hessians.value(::Type{T}, tfs::SVector{N, <:Transform}) where {T, N}
    map(tf -> Hessians.value(T, tf), tfs)
end
function Hessians.value(::Type{T}, tfs::SVector{N, <:TransformJacobian}) where {T, N}
    map(tf -> Hessians.value(T, tf), tfs)
end

function Hessians.extract_jacobian(::Type{T}, r::Rotor{<:Dual}, x) where T 
    extract_jacobian(T, rotor_to_svector(r), x)
end
function Hessians.extract_jacobian(::Type{T}, tf::Transform{<:Dual}, x::S) where {T, S<:StaticArray}
    Jo = extract_jacobian(T, origin(tf), x)
    Jr = extract_jacobian(T, rotor(tf), x)
    TransformJacobian(Jo, Jr)
end
function Hessians.extract_jacobian(::Type{T}, tf::TransformJacobian{<:Dual}, x::S) where {T, S<:StaticArray}
    Jo = extract_jacobian(T, tf.J_origin, x)
    Jr = extract_jacobian(T, tf.J_rotor, x)
    TransformHessian(Jo, Jr)
end

# Extract jacobian from vector of transforms (kinematics output)
# Simply forwards to extract jacobian for each frame
function Hessians.extract_jacobian(::Type{T}, tfs::SVector{N, <:Transform}, x::S) where {T, N, S<:StaticArray}
    map(tf -> extract_jacobian(T, tf, x), tfs)
end
function Hessians.extract_jacobian(::Type{T}, tfs::SVector{N, <:TransformJacobian}, x::S) where {T, N, S<:StaticArray}
    map(tf -> extract_jacobian(T, tf, x), tfs)
end

end