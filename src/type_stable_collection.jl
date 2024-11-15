module TypeStableCollections

export
    TypeStableCollection,
    TypeStableIdx,
    extend, 
    extend_type,
    push_return_index!,
    filtertype


"""
    TypeStableCollection{TUPTYPE}

A collection of vectors, where each vector has a different element type. The element types are
stored in a tuple of type `TUPTYPE`.

Allows for type stable operations on the collection, such as `map`, `mapreduce`, `foreach`.

Can be built using the [`extend`](@ref) function to add new/existing types and return an index;
[`push!`] to add types already present; or [`push_return_index!`](@ref) to add types already
present *and* return the index of the newly added element.
"""
struct TypeStableCollection{TUPTYPE}
    tup::TUPTYPE
end

@generated function TypeStableCollection(tup::TUPTYPE) where {N, TUPTYPE<:NTuple{N, Vector{T} where T}}
    # @assert isa(tup, Tuple) "Expected tuple, got $(typeof(tup))"
    for type in TUPTYPE.types
        @assert type <: Vector "Expected vector, got $type"
        ET = eltype(type)
        @assert isconcretetype(eltype(type)) "Can only add concrete types to a component collection, got $ET"
    end
    :(TypeStableCollection{TUPTYPE}(tup))
end

"""
    TypeStableCollection()

Create an empty TypeStableCollection, useful to then use [`extend`](@ref).
"""
TypeStableCollection() = TypeStableCollection(())

Base.length(c::TypeStableCollection) = mapreduce(length, +, c.tup, init=0)

"""
    TypeStableIdx{T}

An index into a [`TypeStableCollection`](@ref).

The index is parameterized by the type of the element it points to, `T`. Therefore it 
can still be used to access an extended collection, even if the type of the collection 
has changed, or if elements have been appended to the collection.
"""
struct TypeStableIdx{T}
    vecidx::Int
    TypeStableIdx{T}(vecidx::Int) where T = new{T}(vecidx)
end

function Base.show(io::IO, ::Type{TypeStableIdx{T}}) where T
    print(io, "TypeStableIdx{$T}")
end


function _get_matching_tupindex(::Type{TypeStableCollection{TUPTYPE}}, ::Type{T}) where {TUPTYPE, T}
    for (i, vectype) in enumerate(TUPTYPE.types)
        if eltype(vectype) == T
            return i
        end
    end
    return nothing
end

@generated function Base.getindex(c::TypeStableCollection{TUPTYPE}, i::TypeStableIdx{T}) where {TUPTYPE, T} 
    tupidx = _get_matching_tupindex(c, T)
    isnothing(tupidx) && error("Type $T not found in TypeStableCollection. Types in collection: $(map(eltype, TUPTYPE.types))")
    return quote
        c.tup[$tupidx][i.vecidx]
    end
end

@generated function Base.setindex!(c::TypeStableCollection, val::T, i::TypeStableIdx{T}) where T
    tupidx = _get_matching_tupindex(c, T)
    isnothing(tupidx) && error("Type $T not found in TypeStableCollection")
    return quote
        c.tup[$tupidx][i.vecidx] = val
        c
    end
end

@generated function filtertype(c::TypeStableCollection{TUPTYPE}, ::Type{T}) where {TUPTYPE, T}
    idxs = [i for (i, type) in enumerate(TUPTYPE.types) if eltype(type) <: T]
    tup_expr = Expr(:tuple, [:(c.tup[$i]) for i in idxs]...)
    Expr(:call, TypeStableCollection, tup_expr)
    # return quote
    #     ret_tup = map($tup_idxs) do idx
    #         c.tup[idx]
    #     end
    #     return TypeStableCollection(ret_tup)
    # end
end



"""
    extend(c::TypeStableCollection, val)

Extend the type stable collection by adding `val`. If the type of `val` is already in
the collection, val is appended to the corresponding vector. If the type is not already
in the collection, a new vector is created, thus changing the type of the collection.

As a result, the type of the collection may change after calling `extend`.
Therefore, to keep the calling code type stable, it is recommended to always 
name the return value of extend with a new variable, and use that variable in 
subsequent code.

Returns: the new collection and a [`TypeStableIdx`](@ref) to the newly added value. 

# Examples
```
julia> c1 = TypeStableCollection(())
TypeStableCollection{Tuple{}}(())
julia> c2, idx = extend(c1, 2)
```
"""
@generated function extend(c::TypeStableCollection{TUPTYPE}, val::T) where {TUPTYPE, T}
    tupidx = _get_matching_tupindex(c, T)
    # If type T is already in collection, do the following
    if ~isnothing(tupidx)
        return quote
            push!(c.tup[$tupidx], val)
            idx = TypeStableIdx{T}(length(c.tup[$tupidx]))
            return c, idx
        end 
    end
    # Else, create a new vector, and insert it into the tuple
    new_obj_id = objectid(T)
    for (i, vectype) in enumerate(TUPTYPE.types)
        obj_id = objectid(eltype(vectype))
        @assert obj_id != new_obj_id "This should be impossible"
        # Tuple ordering is kept consistent to minimize the number of generated functions
        # and therefore compilation. This is done using the objectid of the element type.
        if obj_id < new_obj_id
            return quote
                newvec = T[val]
                newtup = (c.tup[1:$i-1]..., newvec, c.tup[$i:end]...)
                idx = TypeStableIdx{T}(1)
                ret = TypeStableCollection(newtup)
                @assert ret[idx] == val
                return ret, idx
            end
        end
    end
    # If we fall through, we need to add the new type to the end of the tuple
    return quote
        newvec = T[val]
        newtup = (c.tup..., newvec)
        idx = TypeStableIdx{T}(1)
        return TypeStableCollection(newtup), idx
    end
end

@generated function extend_type(c::TypeStableCollection{TUPTYPE}, ::Type{T}) where {TUPTYPE, T}
    tupidx = _get_matching_tupindex(c, T)
    # If type T is already in collection, do the nothing
    if ~isnothing(tupidx)
        return quote
            return c
        end
    end
    # Else, create a new vector, and insert it into the tuple
    new_obj_id = objectid(T)
    for (i, vectype) in enumerate(TUPTYPE.types)
        obj_id = objectid(eltype(vectype))
        @assert obj_id != new_obj_id "This should be impossible"
        # Tuple ordering is kept consistent to minimize the number of generated functions
        # and therefore compilation. This is done using the objectid of the element type.
        if obj_id < new_obj_id
            return quote
                newvec = Vector{T}() # Empty vector of type T
                newtup = (c.tup[1:$i-1]..., newvec, c.tup[$i:end]...)
                return TypeStableCollection(newtup)
            end
        end
    end
    # If we fall through, we need to add the new type to the end of the tuple
    return quote
        newvec = Vector{T}() # Empty vector of type T
        newtup = (c.tup..., newvec)
        return TypeStableCollection(newtup)
    end
end

@generated function Base.push!(c::TypeStableCollection{TUPTYPE}, val::T) where {TUPTYPE, T}
    tupidx = _get_matching_tupindex(c, T)
    if isnothing(tupidx)
        return quote
            error("Cannot push to type stable collection as this type does not exist in the collection. Type: $(typeof(val)). Consider using `extend`.")
        end
    else
        return quote
            push!(c.tup[$tupidx], val)
            return c
        end
    end
end


"""
    push_return_index!(c::TypeStableCollection, val)

Push `val` to the collection `c`. Requires that type of `val` is already in the collection.

Returns: the modified collection and a [`TypeStableIdx`](@ref) to the newly added value.
"""
@generated function push_return_index!(c::TypeStableCollection{TUPTYPE}, val::T) where {TUPTYPE, T}
    # If type C is already in collection, do the following
    for (i, vectype) in enumerate(TUPTYPE.types)
        # Find the index into the tuple of types, of the vector with eltype C
        if eltype(vectype) == T
            # Return a function that pushes the component and returns the same collection
            return quote
                push!(c.tup[$i], val)
                idx = TypeStableIdx{$i}(length(c.tup[$i]))
                return c, idx
            end
        end
    end
    return quote
        error("Cannot push to type stable collection as this type does not exist in the collection. Type: $(typeof(val)). Consider using `extend`.")
    end
end


@generated function Base.mapreduce(f::F, op::OP, c::TypeStableCollection{TUPTYPE}; init) where {F, OP, TUPTYPE}
    expr = Expr(:block)
    for i=1:fieldcount(TUPTYPE)
        push!(expr.args, quote
            ret = mapreduce(f, op, c.tup[$i], init=ret)
        end)
    end
    quote
        ret::typeof(init) = init # Typeassert here ensures type stability of ret
        $expr
        return ret
    end
end

function Base.map(f::F, c::TypeStableCollection) where F
    ret_tuple = map(c.tup) do vec
        map(f, vec)
    end
    TypeStableCollection(ret_tuple)
end

function Base.map!(f::F, c::TypeStableCollection) where F
    foreach(c.tup) do vec
        for i = eachindex(vec)
            vec[i] = f(vec[i])
        end
    end
    c
end

function Base.foreach(f::F, c::TypeStableCollection) where F
    foreach(c.tup) do vec
        foreach(f, vec)
        nothing
    end
    nothing
end

function Base.all(f, c::TypeStableCollection{TUPTYPE}) where TUPTYPE
    expr = Expr(:block)
    for i = 1 : fieldcount(TUPTYPE)
        push!(expr.args, quote
            v = all(f, c.data[$i])
            !v && return false
        end)
    end
    quote
        $expr
        true
    end
end

function Base.isapprox(c::TypeStableCollection{T1}, other::TypeStableCollection{T2}; kwargs...) where {T1, T2}
    T1 == T2 || return false
    length(c) == length(other) || return false
    for i = 1 : fieldcount(T1)
        length(c.tup[i]) == length(other.tup[i]) || return false
        for j = 1 : length(c.tup[i])
            isapprox(c.tup[i][j], other.tup[i][j]; kwargs...) || return false
        end
    end
    return true
end

Base.isempty(c::TypeStableCollection) = length(c) == 0

function Base.show(io::IO, c::TypeStableCollection)
    NTypes = length(c.tup)
    print(io, "TypeStableCollection with $(length(c)) items of $(NTypes) types")
    NTypes > 3 ? println(io, ", 3 shown:") : println(io, ":")
    for i = 1:min(length(c.tup), 3)
        vec = c.tup[i]
        println(io, "  $(length(vec)) $(eltype(vec))")
    end
    if length(c) > 3
        println(io, "  ...")
    end
end

end