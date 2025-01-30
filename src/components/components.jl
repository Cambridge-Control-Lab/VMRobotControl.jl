"""
    ComponentData{T}

Supertype for all mechanical components with eltype `T`. 
"""
abstract type ComponentData{T} end

valuetype(::MC) where {T, MC<:ComponentData{T}} = T
valuetype(::Type{MC}) where {T, MC<:ComponentData{T}} = T

# frame(mc::ComponentData) = mc.frameID
# parent(mc::ComponentData) = mc.parentFrameID
# child(mc::ComponentData) = mc.childFrameID

struct CompiledComponentID{C}
    idx::TypeStableIdx{C}
    function CompiledComponentID(idx::TypeStableIdx{C}) where C<:ComponentData
        new{C}(idx)
    end
end

Base.getindex(tsc::TypeStableCollection, idx::CompiledComponentID) = tsc[idx.idx]
Base.setindex!(tsc::TypeStableCollection, val, idx::CompiledComponentID) = (tsc[idx.idx] = val)

# Virtual Mechanism System Coord Index 
struct VMSComponentID{C, S}
    idx::CompiledComponentID{C}
    VMSComponentID{ON_ROBOT}(idx::CompiledComponentID{C}) where C = new{C, ON_ROBOT}(idx)
    VMSComponentID{ON_VIRTUAL_MECHANISM}(idx::CompiledComponentID{C}) where C = new{C, ON_VIRTUAL_MECHANISM}(idx)
    VMSComponentID{ON_SYSTEM}(idx::CompiledComponentID{C}) where C = new{C, ON_SYSTEM}(idx)
end
_vms_location(::VMSComponentID{T, S}) where {T, S} = Val{S}()


const ComponentID = Union{String, CompiledComponentID, VMSComponentID}



"""
    generalized_force!(τ, bundle::CacheBundle, c::ComponentData)

Using the current state of the the cache, calculates the generalized force due to component `c`,
and adds it to `τ`. 

Typically `τ` is a vector from the cache, but it is kept as an argument to allow for
more flexibility in the use of this function.
"""
function generalized_force! end



"""
    opspace_force(bundle::MechanismCacheBundle, c::ComponentData)

Calulates and returns the operational space force due to component `c`.
"""
function opspace_force(bundle::MechanismCacheBundle, c::ComponentData)
    ret::SVector = _opspace_force(bundle, c)
    @assert eltype(ret) == eltype(bundle) # This check should run at compile time
    ret
end
    

function opspace_force(bundle::MechanismCacheBundle, componentID::CompiledComponentID)
    component = bundle[componentID]
    ret::SVector = _opspace_force(bundle, component)
    @assert eltype(ret) == eltype(bundle) # This check should run at compile time
    ret
end

function opspace_force(bundle::VMSCacheBundle, componentID::VMSComponentID)
    component = bundle[componentID]
    subbundle = get_vms_subcache(bundle, componentID)
    ret::SVector = _opspace_force(subbundle, component)
    @assert eltype(ret) == eltype(bundle) # This check should run at compile time
    ret
end


"""
    stored_energy(bundle::MechanismCacheBundle, c::ComponentData)

Computes the stored energy of component `c`, kinetic and potential.
"""
function stored_energy(cache::CacheBundle, c::ComponentData)
    T_out = eltype(cache)
    E::T_out = _stored_energy(cache, c)
    E
end



"""
    inertance_matrix!(M, bundle::CacheBundle, c::ComponentData)

Using the current state of the cache, calculates the inertance matrix due to component `c`,
and adds it to `M`. 

Typically `M` is a matrix from the cache, but it is kept as an argument to allow for more
flexibility in the use of this function.
"""
function inertance_matrix! end

###########################
# Reassigning frames/joints
###########################
#       Components are immutable, so we need a way to change the frame they are
#   attached to when compiling the mechanism. This is done with the `reassign_frames`
#   function, and the use of 'remake'

struct FrameMissingException{K, V} <:Exception 
    key::K
    framenames::V
end
struct JointMissingException{K, V} <:Exception 
    key::K
    jointnames::V
end
struct CoordMissingException{K, V} <:Exception 
    key::K
    coordnames::V
end
struct ComponentMissingException{K, V} <:Exception 
    key::K
    componentnames::V
end


function Base.showerror(io::IO, e::FrameMissingException)
    print(io, "Frame not defined: \"$(e.key)\". Choose from: ")
    println(io, join(map(name -> "\n\t\"$(name)\"", e.framenames), ", "))
end
function Base.showerror(io::IO, e::JointMissingException)
    print(io, "Joint not defined: \"$(e.key)\". Choose from: ")
    println(io, join(map(name -> "\n\t\"$(name)\"", e.jointnames), ", "))
end
function Base.showerror(io::IO, e::CoordMissingException)
    print(io, "Coordinate not defined: \"$(e.key)\".\nChoose from: ")
    println(io, join(map(name -> "\n\t\"$(name)\"", e.coordnames), ", "))
end
function Base.showerror(io::IO, e::ComponentMissingException)
    print(io, "Component not defined: \"$(e.key)\".\nChoose from: ")
    println(io, join(map(name -> "\n\t\"$(name)\"", e.componentnames), ", "))
end

function _get_with_exception_if_missing(c, dict::AbstractDict, ::Type{E}) where {E<:Exception}
    haskey(dict, c) || throw(E(c, sort(collect(keys(dict)))))
    dict[c]
    # try 
    #     ret = dict[c]
    # catch e
    #     names = sort(collect(keys(dict)))
    #     isa(e, KeyError) && throw(E(e.key, names))
    #     rethrow(e)
    # end
end

"""
    reassign_frames(c, frame_replacement_dict)

For joint/coordinate/component `c` replace all references to any `frame` with 
`frame_replacement_dict[frame]`. Will be recursively applied to fields of the
component, such as coordinates.
"""
function reassign_frames(c, frameID_map::AbstractDict)
    _reassign_frames(c, frameID_map)
end

function get_compiled_frameID(frameID_map::AbstractDict, frame)
    _get_with_exception_if_missing(frame, frameID_map, FrameMissingException)
end


"""
    reassign_joints(c, joint_replacement_dict)

For joint/coordinate/component `c` replace all references to any joint `jointid` with 
`joint_replacement_dict[jointid]`. Will be recursively applied to fields of the
component, such as coordinates.
"""
function reassign_joints(c, joint_map::AbstractDict)
    _reassign_joints(c, joint_map)
end

function get_compiled_joint(joint_map::AbstractDict, joint)
    _get_with_exception_if_missing(joint, joint_map, JointMissingException)
end

function get_compiled_jointID(joint_map::AbstractDict, joint)
    _get_with_exception_if_missing(joint, joint_map, JointMissingException)
end


"""
    reassign_coords(c, coord_replacement_dict)

For joint/coordinate/component `c`  all references to any coordinate id using
the replacement dictionary.
"""
function reassign_coords(c, coordID_map::Dict)
    # _get_with_exception_if_missing(c, coordID_map, _reassign_coords, CoordMissingException)
    _reassign_coords(c, coordID_map)
end

function get_compiled_coordID(coordID_map::Dict, coord_id)
    _get_with_exception_if_missing(coord_id, coordID_map, CoordMissingException)
end

# Components
function get_compiled_componentID(componentID_map::Dict, component_id)
    _get_with_exception_if_missing(component_id, componentID_map, ComponentMissingException)
end

##########################

@generated function struct_as_namedtuple(st)
    A = (Expr(:(=), n, :(st.$n)) for n in setdiff(fieldnames(st), (:kwargs,)))
    Expr(:tuple, A...)
end

Base.@pure __parameterless_type(T) = Base.typename(T).wrapper
parameterless_type(x) = parameterless_type(typeof(x))
parameterless_type(x::Type) = __parameterless_type(x)

remaker_of(c::ComponentData) = parameterless_type(c)

"""
    remake(thing; <keyword arguments>)

Re-construct `thing` with new field values specified by the keyword
arguments. Must have a keyword-constructor.

Adapted from SciMLBase
"""
function remake(thing; kwargs...)
    T = remaker_of(thing)
    if :kwargs ∈ fieldnames(typeof(thing))
        if :kwargs ∉ keys(kwargs)
            T(; struct_as_namedtuple(thing)..., thing.kwargs..., kwargs...)
        else
            T(; struct_as_namedtuple(thing)..., kwargs[:kwargs]...)
        end
    else
        T(; struct_as_namedtuple(thing)..., kwargs...)
    end
end


#################################
# Implementations for 'coord' components
#################################
# Strategy:
#   If there is a specialized method for a particular component, this will be 
#   used.
#
#   However, most components are if they have a field `coord`. These components
#   require identical behaviour for many functionsIf there is no specialized 
#   method, we check for a field `coord`, and  appply reassign_frames to that field.
#
#   This should not come with any runtime cost, because the compiler should be
#   able to ignore the if statement as fieldnames is known at compile time.


#   WARNING: if a frame does not implement `reassign_frames`, has field coord,
#   and references any frames in another field, this will fail silently, but 
#   probably cause an error later!
function _reassign_frames(c::C, frame_replacement_dict) where C<:ComponentData
    if :coord ∈ fieldnames(C)
        # remake(c; coord=_reassign_frames(c.coord, frame_replacement_dict))
        c
    else
        error("Not implemented. `reassign_frames` is not implemented for component '$c'.")
    end
end

function _reassign_joints(c::C, joint_replacement_dict) where C<:ComponentData
    if :coord ∈ fieldnames(C)
        # remake(c; coord=_reassign_joints(c.coord, joint_replacement_dict))
        c
    else
        error("Not implemented. `reassign_joints` is not implemented for component '$c'.")
    end
end

function _reassign_coords(c::C, coord_replacement_dict) where C<:ComponentData
    if :coord ∈ fieldnames(C)
        remake(c; coord=get_compiled_coordID(coord_replacement_dict, c.coord))
    else
        error("Not implemented. `reassign_coords` is not implemented for component '$c'.")
    end
end


"""
    _jacobian_transpose_times_force!(τ, J, F)

Calculate the product of the transpose of the Jacobian `J` and the force `F`, and *add it* to `τ`.
"""
@inline function _add_jacobian_transpose_times_force!(τ, J, F)
    Nf = length(F)
    Nτ = length(τ)
    @assert Nτ == size(J, 2)
    @assert Nf == size(J, 1)
    for i = 1:Nτ
        τ_i = zero(eltype(τ))
        for j = 1:Nf
            τ_i += J[j, i] * F[j]
        end
        τ[i] += τ_i
    end
    τ
end

# Generic implementations for _generalized_force if _opspace_force is defined
function generalized_force!(τ, cache, c::C) where C<:ComponentData
    @assert hasfield(C, :coord) "Component '$c' does not have a field 'coord'. If this is a valid component, please implement `generalized_force!` for it."
    coord = c.coord
    if cache isa MechanismCacheBundle
        J = _jacobian(cache, c.coord)
        F = _opspace_force(cache, c)
        # F is negated, due to the definition of bias forces. e.g. If a spring opspace force is Kz
        # then the generalized/bias force we need to apply is -kz
        _add_jacobian_transpose_times_force!(τ, J, -F)
    elseif cache isa VirtualMechanismSystemCacheBundle
        @assert isa(coord, VMSCoordID) "The provided coordinate is not a VMSCoordID, but the cache is a VirtualMechanismSystemCacheBundle. This is a bug."
        # F is the generalized force in the operational space
        F = _opspace_force(cache, c)
        # F is negated, due to the definition of bias forces. e.g. If a spring opspace force is Kz
        # then the generalized/bias force we need to apply is -kz
        _vm_add_jacobian_transpose_times_force!(τ, cache, coord, -F)    
    else
        error("Unknown cache type: $(typeof(cache))")
    end
end

# Generic implementation for generalized_force! given a coord_id and force
function _vm_add_jacobian_transpose_times_force!(τ, cache::VirtualMechanismSystemCacheBundle, coord_id::C, F::SVector) where C<:VMSCoordID
    J = _jacobian(cache, coord_id)
    loc = _vms_location(coord_id)
    Nr, Nv = ndof(cache.vms.robot), ndof(cache.vms.virtual_mechanism)
    if loc == Val{ON_ROBOT}()
        _add_jacobian_transpose_times_force!(τ[1], view(J, :, 1:Nr), F)
    elseif loc == Val{ON_VIRTUAL_MECHANISM}()
        _add_jacobian_transpose_times_force!(τ[2], view(J, :, 1:Nv), F)
    elseif loc == Val{ON_SYSTEM}()
        _add_jacobian_transpose_times_force!(τ[1], view(J, :, 1:Nr), F)
        _add_jacobian_transpose_times_force!(τ[2], view(J, :, Nr+1:Nr+Nv), F)
    else
        error("Unknown location: $loc")
    end
    τ
end

"""
    _add_opspace_force!(cache, c::CompiledCoord)

Add opspace force `f` to the cache for coordinate `c`.
"""
@inline function _add_opspace_force!(bundle::CacheBundle, c::C) where C<:ComponentData
    @assert hasfield(C, :coord) "Component '$c' does not have a field 'coord'. If this is a valid component, please implement `_add_opspace_force!` for it."
    f::SVector = opspace_force(bundle, c)
    coord_id = c.coord
    coord = bundle[coord_id]
    f_cache_view(bundle, coord) .-= f # f is negated, due to the definition of bias forces. e.g. If a spring opspace force is Kz
    nothing
end

function _opspace_force(cache::CacheBundle, c::ComponentData)
    error("Not implemented. `_opspace_force` is not implemented for component '$c'.")
end

function inertance_matrix!(M::Matrix, cache::CacheBundle, c::ComponentData)
    error("Not implemented. `_inertance_matrix` is not implemented for component '$c'.")
end

function _stored_energy(cache::CacheBundle, c::ComponentData)
    error("Not implemented. `_stored_energy` is not implemented for component '$c'.")
end


#####################################################

struct Visual{G, FID} <: ComponentData{Float64}
    frame::FID
    geometry::G
    color::RGBAf
    specular::Float32
    shininess::Float32
    function Visual(frame, geometry; color=RGBAf(1.0f0, 0.0f0, 1.0f0, 1.0f0), specular=0.2f0, shininess=32.0f0)
        if isa(geometry, GeometryBasics.GeometryPrimitive)
            _mesh = GeometryBasics.normal_mesh(geometry)
        elseif isa(geometry, GeometryBasics.Mesh)
            _mesh = geometry
        else
            error("Invalid geometry type: $(typeof(geometry))")
        end
        new{typeof(_mesh), typeof(frame)}(frame, _mesh, color, specular, shininess)
    end
    
    function Visual(name, color, geometry)
        # legacy constructor
        Visual(name, geometry; color=color)
    end
    
    function Visual(name, frame, color, geometry)
        @warn "Visual(name, frame, color, geometry) is deprecated. Use Visual(frame, color, geometry) instead."
        Visual(frame, color, geometry)
    end
    # Keyword constructor
    Visual(; frame, geometry, color, specular, shininess) = Visual(frame, geometry; color=color, specular=specular, shininess=shininess)
end

function Base.:*(tf::Transform, v::Visual)
    Visual(v.frame, v.color, VMRobotControl.transform_mesh(v.geometry, tf))
end


_reassign_frames(c::Visual, frd) = remake(c; frame=get_compiled_frameID(frd, c.frame))
_reassign_joints(c::Visual, jrd) = c
_reassign_coords(c::Visual, crd) = c
