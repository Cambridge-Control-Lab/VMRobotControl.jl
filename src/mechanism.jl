############################################
# Mechanism accessors documentation
############################################
_AnySystem = "Union{Mechanism, CompiledMechanism, VirtualMechanismSystem, CompiledVirtualMechanismSystem}"
_Mechanisms = "Union{Mechanism, CompiledMechanism}"
_VMSystems = "Union{VirtualMechanismSystem, CompiledVirtualMechanismSystem}"
_CompiledSystems = "Union{CompiledMechanism, CompiledVirtualMechanismSystem}"
_UncompiledSystems = "Union{Mechanism, VirtualMechanismSystem}"

__add_component_docs = "It is recommended to use [`add_component!`](@ref) to add components to the
mechanism, and [`get_compiled_componentID`](@ref) to get an ID/index for modifying components in compiled systems."

"""
    name(m::$_AnySystem)

Returns the name of the mechanism.
"""
function name end


"""
    frames(m::$_Mechanisms)

Returns a vector of the names of the frames in the mechanism.
"""
function frames end

"""
    num_frames(m::$_Mechanisms)

Returns the number of reference frames in the mechanism.
"""
function num_frames end

"""
    root_frame(m::$_Mechanisms)

Returns the name of the first frame of the mechanism.
This should be the root frame of the tree and should be called "root_frame".
"""
function root_frame end

"""
    joints(m::$_Mechanisms)

Returns joints in the mechanism. 

If `m<:$_UncompiledSystems` this is an OrderedDict. Mutating it WILL mutate the mechanism.
If `m<:$_CompiledSystems`  this is a `TypeStableCollection`. Mutating the returned collection WILL mutate the mechanism.

It is recommended to use [`add_joint!`](@ref) to add joints to the mechanism, and [`get_compiled_jointID`](@ref)
to get an ID/index for modifying joints in compiled systems.
"""
function joints end

"""
    coordinates(m::$_AnySystem)

Returns the coordinates of the system. 

If `m<:$_UncompiledSystems` this is a Dict. Mutating the returned Dict WILL mutate the mechanism.
If `m<:$_CompiledSystems`  this is a `TypeStableCollection`. Mutating the returned collection WILL mutate the mechanism.

It is recommended to use [`add_coordinate!`](@ref) to add coordinates to the mechanism, and 
[`get_compiled_coordID`](@ref) to get an ID/index for modifying coordinates in compiled systems.
"""
function coordinates end

"""
    components(m::$_AnySystem)

Returns the dynamic components of the mechanism (e.g. springs/dampers/masses). 

If `m<:$_UncompiledSystems` this is a Dict. Mutating the returned Dict WILL mutate the mechanism.
If `m<:$_CompiledSystems`  this is a `TypeStableCollection`. Mutating the returned collection WILL mutate the mechanism.

$__add_component_docs
"""
function components end

"""
    config_size(m::$_Mechanisms)

Returns the number of configuration variables in the mechanism (e.g. the number of joint angles),
the length of the joint configuration vector `q`.
"""
function config_size end

"""
    velocity_size(m::$_Mechanisms)

Returns the number of velocity variables in the mechanism (e.g. the number of joint velocities),
the length of the joint velocity vector `q̇`.
"""
function velocity_size end

"""
    ndof(m::$_AnySystem)

The number of degrees of freedom of the system.
"""
function ndof end

"""
    inertances(m::$_AnySystem)

Returns the inertance components of the system.

If `m<:$_UncompiledSystems` this is a Dict formed by filtering the components by `Inertance` type: 
mutating the returned Dict WILL NOT mutate the mechanism.

If `m<:$_CompiledSystems`  this is a `TypeStableCollection` made from a subset of the vectors
in the components TypeStableCollection. Mutating the returned collection WILL mutate the mechanism.

$__add_component_docs
"""
function inertances end

"""
    storages(m::AbstractMechanism)

Returns the storage components from mechanism.

If `m<:$_UncompiledSystems` this is a Dict formed by filtering the components by `Storage` type:
mutating the returned Dict WILL NOT mutate the mechanism.

If `m<:$_CompiledSystems` this is a `TypeStableCollection` made from a subset of the vectors
in the components TypeStableCollection. Mutating the returned collection WILL mutate the mechanism.

$__add_component_docs
"""
function storages end

"""
    dissipations(m::AbstractMechanism)

Returns the dissipation components from mechanism.

If `m<:$_UncompiledSystems` this is a Dict formed by filtering the components by `Dissipation` type:
mutating the returned Dict WILL NOT mutate the mechanism.

If `m<:$_CompiledSystems` this is a `TypeStableCollection` made from a subset of the vectors
in the components TypeStableCollection. Mutating the returned collection WILL mutate the mechanism.

$__add_component_docs
"""
function dissipations end

"""
    generic_components(m::AbstractMechanism)

Returns the generic components from mechanism.

If `m<:$_UncompiledSystems` this is a Dict formed by filtering the components by `GenericComponent` type:
mutating the returned Dict WILL NOT mutate the mechanism.

If `m<:$_CompiledSystems` this is a `TypeStableCollection` made from a subset of the vectors
in the components TypeStableCollection. Mutating the returned collection WILL mutate the mechanism.
"""
function generic_components end

"""
    visuals(m::AbstractMechanism)

Returns the visual components from mechanism.

If `m<:$_UncompiledSystems` this is a Dict formed by filtering the components by `Visual` type:
mutating the returned Dict WILL NOT mutate the mechanism.

If `m<:$_CompiledSystems` this is a `TypeStableCollection` made from a subset of the vectors
in the components TypeStableCollection. Mutating the returned collection WILL mutate the mechanism.

$__add_component_docs
"""
function visuals end

#########################
# Mechanism construction
#########################

"""
    Mechanism{T}(name; with_root_frame=true)

A rigid body 'Mechanism', composed of reference frames, joints, coordinates and 
dynamic components.
"""
struct Mechanism{T}
    name::String
    frames::Vector{String}
    # Must be ordered so that the order of the joints is preserved, i.e. the order of 
    # q, q̇, τ, etc. matches the order of the joints added or (in the urdf/rson file).
    joints::OrderedDict{String, MechanismJoint{<:AbstractJointData{T}, String}} 
    coordinates::Dict{String, CoordinateData}
    components::Dict{String, ComponentData{T}}
    function Mechanism{T}(name::AbstractString; with_root_frame::Bool = true) where T
        m = new{T}(
            name, 
            String[], 
            OrderedDict{String, MechanismJoint{<:AbstractJointData{T}, String}}(),
            Dict{String, CoordinateData}(),
            Dict{String, ComponentData{T}}()
        )
        with_root_frame && add_frame!(m, "root_frame")
        m
    end
    function Mechanism(name::AbstractString; with_root_frame::Bool = true)
        error("Please specify the floating point type of the mechanism, e.g. Mechanism{Float64}('$name')")
    end
end

name(m::Mechanism) = m.name
frames(m::Mechanism) = m.frames
num_frames(m::Mechanism) = length(m.frames)
root_frame(m::Mechanism) = frames(m)[1]
joints(m::Mechanism) = m.joints
coordinates(m::Mechanism) = m.coordinates
components(m::Mechanism) = m.components
Base.eltype(::Type{Mechanism{T}}) where T = T
Base.eltype(m::Mechanism) = eltype(typeof(m))

config_size(m::Mechanism) = reduce((N, joint) -> N + config_size(joint), values(joints(m)); init=0)
velocity_size(m::Mechanism) = reduce((N, joint) -> N + velocity_size(joint), values(joints(m)); init=0)
function ndof(m::Mechanism)
    Nc = config_size(m)
    Nv = velocity_size(m) 
    @assert Nc==Nv
    Nc
end

inertances(m::Mechanism) =          filter(p -> isa(p.second, Inertance),           components(m))
storages(m::Mechanism) =            filter(p -> isa(p.second, Storage),             components(m))
dissipations(m::Mechanism) =        filter(p -> isa(p.second, Dissipation),         components(m))
generic_components(m::Mechanism) =  filter(p -> isa(p.second, GenericComponent),    components(m))
visuals(m::Mechanism) =             filter(p -> isa(p.second, Visual),              components(m))

#######################################
# Virtual Mechanism System
#######################################
"""
    VirtualMechanismSystem{T}(name)
    VirtualMechanismSystem(name, robot::Mechanism{T}, [virtual_mechanism::Mechanism{T}])

Construct a virtual mechanism system. If only a name is provided, then an empty virtual mechanism 
system is created, and the robot/virtual mechanism can be accessed by the `robot` and `virtual_mechanism`
fields.

Coordinates and components can then be added to the virtual mechanism system, and these can act 
inbetween the robot and the virtual mechanism. Coordinates on the robot/virtual mechanism can be 
referenced using the naming syntax `".robot.coordID"` or `".virtual_mechanism.coordID"`.

Note: the robot and virtual mechanism are not copied, so modifying them after creating the virtual 
mechanism system WILL modify the virtual mechanism system.

See also [`Mechanism`](@ref), [`add_coordinate!`](@ref), [`add_component!`](@ref)
"""
struct VirtualMechanismSystem{T}
    name::String
    robot::Mechanism{T}
    virtual_mechanism::Mechanism{T}
    coordinates::Dict{String, CoordinateData}
    components::Dict{String, Union{Storage{T}, Dissipation{T}}}
    function VirtualMechanismSystem{T}(name::String) where T
        robot = Mechanism{T}("$(name)_robot")
        virtual_mechanism = Mechanism{T}("$(name)_virtual_mechanism")
        new{T}(name, robot, virtual_mechanism, Dict(), Dict())
    end
    function VirtualMechanismSystem(name::String, robot::Mechanism{T}) where T
        virtual_mechanism = Mechanism{T}("$(name)_virtual_mechanism")
        new{T}(name, robot, virtual_mechanism, Dict(), Dict())
    end
    function VirtualMechanismSystem(name::String, robot::Mechanism{T}, virtual_mechanism::Mechanism{T}) where T
        new{T}(name, robot, virtual_mechanism, Dict(), Dict())
    end

    function VirtualMechanismSystem(name::String, robot::Mechanism{T}, virtual_mechanism::Mechanism{T}, Nr) where T
        error("Doing a reference in this way is no longer supported")
    end
end

Base.eltype(::Type{VirtualMechanismSystem{T}}) where T = T

name(vms::VirtualMechanismSystem) = vms.name
coordinates(vms::VirtualMechanismSystem) = vms.coordinates
components(vms::VirtualMechanismSystem) = vms.components

storages(vms::VirtualMechanismSystem) = filter(p -> isa(p.second, Storage), components(vms))
dissipations(vms::VirtualMechanismSystem) = filter(p -> isa(p.second, Dissipation), components(vms))

const MechanismOrVMS = Union{Mechanism, VirtualMechanismSystem}

####################
# Mechanism Graph interface
####################

struct MechanismGraphJoint
    src::Int
    dst::Int
    joint::MechanismJoint
end

Graphs.src(e::MechanismGraphJoint) = e.src
Graphs.dst(e::MechanismGraphJoint) = e.dst

function find_frame_index_failloud(m, id)
    idx = find_frame_index(m, id)
    isnothing(idx) && error("Cannot find frame '$id' in mechanism '$(name(m))'. Frames: $(frames(m))")
    idx
end

struct MechanismGraph <: AbstractGraph{Int}
    ne::Int
    nv::Int # Vertices = Frames
    edges::Vector{MechanismGraphJoint} # Edges = Joints
    outneighbors::Vector{Vector{Int}}
    inneighbors::Vector{Vector{Int}}
    function MechanismGraph(m::Mechanism)
        ne = length(joints(m))
        nv = length(frames(m))

        edges = [
            MechanismGraphJoint(
                find_frame_index_failloud(m, j.parentFrameID),
                find_frame_index_failloud(m, j.childFrameID),
                j
            ) 
            for j in values(joints(m))
        ]
        outneighbors = [_frame_outneighbors(edges, i) for i = 1:nv]
        inneighbors = [_frame_inneighbors(edges, i) for i = 1:nv]
        new(ne, nv, edges, outneighbors, inneighbors)
    end
end

function _frame_outneighbors(edges, frameID::Int)
    ret = Vector{Int}()
    for (;src, dst) in edges
        if src == frameID
            push!(ret, dst)
        end
    end
    ret
end

function _frame_inneighbors(edges, frameID::Int)
    ret = Vector{Int}()
    for (;src, dst) in edges
        if dst == frameID
            push!(ret, src)
        end
    end
    ret
end

Graphs.is_directed(::Type{<:MechanismGraph}) = true
Graphs.ne(m::MechanismGraph) = m.ne
Graphs.nv(m::MechanismGraph) = m.nv
Graphs.edges(m::MechanismGraph) = m.edges
Graphs.vertices(m::MechanismGraph) = 1:m.nv
Graphs.inneighbors(m::MechanismGraph, v) = m.inneighbors[v]
Graphs.outneighbors(m::MechanismGraph, v) = m.outneighbors[v]
function Graphs.has_edge(m::MechanismGraph, s::Int, d::Int)
    for e in m.edges
        if e == (s, d)
            return true
        end
    end
    return false
end

"""
    frame_ancestor_matrix(m::Mechanism)

Returns matrix B such that `B[frame_1, frame_2]` ⟹ frame_1 is an ancestor of
frame 2 in the tree
"""
function frame_ancestor_matrix(m::MechanismGraph)
    A = adjacency_matrix(m) # Sparse int matrix
    B = reduce(2:size(A, 1); init=A) do B, i
        B + A^i
    end # Sparse int matrix of zeros or 1s
    Matrix{Bool}(Bool.(B))
end

##############################
# Mechanism Construction
##############################
function _add_frame!(m::Mechanism, id::String)
    _frames = frames(m)
    if id in _frames
        error("Frame '$id' already exists in mechanism '$(name(m))'. Frames: $_frames")
    end
    push!(frames(m), id)
    id
end

"""
    add_frame!(m::Mechanism; id::String)

Add a frame to a mechanism. The name must be unique.

See also [`frames`](@ref), [`add_joint!`](@ref)
"""
function add_frame! end

function add_frame!(m::Mechanism; id::String)
    _add_frame!(m, id)
    return id
end

# Also accept a string as the second argument
function add_frame!(m::Mechanism, id::String)
    _add_frame!(m, id)
    return id
end

"""
    add_joint!(mechanism, jointdata; [parent, child, jointID, check_frames_exist::Bool])

Add a joint to a mechanism. Returns the joint id.

# Arguments
 - `jointID`: the name of the joint, a String. If not provided, a unique jointID is created.
 - `parent`: the name of the parent frame, a String. If not provided, then the default is the most recently added frame.
 - `child` the name of the child frame, a String. If not provided, adds a new frame to the robot. 
 - `check_frames_exist`: if true, then check that the parent and child frame already exist in the 
    mechanism. If false, then this check is skipped. In this case the frames are not added to the 
    mechanism.

# See Also 
To add a frame to a mechanism see [`add_frame!`](@ref). For more information on types of joint
see [`AbstractJointData`](@ref) and [`MechanismJoint`](@ref).
"""
function add_joint! end

function _add_joint!(m::Mechanism, jointdata::AbstractJointData, jointID, parent, child, check_frames_exist)
    if isnothing(parent)
        # If no parent frame is given, append to the frame added most recently
        parent = frames(m)[end]
    end
    if isnothing(child)
        # If no child frame is given, append to the frame added most recently
        frame_name = "$(jointID)_child_frame"
        success, child = _add_frame!(m, frame_name)
        @assert success "Tried to create an anonymous child frame but failed. This mechanism may be messed up now..."
    end
    check_frames_exist && ~(parent in frames(m)) && error("Cannot find parent frame '$parent' in mechanism. Frames: $(frames(m))")
    check_frames_exist && ~(child in frames(m)) && error("Cannot find child frame '$child' in mechanism. Frames: $(frames(m))")
    mj = MechanismJoint{typeof(jointdata), String}(jointdata, parent, child)
    _safe_add_to_dict(joints(m), jointID, mj; errmsg="Joint with id '$jointID' already exists, cannot be added to $(name(m)).")
    jointID
end

# Public API
function add_joint!(
        m::Mechanism, 
        jointdata::AbstractJointData; 
        parent::Union{String, Nothing}=nothing, 
        child::Union{String, Nothing}=nothing, 
        jointID::Union{String, Nothing}=nothing, 
        id::Union{String, Nothing}=nothing,
        check_frames_exist::Bool=true) 
    if isnothing(id) && ~isnothing(jointID)
        @warn "Deprecation warning: `add_joint!(... ; jointID=\"my_joint\", ...)` is deprecated. Use `add_joint!(... ; id=\"my_joint\", ...)` instead."
        _add_joint!(m, jointdata, jointID, parent, child, check_frames_exist)
        return jointID
    elseif ~isnothing(id)
        _add_joint!(m, jointdata, id, parent, child, check_frames_exist)
        return id
    else
        error("Please specify `id` as a keyword argument: e.g. `add_joint!(... ; id=\"my_joint\"), ...`")
    end
end

#############################
# Coordinates
#############################

function _safe_add_to_dict(dict, key, value; errmsg=nothing)
    if haskey(dict, key)
        errmsg = isnothing(errmsg) ? "Key `$key` already exists, cannot be added." : errmsg
        error(errmsg)
    end
    dict[key] = value
    return key
end

function _add_coordinate!(m::MechanismOrVMS, c::CoordinateData, coordID::String)
    _safe_add_to_dict(coordinates(m), coordID, c; errmsg="Coordinate with id `$coordID` already exists, cannot be added to $(name(m)).")
    
    # # Check if dependencies exist, or warn
    # warn(dep) = @warn "Coordinate '$coordID' depends on coordinate '$dep'. There is no coordinate called `$dep` in `$(name(m))`."
    # foreach(dependencies(c)) do dep
    #      if m isa Mechanism
    #         if !(dep in keys(coordinates(m)))
    #             warn(dep)
    #         end
    #     elseif m isa VirtualMechanismSystem
    #         if startswith(".robot.")(dep) 
    #             if !(chopprefix(dep, ".robot.") in keys(coordinates(m.robot)))
    #                 warn(dep)
    #             end
    #         elseif startswith(".virtual_mechanism.")(dep)
    #             if !(chopprefix(dep, ".virtual_mechanism.") in keys(coordinates(m.virtual_mechanism)))
    #                 warn(dep)
    #             end
    #         else # It must be a reference to a coordinate in the VMS
    #             if !(dep in keys(coordinates(m)))
    #                 warn(dep)
    #             end
    #         end
    #     end
    # end
    return coordID
end

"""
    add_coordinate!(m::Mechanism, c::CoordinateData; id::String)
    add_coordinate!(m::VirtualMechanismSystem, c::CoordinateData; id::String)

Add a coordinate to a mechanism or virtual mechanism system. The `coordID` is the name of the 
coordinate. 

See also [`get_compiled_coordID`](@ref), [`configuration`](@ref), [`velocity`](@ref), [`vpa`](@ref), [`jacobian`](@ref)
"""
function add_coordinate!(
        m::MechanismOrVMS, 
        c::CoordinateData
        ;
        id::Union{String, Nothing}=nothing, 
        coordID::Union{String, Nothing}=nothing)
    if isnothing(id) && ~isnothing(coordID)
        @warn "Deprecation warning: `add_coordinate!(m, c; coordID=\"my_coord\")` is deprecated. Use `add_coordinate!(m, c; id=\"my_coord\")` instead."
        _add_coordinate!(m, c, coordID)
        return coordID
    elseif ~isnothing(id)
        _add_coordinate!(m, c, id)
        return id
    else
        error("Please specify `id` as a keyword argument: e.g. `add_coordinate!(m, c; id=\"my_coord\")`")
    end
end

#############################
# Components
#############################

get_all_component_ids(args...) = error("get_all_component_ids deprecated, use `keys(components(m))` instead.")

function _add_component!(m::MechanismOrVMS, c::ComponentData, id::String)
    _safe_add_to_dict(components(m), id, c; errmsg="Component with id `$id` already exists, cannot be added to $(name(m)).")
    return id
end

"""
    add_component!(m::Mechanism, c::ComponentData; id::String)
    add_component!(m::VirtualMechanismSystem, c::ComponentData; id::String)

Add a coordinate to a mechanism or virtual mechanism system. The `coordID` is the name of the 
coordinate. 

See also [`get_compiled_coordID`](@ref), [`configuration`](@ref), [`velocity`](@ref), [`vpa`](@ref), [`jacobian`](@ref)
"""
add_component!(m::MechanismOrVMS, c::ComponentData; id::String) = _add_component!(m, c, id)


#############################
# Other
#############################

empty_components!(m::Mechanism) = error("empty_components! not deprecated. Modify the dict `components(m)` directly.")

function Base.show(io::IO, ::MIME"text/plain", m::Mechanism)
    print(io, "$(ndof(m))DOF Mechanism{$(eltype(m))} \"$(name(m))\" with ")
    pretty_print_length(io, frames(m), "frame"; prefix="", print_on_zero=true)
    pretty_print_length(io, joints(m), "joint")
    pretty_print_length(io, coordinates(m), "coordinate")
    pretty_print_length(io, components(m), "component")
end

function pretty_print_length(io::IO, vec_or_length, name::String; prefix=", ", suffix="", plural="s", print_on_zero=false) 
    N = isa(vec_or_length, Int) ? vec_or_length : length(vec_or_length)
    if N == 0 && print_on_zero
        print(io, "$(prefix)0 $(name)$(plural)$(suffix)")
    elseif N == 1
        print(io, "$(prefix)1 $(name)$(suffix)")
    elseif N > 1
        print(io, "$(prefix)$(N) $(name)$(plural)$(suffix)")
    end
end

function Base.show(io::IO, ::MIME"text/plain", m::VirtualMechanismSystem)
    print(io, "VirtualMechanismSystem '$(name(m))' with robot `$(m.robot.name)`, and virtual mechanism `$(m.virtual_mechanism.name)`.")
    isempty(coordinates(m))         || print(io, "$(length(coordinates(m))) coordinates, ")
    isempty(storages(m))            || print(io, "$(length(storages(m))) storages, ")
    isempty(dissipations(m))        || print(io, "$(length(dissipations(m))) dissipations, ")
end
