"""
    mutable struct RSONParserConfig(; kwargs...)

Configuration for the RSON parser.

# Fields
## Options
- `element_type::Type`: Type of the elements in the mechanism. Default is `Float64`.
- `parse_visuals::Bool`: Whether to parse visuals. Default is `true`.
- `error_on_not_supported::Bool`: Whether to throw an error if an unsupported feature is encountered. Default is `false`.
- `error_on_not_recognized::Bool`: Whether to throw an error if an unrecognized key is encountered. Default is `true`.
- `suppress_warnings::Bool`: Whether to suppress warnings. Default is `false`.
- `default_material::RGBA`: Default material for visuals. Default is `RGBA(0.8, 0.8, 0.8, 1.0)`.

## Fields used by the parser
- `rson_path::String`: Path to the RSON file parsed. (Used to locate meshes)
- `rson_version::Tuple{Int, Int, Int}`: Version of the RSON file parsed.
- `named_materials::Dict{String, RGBA}`: Named materials for visuals. Default is an empty dictionary.

## Internal
- `name_stack::Vector{String}`: Stack of names for error messages.
"""
@kwdef mutable struct RSONParserConfig    
    # Options
    rson_path::Union{Nothing, String}=nothing
    element_type::Type=Float64
    parse_visuals::Bool=true
    error_on_not_supported::Bool=false
    error_on_not_recognized::Bool=true
    suppress_warnings::Bool=false
    default_material::RGBA=RGBA(0.8, 0.8, 0.8, 1.0)
     
    # State
    # Used in parsing, and also `returned' by the parser, in the config after parsing
    rson_version::Union{Nothing, Tuple{Int, Int, Int}}=nothing
    named_materials::Dict{String, RGBA}=Dict()

    # Stack of names for error messages
    name_stack::Vector{String} = [] 
end

function reset_rson_parser_config_state!(cfg)
    cfg.rson_version = nothing
    cfg.named_materials = Dict()
    empty!(cfg.name_stack)
    nothing
end

################################################
# Exception type
################################################
struct RSONParserException <: Exception 
    msg::String
    rson_path::Union{Nothing, String}
    name_stack::Vector{String}
end

function Base.showerror(io::IO, e::RSONParserException)
    print(io, "Error while parsing RSON")
    if isnothing(e.rson_path)
    else
        print(io, " in file '$(e.rson_path)'")
    end
    
    names = map(name -> "\"$name\"", e.name_stack)
    if isempty(names)
        println(io, ".")
    else
        print(io, " in ")
        paren_count = 0
        for name in names
            paren_count > 0 && print(io, " :")
            print(io, "{$name")
            paren_count += 1
        end
        for i in 1:paren_count
            print(io, "}")
        end
        println(io, ".")
    end
    println(io, "\n")
    println(io, e.msg)
end

@inline function rson_throw_exception(msg, cfg::RSONParserConfig)
    ex = RSONParserException(msg, cfg.rson_path, deepcopy(cfg.name_stack))
    throw(ex)
end

function with_name_stack(f, name::String, cfg::RSONParserConfig)
    push!(cfg.name_stack, name)
    ret = f()
    pop!(cfg.name_stack)
    ret
end

################################################
# Parsing util type/functions
################################################

function check_type(field, val, expected_type::Type{T}, cfg::RSONParserConfig) where T
    if isa(val, expected_type)
        return val
    elseif expected_type == JSON_FLOAT && isa(val, JSON_INT)
        # Special case Int to Float conversion
         return JSON_FLOAT(val)
    else
        rson_throw_exception("Expected field '$(field)' to be of type '$(expected_type)', got a '$(typeof(val))'.", cfg)
    end
end

function expectField(
        data::JSON_OBJECT, 
        field::JSON_STR, 
        expect_type::Type{T}, 
        cfg::RSONParserConfig
        ; pop=true)::T where T
    if haskey(data, field)
        ret = pop ? pop!(data, field) : data[field]
        return check_type(field, ret, expect_type, cfg)
    else
        msg = "Expected field '$(field)'"
        if ~isempty(cfg.name_stack)
            msg *= " in '$(cfg.name_stack[end])'"
        end
        msg *= "."
        rson_throw_exception(msg, cfg)
    end
end

function getField(
        data::JSON_OBJECT, 
        field::JSON_STR,
        expect_type::Type{T}, 
        cfg::RSONParserConfig;
        default=nothing) where T
    if haskey(data, field)
        ret = pop!(data, field)
        return check_type(field, ret, expect_type, cfg)
    end
    default
end

function check_for_remaining_fields(data::JSON_OBJECT, cfg::RSONParserConfig)
    all_keys = keys(data) # In normal operation this is
    # a very small list, so we don't need to worry about performance
    unexcepted_keys = filter(k -> !in(k, (S_EXTRA, S_DESCRIPTION)), all_keys)
    if !isempty(unexcepted_keys)
        unexpected_keys_string = join( ("\"$k\"" for k in unexcepted_keys) , ", ")
        if cfg.error_on_not_recognized
            @assert length(cfg.name_stack) > 0 # This should never happen. If it does
            # it means we are checking for remaining fields in the root object
            # or we have put `check_for_remaining_fields` in the wrong place.
            rson_throw_exception("Unexpected field(s) when parsing \"$(cfg.name_stack[end])\": [$unexpected_keys_string].", cfg)
        elseif !cfg.suppress_warnings
            @warn "Unexpected field(s) when parsing \"$(cfg.name_stack[end])\": [$unexpected_keys_string]. Ignoring. Set error_on_not_recognized=true in the RSON parser config to throw an error and get a stacktrace."
        end
    end
end

################################################
# Entry points
################################################
"""
    parseRSON(filepath::AbstractString, [cfg::RSONParserConfig])

Parse a `Mechanism` or `VirtualMechanismSystem` from an RSON file (with a .rson extension).

See also [`parseRSONString`](@ref), [`serializeRSON`](@ref), [`RSONParserConfig`](@ref).
"""
parseRSON(filepath::AbstractString, cfg=RSONParserConfig()) = parseRSONfile(filepath, cfg)

function parseRSONfile(
        filepath::AbstractString,
        cfg::RSONParserConfig=RSONParserConfig()
    )
    dict = JSON.parsefile(filepath; use_mmap=false, dicttype=OrderedDict)
    reset_rson_parser_config_state!(cfg)
    cfg.rson_path = filepath
    _parseRSON(dict, cfg)
end

"""
    parseRSONString(rsonstr::AbstractString, [rsonpath::AbstractString], [cfg::RSONParserConfig])

Parse a `Mechanism` or `VirtualMechanismSystem` from an RSON string. If `rsonpath` is not provided, it
defaults to ".", which is used for relative paths to locate associated files such as meshes for 
visuals.
"""
function parseRSONString(
        rsonstr::AbstractString, 
        cfg::RSONParserConfig=RSONParserConfig()
    )
    dict = JSON.parse(rsonstr, dicttype=OrderedDict)
    reset_rson_parser_config_state!(cfg)
    _parseRSON(dict, cfg)
end

function parseRSONDict(
        json_dict::AbstractDict,
        cfg::RSONParserConfig=RSONParserConfig()
    )
    reset_rson_parser_config_state!(cfg)
    _parseRSON(json_dict, cfg)
end

function parseRSON(
        json_dict::AbstractDict, 
        rsonpath,
        cfg::RSONParserConfig=RSONParserConfig()
    )
    reset_rson_parser_config_state!(cfg)
    cfg.rson_path = rsonpath
    _parseRSON(json_dict, cfg)
end

################################################
# Main parsing function
################################################
function _parseRSON(data, cfg::RSONParserConfig)
    T = cfg.element_type
    T <: Real || rson_throw_exception("element_type must be a subtype of Real, not '$(cfg.element_type)'", cfg)
    try
        version_string = expectField(data, "rson_version", String, cfg)
        parseRSONVersion(version_string, cfg)
        type = expectField(data, S_TYPE, JSON_STR, cfg; pop=false)
        if type == S_MECHANISM
            parse_mechanism(data, T, cfg)
        elseif type == S_VIRTUAL_MECHANISM_SYSTEM
            parse_vms(data, T, cfg)
        else
            rson_throw_exception("Unrecognized type '$(type)'", cfg)
        end
    catch e
        if isa(e, RSONParserException)
            rethrow(e)
        else
            rson_throw_exception("Unexcpected error while parsing RSON.", cfg)
        end
    end
end

function parse_mechanism(data, ::Type{T}, cfg) where T
    if data isa JSON_STR # Then this is a path to another .rson file.
        new_cfg = deepcopy(cfg) # Make a copy to avoid mutating the original config
        path = joinpath(dirname(new_cfg.rson_path), data)
        mechanism = parseRSONfile(path, new_cfg)
        (mechanism isa Mechanism) || rson_throw_exception("When parsing \"$path\", expected a 'Mechanism', but got a '$(typeof(mechanism))'", cfg)
        mechanism
    elseif data isa JSON_OBJECT
        name = expectField(data, S_NAME, JSON_STR, cfg)
        with_name_stack(name, cfg) do
            version_string = getField(data, "rson_version", JSON_STR, cfg; default=nothing) # 

            mechanism = Mechanism{T}(name; from_path=cfg.rson_path, with_root_frame=false)
            _type = expectField(data, S_TYPE, JSON_STR, cfg)
            @assert _type == S_MECHANISM # This should always be the case
            parseOptionalArray!((mechanism, ), data, S_FRAMES, parseNewFrame!, cfg)
            parseOptionalNamedElements!((mechanism, ), data, S_JOINTS, parseJoint!, cfg)
            parseOptionalNamedElements!((mechanism, ), data, S_COORDINATES, _parseCoordDefinition!, cfg)
            if cfg.parse_visuals
                parseOptionalNamedElements!((mechanism, ), data, S_MATERIALS, parseMaterialEntry!, cfg)
                parseOptionalNamedElements!((mechanism, ), data, S_VISUALS, parseVisual!, cfg)
            else # Remove from dict to avoide problems when checking for remaining fields
                pop!(data, S_MATERIALS, nothing)
                pop!(data, S_VISUALS, nothing)
            end
            parseOptionalNamedElements!((mechanism, ), data, S_COMPONENTS, parseComponent!, cfg)
            
            check_for_remaining_fields(data, cfg)
            # TODO check all frames are defined in 'frames'
            # TODO check all coords are defined
            # TODO check all joints are defined
            mechanism
        end
    else
        rson_throw_exception("Expected mechanism value to be either a path (String) or json (AbstractDict), got a $(typeof(data)).", cfg)
    end
end

function parse_vms(data, ::Type{T}, cfg) where T
    isa(data, JSON_OBJECT) || rson_throw_exception("Expected virtual mechanism system value to be a json object ($JSON_OBJECT), not '$(typeof(data))'.", cfg)
    name = expectField(data, S_NAME, String, cfg)
    with_name_stack(name, cfg) do
        _type = expectField(data, S_TYPE, JSON_STR, cfg)
        @assert _type == S_VIRTUAL_MECHANISM_SYSTEM # This should always be the case
        expect = (field, TYPE) -> expectField(data, field, TYPE, cfg)
        robot_data = expect("robot", Union{JSON_STR, JSON_OBJECT})
        vm_data = expect("virtual_mechanism", Union{JSON_STR, JSON_OBJECT})
        robot = parse_mechanism(robot_data, T, cfg)
        vm = parse_mechanism(vm_data, T, cfg)
        vms = VirtualMechanismSystem(name, robot, vm; from_path=cfg.rson_path)
        
        parseOptionalNamedElements!((vms,), data, S_COORDINATES, _parseCoordDefinition!, cfg)
        parseOptionalNamedElements!((vms,), data, S_COMPONENTS, parseComponent!, cfg)
        check_for_remaining_fields(data, cfg)
        # TODO check all coords are defined
        vms
    end
end

###############################
# Frames
###############################

function parseNewFrame!(mechanism, data, cfg)
    isa(data, JSON_STR) || rson_throw_exception("Expected frame to be a json string ($JSON_STR), not a '$(typeof(data))'", cfg)
    add_frame!(mechanism; id=data)
end

function parseFrameOutsideFramesList(mechanism, data, cfg)::String
    if isa(data, JSON_STR)
        frame_name = parseFrameString(data, cfg)
    elseif isa(data, JSON_OBJECT)
        frame_name = create_anonymous_frame!(mechanism, data, cfg);
    else
        rson_throw_exception("Expected frame to be a json string ($JSON_STR) or a json object ($JSON_OBJECT) defining an anonymous frame, not a '$(typeof(data))'", cfg)
    end
end

function create_anonymous_frame!(mechanism, data::JSON_OBJECT, cfg::RSONParserConfig)::String
    with_name_stack("frame", cfg) do
        T = eltype(mechanism)
        parent = parseFrameString(
            expectField(data, "parent", JSON_STR, cfg), cfg
        );
        frame_name = get_anon_frame_name(parent, frames(mechanism), cfg);
        child = add_frame!(mechanism, frame_name)
        jointID = "anon_joint_`$(frame_name)"
        add_joint!(mechanism, Rigid(parseTransformData(data, T, cfg)); parent, child, id=jointID)
        check_for_remaining_fields(data, cfg)
        child
    end
end

function get_anon_frame_name(parent, frames, cfg)
    i = 1;
    frame = ""
    while true
        frame = "$(parent)_anon_$(i)"
        if !(frame in frames)
            break
        end
        i = i + 1;
    end
    frame
end

function parseFrameString(frame_str, cfg)
    isa(frame_str, JSON_STR) || rson_throw_exception("Expected frame to be a json string ($JSON_STR), not a '$(typeof(frame_str))'", cfg)
    frame_str
end

function parseTransformData(data::JSON_OBJECT, ::Type{T}, cfg) where T
    origin = getField(data, "origin", JSON_ARRAY, cfg; default=nothing)
    if isnothing(origin)
        origin = zero(SVector{3, T});
    else
        origin = parseVec(origin, T, Val{3}(), cfg)
    end
    rotation = getField(data, "rotation", JSON_ARRAY, cfg; default=nothing)
    if isnothing(rotation)
        rotor = zero(Rotor{T});
    else
        rotor = parseRotation(rotation, T, cfg)
    end
    Transform(origin, rotor)
end

function parseRotation(v, ::Type{T}, cfg) where T
    if length(v) == 3
        rpy = parseVec(v, T, Val{3}(), cfg)
        ZRotor(rpy[3]) * YRotor(rpy[2]) * XRotor(rpy[1])
    elseif length(v) == 4
        quaternion = parseVec(v, T, Val{4}(), cfg)
        Rotor(quaternion)
    end
end

###############################
# Joints
###############################

function parseAxis(axisdata, ::Type{T}, cfg) where T
    normalize(parseVec(axisdata, T, Val{3}(), cfg)) # All axes are normalized
end

function parseJoint!(mechanism, name::String, data, cfg::RSONParserConfig)
    if isa(data, JSON_OBJECT)
        type = expectField(data, "type", String, cfg) 
        T = eltype(mechanism)
        expect = (field, TYPE) -> expectField(data, field, TYPE, cfg)
        if type in ["revolute", "continuous"]
            transform = parseTransformData(data, T, cfg)
            axis = parseAxis(expect("axis", Vector{Any}), T, cfg)
            jointdata = RevoluteData(axis, transform)
        elseif type in ["fixed", "rigid"]
            transform = parseTransformData(data, T, cfg)
            jointdata = Rigid(transform)
        elseif type == "prismatic"
            transform = parseTransformData(data, T, cfg)
            axis = parseAxis(expect("axis", Vector{Any}), T, cfg)
            jointdata = PrismaticData(axis, transform)
        elseif type == "helical"
            transform = parseTransformData(data, T, cfg)
            axis = parseAxis(expect("axis", Vector{Any}), T, cfg)
            lead = parseScalarValue(expect("lead", Float64), T, cfg)
            jointdata = Helical(axis, lead, transform)
        else
            rson_throw_exception("Unrecognised joint type '$(type)'", cfg)
        end

        parent = parseFrameString(expect("parent", String), cfg)
        child = parseFrameString(expect("child", String), cfg)
        add_joint!(mechanism, jointdata; parent, child, id=name)
        check_for_remaining_fields(data, cfg)
    else
        rson_throw_exception("Expected joint to be a json object ($JSON_OBJECT), not a '$(typeof(data))'", cfg)
    end
    nothing
end

###############################
# Coordinates
###############################

function _parseCoordDefinition!(mechanism, name::String, data, cfg::RSONParserConfig)
    if isa(data, JSON_OBJECT)
        T = eltype(mechanism)
        type = expectField(data, "type", String, cfg) 
        expect = (field, TYPE) -> expectField(data, field, TYPE, cfg)
        if type == "const_coord"
            val = parseVec(expect("val", Vector{Any}), T, cfg)
            coord = ConstCoord(val)
        elseif type == "coord_difference"
            parent_id = parseCoordString(expect("parent", String), cfg)
            child_id = parseCoordString(expect("child", String), cfg)
            coord = CoordDifference(parent_id, child_id)
        elseif type == "coord_norm"
            coord_id = parseCoordString(expect("coord", String), cfg)
            coord = CoordNorm(coord_id)
        elseif type == "coord_slice"
            idxs = parseVec(expect("idxs", Vector{Any}), Int, cfg)
            c1 = parseCoordString(expect("coord", String), cfg)
            coord = CoordSlice(c1, idxs)
        elseif type == "coord_stack"
            c1 = parseCoordString(expect("c1", String), cfg)
            c2 = parseCoordString(expect("c2", String), cfg)
            coord = CoordStack(c1, c2)
        elseif type == "coord_sum"
            c1_id = parseCoordString(expect("c1", String), cfg)
            c2_id = parseCoordString(expect("c2", String), cfg)
            coord = CoordSum(c1_id, c2_id)
        elseif type == "frame_angular_velocity"
            frame = parseFrameString(expect("frame", String), cfg)
            coord = FrameAngularVelocity(frame)
        elseif type == "frame_origin"
            frame = parseFrameString(expect("frame", String), cfg)
            coord = FrameOrigin(frame)
        elseif type == "frame_point"
            frame = parseFrameString(expect("frame", String), cfg)
            point = parseVec(expect("point", Vector{Any}), T, Val{3}(), cfg)
            coord = FramePoint(frame, point)
        elseif type == "joint_subspace"
            jointid = expect("joint", String)
            coord = JointSubspace(jointid)
        elseif type == "quaternion_attitude"
            frame = parseFrameString(expect("frame", String), cfg)
            rotation_data = getField(data, "target_rotation", JSON_ARRAY, cfg; default=nothing)
            target_rotation = isnothing(rotation_data) ? zero(Rotor{T}) : parseRotation(rotation_data, T, cfg)
            coord = QuaternionAttitude(frame, target_rotation)
        elseif type == "reference_coord"
            val = parseVec(expect("val", Vector{Any}), T, cfg) # Size unknown
            N = length(val)
            vel_data = getField(data, "vel", JSON_ARRAY, cfg; default=nothing)
            vel = isnothing(vel_data) ? zero(SVector{N, Float64}) : parseVec(vel_data, T, Val{N}(), cfg)
            coord = ReferenceCoord(Ref(val), Ref(vel))
        elseif type == "rotated_coord"
            frame = parseFrameString(expect("frame", String), cfg)
            world_frame_coord_id = parseCoordString(expect("world_frame_coord", String), cfg)
            coord = RotatedCoord(world_frame_coord_id, frame)
        elseif type == "unrotated_coord"
            frame = parseFrameString(expect("frame", String), cfg)
            link_frame_coord_id = parseCoordString(expect("link_frame_coord", String), cfg)
            coord = UnrotatedCoord(link_frame_coord_id, frame)
        else
            rson_throw_exception("Unrecognised coord type '$(type)'", cfg)
        end
        add_coordinate!(mechanism, coord; id=name)
        check_for_remaining_fields(data, cfg)
        name
    else
        rson_throw_exception("Expected coord to be a json object ($JSON_OBJECT), not a '$(typeof(data))'", cfg)
    end
end

function parseCoordString(coord_str, cfg)
    string(coord_str)
end

###############################
# Visuals
###############################

function scalemesh(mesh, scale, cfg)
    c, f = GeometryBasics.coordinates(mesh), faces(mesh)
    GeometryBasics.Mesh(scale .* c, f)  
end

function parseVisual!(mechanism, name::JSON_STR, data, cfg::RSONParserConfig)
    if isa(data, JSON_OBJECT)
        type = expectField(data, "type", String, cfg) 
        expect = (field, TYPE) -> expectField(data, field, TYPE, cfg)
        if type == "box"
            widths = parseVec(expect("size", Vector{Any}), Float64, Val{3}(), cfg)
            geom = Rect{3, Float64}(-widths./2, widths)
        elseif type == "cylinder"
            L = expect("length", Float64)
            origin = L * SVector(0.0, 0.0, -0.5) 
            extremity = L * SVector(0.0, 0.0, 0.5) 
            geom = Cylinder{Float64}(origin, extremity, expect("radius", Float64))
        elseif type == "sphere"
            origin = SVector(0.0, 0.0, 0.0)
            geom = Sphere{Float64}(origin, expect("radius", Float64))
        elseif type == "mesh"
            if isnothing(cfg.rson_path)
                rson_path = "."
                cfg.suppress_warnings || @warn "No rson_path provided, using '.' as default path for relative path to find meshes."
            else
                rson_path, _ = splitdir(cfg.rson_path)
            end
            mesh_path = expect("filename", String)
            path = joinpath(rson_path, mesh_path)
            mesh = load_mesh(path, cfg) # This can return multiple meshes
            geom = MeshFromFile(mesh, mesh_path) # MeshFromFile is a wrapper for GeometryBasics.Mesh
            # visual.scale = expect("scale")
        elseif type == "meshdata"
            verts = expect("vertices", Vector{Any})
            faces = expect("faces", Vector{Any})
            # TODO verify that verts and faces are correct types/lengths
            verts = [parseVertex(vert, cfg) for vert in verts]
            faces = [parseFace(face, cfg) for face in faces]
            geom = GeometryBasics.Mesh(verts, faces)
        else
            rson_throw_exception("Unrecognised visual type '$(type)'", cfg)
        end
        frame = parseFrameOutsideFramesList(mechanism, expect("frame", Union{String, OrderedDict{String, Any}}), cfg)
        color = parseMaterial(expect("material", Union{String, Vector{Any}}), cfg)
        
        tf = zero(Transform{Float64})
        add_component!(mechanism, Visual(frame, tf, geom; color); id=name)
        check_for_remaining_fields(data, cfg)
        nothing
    else
        rson_throw_exception("Expected visual to be a json object ($JSON_OBJECT), not a '$(typeof(data))'", cfg)
    end
end

function load_mesh(path, cfg)
    with_name_stack(path, cfg) do
        ret = load(path)
        if isa(ret, GeometryBasics.Mesh)
            return ret
        elseif isa(ret, DAEScene)
            return ret
        else
            throw("Unhandled mesh type $(typeof(ret))")
        end
    end
end

parseMaterial(data::Dict, cfg) = error("Expect named color (string) or RGBA vector, not dictionary. Data: '$(data)'")
parseMaterial(data::AbstractString, cfg) = cfg.named_materials[data] # IF material is a string, it should be a material name 
parseMaterial(data::AbstractVector, cfg) = RGBA(parseVec(data, Float64, Val{4}(), cfg)...) # If material is vector, then it is must be a length 4 RGBA color

function parseMaterialEntry!(mechanism, name, data, cfg)
    cfg.named_materials[name] = RGBA(parseVec(data, Float64, Val{4}(), cfg)...)
end

function parseVertex(data, cfg)
    if isa(data, JSON_ARRAY)
        point = parseVec(data, Float64, Val{3}(), cfg)
        return Point3f(point...)
    else
        rson_throw_exception("Expected vertex to be a json array ($JSON_ARRAY), not a '$(typeof(data))'", cfg)
    end
end

function parseFace(data, cfg)
    if isa(data, JSON_ARRAY)
        face = parseVec(data, Int, Val{3}(), cfg)
        return TriangleFace(face...)
    else
        rson_throw_exception("Expected face to be a json array ($JSON_ARRAY), not a '$(typeof(data))'", cfg)
    end
end

###############################
# Components
###############################

function parseComponent!(
        mechanism,
        name::JSON_STR, 
        component_data, 
        cfg::RSONParserConfig
    )
    if isa(component_data, JSON_OBJECT)
        T = eltype(mechanism)::Type{<:Real}
        type = expectField(component_data, "type", String, cfg) 
        _expect = (field, TYPE) -> expectField(component_data, field, TYPE, cfg);
        _get = (field, TYPE; default) -> getField(component_data, field, TYPE, cfg; default);
        ################################################################
        # Inertances
        ################################################################
        if type == "point_mass"
            mass = T( _expect("mass", Float64))
            coord = parseCoordString(_expect("coord", String), cfg)
            add_component!(mechanism, PointMass(mass, coord); id=name)
        elseif type == "inertia"
            inertia = parseInertiaValue(_expect("inertia", Any), T, cfg)
            coord_data = _get("coord", JSON_STR; default=nothing)
            frame_data = _get("frame", JSON_STR; default=nothing)
            xor(isnothing(coord_data), isnothing(frame_data)) || rson_throw_exception("Expected exactly one of 'coord' or 'frame' to be defined: $(component_data)", cfg)
            if isnothing(frame_data)
                coord = parseCoordString(coord_data, cfg)
                add_component!(mechanism, Inertia(inertia, coord); id=name)
            else
                frame = parseFrameString(frame_data, cfg)
                add_inertia!(mechanism, frame, inertia; id=name*"_inertia")
            end
        elseif type == "inertial"
            frame = parseFrameString(_expect("frame", String), cfg)
            mass = T(_expect("mass", Float64))
            centre_of_mass = _get("com", JSON_ARRAY; default=nothing)
            isnothing(centre_of_mass) || (centre_of_mass = parseVec(centre_of_mass, cfg.element_type, Val{3}(), cfg))
            if ~iszero(mass)
                com_coord_data = isnothing(centre_of_mass) ? FrameOrigin(frame) : FramePoint(frame, centre_of_mass)
                com_coord_id = add_coordinate!(mechanism, com_coord_data; id="__FrameOrigin_"*frame)
                add_component!(mechanism, PointMass(mass, com_coord_id); id=name*"_point_mass")
            end
            inertia = parseInertiaValue(_expect("inertia", Any), T, cfg)
            if ~iszero(inertia)
                add_inertia!(mechanism, frame, inertia; id=name*"_inertia", coordID="__FrameAngularVelocity_"*frame)
            end
        elseif type == "linear_inerter"
            inerter_inertance = parseParameterValue(_expect("inertance", Union{Float64, Vector{Any}}), T, cfg)
            coord = parseCoordString(_expect("coord", String), cfg)
            add_component!(mechanism, LinearInerter(inerter_inertance, coord); id=name)
        ################################################################
        # Storages
        ################################################################
        elseif type == "linear_spring"
            stiffness = parseParameterValue(_expect("stiffness", Any), T, cfg)
            coord = parseCoordString(_expect("coord", String), cfg)
            add_component!(mechanism, LinearSpring(stiffness, coord); id=name)
        elseif type == "tanh_spring"
            stiffness = _get("stiffness", JSON_FLOAT; default=nothing)
            max_force = _get("max_force", JSON_FLOAT; default=nothing)
            width = _get("width", JSON_FLOAT; default=nothing)
            if sum([isnothing(stiffness), isnothing(max_force), isnothing(width)]) != 1 
                rson_throw_exception("Expected exactly two of 'stiffness', 'max_force', 'width'.", cfg)
            end
            coord = parseCoordString(_expect("coord", String), cfg)
            tspring = TanhSpring(coord; stiffness, max_force, width)
            add_component!(mechanism, tspring; id=name)
        elseif type == "rectified_spring"
            stiffness = _expect("stiffness")
            coord = parseCoordString!(mechanism, _expect("coord", String), cfg)
            flipped = _get("flipped", JSON_BOOL; default=false)
            rspring = RectifiedSpring(stiffness, coord, flipped)
            add_component!(mechanism, rspring; id=name)
        elseif type == "gravity_compensator"
            mass = parseParameterValue(_expect("mass", Float64), T, cfg)
            coord = parseCoordString(_expect("coord", String), cfg)
            gravity = parseVec(_expect("gravity", Vector{Any}), T, Val{3}(), cfg)
            add_component!(mechanism, GravityCompensator(mass, coord, gravity); id=name)
        ################################################################
        # Dissipations
        ################################################################
        elseif type == "linear_damper"
            damping = parseParameterValue(_expect("damping_coefficient", Any), T, cfg)
            coord = parseCoordString(_expect("coord", String), cfg)
            add_component!(mechanism, LinearDamper(damping, coord); id=name)
        elseif type == "rectified_damper"
            damping = _expect("damping_coefficient", Float64)
            coord = parseCoordString(_expect("coord", String), cfg)
            bounds = _expect("bounds", Vector{Any})
            length(bounds) == 2 ||  rson_throw_exception("Expected exactly two bounds for rectified damper, got $(length(bounds)), with values: $(bounds)", cfg)
            flipped = _get("flipped", JSON_BOOL; default=false)
            diodic = _get("diodic", JSON_BOOL; default=false)
            rdamper = RectifiedDamper(damping, coord, (bounds...,), flipped, diodic)
            add_component!(mechanism, rdamper; id=name)
        else    
            rson_throw_exception("Unhandled component type : '$(type)'", cfg)
        end
        check_for_remaining_fields(component_data, cfg)
        nothing    
    else
        rson_throw_exception("Expected component to be a json object ($JSON_OBJECT), not a '$(typeof(component_data))'", cfg)
    end
end

###############################
# Utility functions
###############################

function find_frame_index(mechanism, frame_name)
    findfirst(item -> item == frame_name, frames(mechanism))
end

"""
    parseRSONVersion(data, cfg)

Will parse the version, and assert that it is compatible with the current parser version.
The parsed version will be stored in `cfg.rson_version`.

The version is expected to be a string of the form "v<major>.<minor>.<patch>", where each part is
an integer.
"""
function parseRSONVersion(version_string, cfg)
    @assert typeof(version_string) == String "Expected version string to be a string, got type '$(typeof(version_string))', with value '$(version_string)'."
    # Regex pattern from https://semver.org/
    pattern = r"^(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)(?:-((?:0|[1-9]\d*|\d*[a-zA-Z-][0-9a-zA-Z-]*)(?:\.(?:0|[1-9]\d*|\d*[a-zA-Z-][0-9a-zA-Z-]*))*))?(?:\+([0-9a-zA-Z-]+(?:\.[0-9a-zA-Z-]+)*))?$"
    regex_result = match(pattern, version_string)
    isnothing(regex_result) && rson_throw_exception("Invalid version string '$(version_string)'", cfg)
    major, minor, patch = parse.(Int, regex_result.captures[1:3])
    cfg.rson_version = (major, minor, patch)

    errmsg = "Parser version $RSON_PARSER_VERSION_STRING, file version $version_string"
    major == RSON_PARSER_VERSION[1] || rson_throw_exception("Major version mismatch. $errmsg", cfg)
    minor <= RSON_PARSER_VERSION[2] || rson_throw_exception("Minor version incompatibility. $errmsg", cfg)
    nothing
end

function validate_name_string(name::String)
    if startswith(name, ".")
        rson_throw_exception("Name '$(name)' cannot start with a period.", cfg)
    end
end

const PARSE_INERTIA_ERROR_MSG = """
Expected inertia to be one of the following: a dictionary with keys \"xx\", \"yy\" 
OR a 3x3 matrix, or a 6-vector in the order [Ixx, Iyy, Izz, Ixy, Ixz, Iyz].
Examples:
{"xx":1e-2, "yy": 1e-2, "zz": 1e-2, "xy": 1e-3, "xz": 1e-3, "yz": 1e-3}
OR
[[1e-2, 1e-3, 1e-3],
 [1e-3, 1e-2, 1e-3],
 [1e-3, 1e-3, 1e-2]]
 OR
[1e-2, 1e-2, 1e-2, 1e-3, 1e-3, 1e-3].

"""

function parseInertiaValue(in, ::Type{T}, cfg) where T
    inertia = if isa(in, AbstractDict)
        _get(key) = get(in, key, zero(T)) # Get value or 0 if not found
        r1 = SVector(_get("xx"), _get("xy"), _get("xz"))
        r2 = SVector(_get("xy"), _get("yy"), _get("yz"))
        r3 = SVector(_get("xz"), _get("yz"), _get("zz"))
        inertia = vcat(r1', r2', r3')        
    elseif isa(in, AbstractVector)
        if length(in) == 3
            r1 = parseVec(in[1], T, Val{3}(), cfg)
            r2 = parseVec(in[2], T, Val{3}(), cfg)
            r3 = parseVec(in[3], T, Val{3}(), cfg)
            inertia = vcat(r1', r2', r3')
        elseif length(in) == 6
            r1 = SVector(in[1], in[4], in[5])
            r2 = SVector(in[4], in[2], in[6])
            r3 = SVector(in[5], in[6], in[3])
            inertia = vcat(r1', r2', r3')
        end
    else
        rson_throw_exception(PARSE_INERTIA_ERROR_MSG * "Got '$(in)'", cfg)
    end
    inertia::SMatrix{3, 3, T, 9}
    isposdef(inertia) || iszero(inertia) || rson_throw_exception("Expected positive definite or zero inertia, got '$(inertia)'", cfg)
    inertia
end

function parseOptionalArray!(args, data, field, parse_single!, cfg)
    # Parse data for a list with name 'field' that may or may not exist. 
    # Can also mutate mechanism, such as for adding anonymous frames
    array = getField(data, field, JSON_ARRAY, cfg; default=nothing)
    isnothing(array) && return nothing
    with_name_stack(field, cfg) do
        for data in array
            parse_single!(args..., data, cfg)
            nothing
        end
    end
    nothing
end


function parseOptionalNamedElements!(args, data, field, parse_single!, cfg)
    # Parse data for a list with name 'field' that may or may not exist. 
    # Can also mutate mechanism, such as for adding anonymous frames
    named_elements = getField(data, field, JSON_OBJECT, cfg; default=nothing)
    isnothing(named_elements) && return nothing
    with_name_stack(field, cfg) do
        for (name, data) in named_elements
            with_name_stack(name, cfg) do
                parse_single!(args..., name, data, cfg)
                nothing
            end
        end
    end
    nothing
end

function parseScalarValue(data, T, cfg)
    return T(data)
end

# TODO rename parseParameterValue to something like "parsePosdefValue"
function parseParameterValue(data::Real, T, cfg)
    data >= 0.0 || rson_throw_exception("Expected parameter value to be non-negative, got '$(data)'", cfg)
    T(data)
end

function parseParameterValue(data::AbstractVector, T, cfg)
    if all(isa.(data, AbstractVector))
        data = Vector{Vector}(data)
        return parseParameterValue(data, T, cfg)
    else
        rson_throw_exception("Failed to parse parameter value of type: '$(typeof(data))', with data '$(data)'", cfg)
    end
end

function parseParameterValue(data::AbstractVector{Real}, T, cfg)
    # Interpret as diagonal matrix
    length(data) == 3 || rson_throw_exception("Expected parameter vector to be length 3, got $(length(data)). Data: $(data)", cfg)
    all(data .>= 0.0) || rson_throw_exception("Expected parameter values to all be non-negative, got $(data)", cfg)
    data_T = map(T, data)
    Diagonal(data_T)
end

function parseParameterValue(data::AbstractVector{AV}, T, cfg) where AV <: AbstractVector 
    if all(all(isa(item, Real) for item in vec) for vec in data)
        matrix = Matrix{Float64}(vcat(data'...))
        size(matrix) == (3, 3) || rson_throw_exception("Expected parameter matrix to be 3x3, not '$(size(matrix))'. Data: '$(data)'", cfg)
        issymmetric(matrix) || rson_throw_exception("Expected parameter matrix to be symmetric, not '$(matrix)'. Data: '$(data)'", cfg)
        isposdef(matrix) || rson_throw_exception("Expected parameter matrix to be positive definite. Data: '$(data)'", cfg)
        matrix_T = map(T, matrix)
        SMatrix{3, 3}(matrix_T)
    else
        rson_throw_exception("Failed to parse parameter value of type: '$(typeof(data))', with data '$(data)'", cfg)
    end
end

@inline function parseVec(
        data,
        ::Type{T}, 
        size::Val{N}, 
        cfg
    ) where {T, N}
    if !isa(data, JSON_ARRAY)
        rson_throw_exception("Expected data to be a json array ($JSON_ARRAY), got a '$(typeof(data))'. data: '$(data)'", cfg)
    end
    local vals
    try 
        vals = map(data) do v
            v = isa(v, Nothing) ? NaN : v # This is because JSON serializing will convert NaN to null, which is parsed as nothing
            convert(T, v)
        end
    catch e
        rson_throw_exception("Cannot convert data in vector to type '$T'. data: '$(data)'.", cfg)
    else
        return SVector{N, T}(vals)
    end
    error()
end

@inline function parseVec(
        data,
        ::Type{T},
        cfg
    ) where T
    if !isa(data, Vector)
        rson_throw_exception("Expected data to be a vector, got a '$(typeof(data))'. data: '$(data)'", cfg)
    end
    SVector{length(data), T}(data)
end
