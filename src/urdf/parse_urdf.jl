# Joint limits are not directly supported, but the information from the URDF is parsed,
# and stored in the URDFParserConfig object. This information can be used to add joint limits
# to the mechanism after parsing, based upon custom mechanisms/logic for how to enforce the joint
# limits.
struct URDFJointLimit
    lower::Union{Float64, Nothing}
    upper::Union{Float64, Nothing}
    effort::Float64
    velocity::Float64
end

"""
mutable struct URDFParserConfig
        
Used to configure the URDF parser. WARNING: may be modified during parsing (e.g. materials are added
to named materials).

# Fields
- `urdf_path::String`: Path to the URDF file.
- `element_type::Type`: eltype to build the mechanism with, default `Float64`.
- `parse_visuals::Bool`: Whether to parse visual elements, default `true`.
- `error_on_not_supported::Bool`: Whether to error on unsupported URDF features, default `false`.
- `error_on_not_recognized::Bool`: Whether to error on unrecognized URDF features, default `true`.
- `suppress_warnings::Bool`: Whether to suppress warnings, default `false`.
- `default_material::RGBA`: Default colour to use for visuals, default `RGBA(0.8, 0.8, 0.8, 1.0)`.
- `named_materials::Dict{String, RGBA}`: Named materials to use for visuals, default `Dict()`.
"""
@kwdef mutable struct URDFParserConfig
    urdf_path::String=""
    element_type::Type=Float64
    parse_visuals::Bool=true
    error_on_not_supported::Bool=false
    error_on_not_recognized::Bool=true
    suppress_warnings::Bool=false
    add_floating_base::Bool=false # TODO support
    default_material::NamedTuple=(;color=RGBA(0.8, 0.8, 0.8, 1.0))
    named_materials::Dict{String, NamedTuple}=Dict()
    joint_limits::Dict{String, Union{URDFJointLimit, Nothing}}=Dict()
end

const DEFAULT_CONFIG = URDFParserConfig()

##
# Entry points
##

"""
    parseURDF(filepath::AbstractString, [cfg::URDFParserConfig])
    parseURDF(cfg::URDFParserConfig)

Parse a `Mechanism` from a URDF file. If filepath is provided, a default config is used. 
If ONLY cfg is provided, the URDF file path must be set in the config.

See also [`parseURDFString`](@ref), [`URDFParserConfig`]
"""
function parseURDF(filepath::AbstractString, cfg::URDFParserConfig=DEFAULT_CONFIG)
    cfg.urdf_path = filepath
    doc = readxml(cfg.urdf_path)
    parseURDF(doc, cfg)
end

function parseURDF(cfg::URDFParserConfig)
    doc = readxml(cfg.urdf_path)
    parseURDF(doc, cfg)    
end

"""
    parseURDFString(urdfstring::AbstractString, [cfg::URDFParserConfig])

Parse a `Mechanism` from a URDF string. If cfg is provided, the directory of the URDF file path 
from the config will be used to find mesh files.
"""
function parseURDFString(urdfstring::AbstractString, cfg::URDFParserConfig=DEFAULT_CONFIG)
    doc = parsexml(urdfstring)
    cfg.urdf_path = filepath
    parseURDF(doc, cfg)
end

############################

function parseURDF(doc::EzXML.Document, cfg)
    robot = root(doc)
    nodename(robot) == "robot" || error("The root node of the document is not a 'robot', it is '$(nodename(robot))'.")
    parse_robot(robot, cfg)
end

function raise_not_supported(node::String, cfg::URDFParserConfig)
    if cfg.error_on_not_supported
        @error "Node type '$node' not supported. To change this error to a warning set 'error_on_not_supported=false' in URDFParserConfig."
    else
        cfg.suppress_warnings || @warn "Node type '$node' not supported, and will be ignored."
    end
    nothing
end

function raise_not_recognized(node::String, cfg::URDFParserConfig)
    if cfg.error_on_not_recognized
        @error "Node type '$node' not recognized. To change this error to a warning set 'error_on_not_recognized=false' in URDFParserConfig."
    else
        cfg.suppress_warnings || @warn "Node type '$node' not supported, and will be ignored."
    end
    nothing
end

function parse_robot(robot, cfg)
    name = robot["name"]
    mechanism = Mechanism{Float64}(name; with_root_frame=false)
    cfg.add_floating_base && error("Floating base not implemented yet.")

    for node in eachelement(robot)
        nn = nodename(node)
        if nn == "link"
            parse_link!(mechanism, node, cfg)
        elseif nn == "joint"
            parse_joint!(mechanism, node, cfg)
        elseif nn == "material"
            # Materials at the top level are added to the named materials dict
            cfg.named_materials[node["name"]] = parse_material(node, cfg)
        elseif nn == "transmission"
            raise_not_supported(nn, cfg)
        elseif nn == "gazebo"
            raise_not_supported(nn, cfg)
        else
            raise_not_recognized(nn, cfg)
        end
    end
    mechanism
end

##########################
# Links
##########################

function parse_link!(mechanism, link, cfg)
    name = link["name"]
    add_frame!(mechanism, name)
    for node in eachelement(link)
        nn = nodename(node)
        if nn == "inertial"
            parse_inertial!(mechanism, node, name, cfg)
        elseif nn == "visual"
            parse_visual!(mechanism, node, name, cfg)
        elseif nn == "collision"
            raise_not_supported(nn, cfg)
        else
            raise_not_recognized(nn, cfg)
        end
    end
    nothing
end

function parse_inertial!(mechanism, inertial, link_frame, cfg)
    transform = nothing
    mass = nothing
    inertia = nothing
    for node in eachelement(inertial)
        nn = nodename(node)
        if nn == "origin"
            transform = parse_origin(node, cfg)
        elseif nn == "mass"
            mass = parse_mass(node, cfg)
        elseif nn == "inertia"
            inertia = parse_inertia(node, cfg)
        else
            raise_not_recognized(nn, cfg)
        end
    end 
    if ~isnothing(mass)
        mass_coord = let
            position_of_mass_in_link_frame = VMRobotControl.origin(transform)
            if isnothing(transform) || iszero(position_of_mass_in_link_frame)
                FrameOrigin(link_frame)
            else
                FramePoint(link_frame, position_of_mass_in_link_frame)
            end
        end
        coord_id_1 = add_coordinate!(mechanism, mass_coord; id = link_frame * "_mass_coord")
        add_component!(mechanism, PointMass(mass, coord_id_1); id=link_frame * "_mass")
    end
    if ~isnothing(inertia)
        # Transform inertia from inertia frame to link frame, to reduce number of different frames needed
        inertia_in_link_frame = VMRobotControl.change_inertia_frame(rotor(transform), inertia)
        # Ensure symmetric, as it should be, otherwise we have issues when serializing and 
        # deserializing the mechanism
        inertia_in_link_frame = 0.5 * (inertia_in_link_frame + inertia_in_link_frame')
        @assert issymmetric(inertia_in_link_frame)
        @assert isposdef(inertia_in_link_frame)

        inertia_coord = FrameAngularVelocity(link_frame)
        coord_id_2 = add_coordinate!(mechanism, inertia_coord; id = link_frame * "_inertia_coord")
        add_component!(mechanism, Inertia(inertia_in_link_frame, coord_id_2); id=link_frame * "_inertia")
    end
    nothing
end

function parse_mass(node, cfg)
    return parse_scalar_value(node["value"], cfg)
end

function parse_inertia(node, cfg)
    ixx = parse_scalar_value(node["ixx"], cfg)
    ixy = parse_scalar_value(node["ixy"], cfg)
    ixz = parse_scalar_value(node["ixz"], cfg)
    iyy = parse_scalar_value(node["iyy"], cfg)
    iyz = parse_scalar_value(node["iyz"], cfg)
    izz = parse_scalar_value(node["izz"], cfg)
    I = @SMatrix [ixx ixy ixz;
                  ixy iyy iyz;
                  ixz iyz izz]
    @assert issymmetric(I) "Inertia matrix must be symmetric."
    if iszero(I) 
        return nothing
    else
        @assert isposdef(I) "Inertia matrix must be positive definite, but isn't: $I."
    end
    I
end

function parse_visual!(mechanism, visualnode, link_frame, cfg)
    name = geometry = transform = material = nothing
    for node in eachelement(visualnode)
        nn = nodename(node)
        if nn == "name"
            name = node
        elseif nn == "geometry"
            geometry = parse_geometry(node, cfg)
        elseif nn == "origin"
            transform = parse_origin(node, cfg)
        elseif nn == "material"
            material = parse_material(node, cfg)
        else 
            raise_not_recognized(nn, cfg)
        end
    end
    isnothing(name) && (name = link_frame * "_unnamed_visual_$(rand(Int))")
    isnothing(geometry) && error("Geometry not specified for visual with name '$name'.")
    isnothing(material) && (material = cfg.default_material)

    if typeof(geometry)<:Vector{<:Tuple{<:Mesh, <:NamedTuple}}
        for (i, (mesh, kwargs)) in enumerate(geometry)
            v = Visual(link_frame, mesh; kwargs...)
            isnothing(transform) || (v = transform * v)
            add_component!(mechanism, v; id=name*"_$i")
        end
    else
        v = Visual(link_frame, geometry; material...)
        isnothing(transform) || (v = transform * v)
        add_component!(mechanism, v; id=name)
    end

    # visual = let
    #     Visual(name, link_frame, material, geometry)
    # end
    
    # # As each urdf link can have several visuals, we need to give them different names.
    # visual_ids = collect(keys(VMRobotControl.visuals(mechanism)))
    # # Check if name already exists, and if so, add a number to it
    # i = 1
    # while true
    #     if (name * "_$i") in visual_ids 
    #         i += 1
    #     else
    #         break
    #     end
    # end
    # add_component!(mechanism, visual; id=name*"_$i")
end

function parse_geometry(geometry_node, cfg)
    N_nodes = 0
    geom = nothing
    for node in eachelement(geometry_node)
        N_nodes += 1
        N_nodes > 1 && error("`geometry` must have exactly one child.")
        nn = nodename(node)
        if nn == "box"
            size = parse_3vec(node["size"], cfg)
            geom = Rect3{Float64}(SVector(0, 0, 0), size)
        elseif nn == "cylinder"
            r = parse_scalar_value(node["radius"], cfg)
            l = parse_scalar_value(node["length"], cfg)
            origin = SVector(0.0, 0.0, -l/2)
            extremity = SVector(0.0, 0.0, l/2)
            geom = Cylinder3{Float64}(origin, extremity, r)
        elseif nn == "sphere"
            r = parse_scalar_value(node["radius"], cfg)
            geom = Sphere3{Float64}(SVector(0, 0, 0), r)
        elseif nn == "mesh"
            filename = node["filename"]
            geom = load_mesh(filename, cfg)
            if haskey(node, "scale") 
                scale_vec = parse_3vec(node["scale"], cfg)
                scale = GeometryBasics.Point{3, Float64}(scale_vec...)
                geom = VMRobotControl.rescale_mesh(geom, scale)
            end
        else 
            raise_not_recognized(nn, cfg)
        end
    end
    if isnothing(geom)
        error("No geometry specified in `geometry` node.")
    end
    geom
end

function load_mesh(filename, cfg)
    if startswith(filename, "package://")
        filename = chopprefix(filename, "package://")
        fileparts = splitpath(filename)
        package_name = fileparts[1] # TODO use this?
        path = fileparts[2:end]

        package_path = find_package(cfg.urdf_path, package_name)        
        path = joinpath(package_path, joinpath(path))
    else
        path = filename
    end
    try
        ret = load(path) # Try load
        if isa(ret, DAEScene)
            return convert_for_glmakie(ret)
        elseif isa(ret, Mesh)
            return ret
        else
            error("Loaded mesh is not a Mesh or DAEScene.")
        end
    catch
        error("Failed to load mesh at $path, from filename $filename.")
    end
end

function find_package(urdf_path, package_name)
    dir_path_parts = splitpath(abspath(urdf_path))
    while !isempty(dir_path_parts)
        if isfile(joinpath(joinpath(dir_path_parts), "package.xml"))
            # TODO check package name
            # for now, assume we found the right package
            return joinpath(dir_path_parts)
        else
            pop!(dir_path_parts)
        end
    end
    error("Could not find package $package_name, no package.xml in parents of path $(urdf_path).")
end

function parse_material(material_node, cfg)
    rgba = nothing
    for node in eachelement(material_node)
        nn = nodename(node)
        if nn == "color"
            rgba = RGBA(parse_4vec(node["rgba"], cfg)...)
        else 
            raise_not_recognized(nn, cfg)
        end
    end
    if isnothing(rgba)
        if haskey(cfg.named_materials, material_node["name"])
            return cfg.named_materials[material_node["name"]]
        else
            cfg.suppress_warnings || @warn "No color found for material, using default."
            return cfg.default_material
        end
    end
    (;color=rgba::RGBA)
end

###########################
# Joints
###########################

function parse_joint!(mechanism, jointnode, cfg)
    name = jointnode["name"]
    type = jointnode["type"]
    transform = parent = child = axis = limit = nothing
    for node in eachelement(jointnode)
        nn = nodename(node)
        if nn == "origin"
            transform = parse_origin(node, cfg)
        elseif nn == "parent"
            parent = node["link"]
        elseif nn == "child"
            child = node["link"]
        elseif nn == "axis"
            axis = parse_3vec(node["xyz"], cfg)
        elseif nn == "calibration"
            raise_not_supported(nn, cfg)
        elseif nn == "dynamics"
            raise_not_supported(nn, cfg)
        elseif nn == "limit"
            limit = parse_joint_limit(node, cfg)
        elseif nn == "mimic"
            raise_not_supported(nn, cfg)
        elseif nn == "safety_controller"
            raise_not_supported(nn, cfg)
        else 
            raise_not_recognized(nn, cfg)
        end
    end
    (isnothing(parent) || isnothing(child)) && error("Parent or child not specified for joint '$name'.")
    isnothing(transform) && (transform = zero(Transform{cfg.element_type}))
    jointdata = let
        if type == "revolute"
            isnothing(axis) && error("Axis not specified for revolute joint '$name'.")
            L = sqrt(axis' * axis)
            if ~isapprox(L, one(L), rtol=1e-4) 
                cfg.suppress_warnings || @warn "Axis for revolute joint '$name' is not normalized, has norm '$L'. Normalizing."
            end
            Revolute(axis./L, transform)
        elseif type == "continuous"
            isnothing(axis) && error("Axis not specified for continuous joint '$name'.")
            Revolute(axis, transform)
        elseif type == "prismatic"
            isnothing(axis) && error("Axis not specified for prismatic joint '$name'.")
            Prismatic(axis, transform)
        elseif type == "fixed"
            Rigid(transform)
        elseif type == "floating"
            raise_not_supported(type, cfg)
            return
        elseif type == "planar"
            raise_not_supported(type, cfg)
            return
        else
            raise_not_recognized(type, cfg)
            return
        end
    end
    cfg.joint_limits[name] = limit # Store joint limits for later
    add_joint!(mechanism, jointdata; parent, child, id=name, check_frames_exist=false)
end

###########################
# Utils
###########################

function parse_origin(node, cfg)
    T = cfg.element_type
    xyz = parse_optional_3vec(node, "xyz", cfg; default=zero(SVector{3, T}))
    rpy = parse_optional_3vec(node, "rpy", cfg; default=zero(SVector{3, T}))
    rotor = ZRotor(rpy[3]) * YRotor(rpy[2]) * XRotor(rpy[1])
    return Transform(xyz, rotor)
end

function parse_scalar_value(str, cfg)
    cfg.element_type(parse(Float64, str))
end

function parse_optional_3vec(node, attr_name, cfg; default)
    haskey(node, attr_name) ? parse_3vec(node[attr_name], cfg) : default
end

function parse_3vec(str, cfg)
    T = cfg.element_type
    rpy = map(split(str, " "; keepempty=false)) do s
        T(parse(Float64, s)) # Parse as Float64, but convert to T. Should work for Duals, etc.
    end
    length(rpy) == 3 || error("Must have 3 elements, separated by spaces. Received: '$rpystr'.")
    SVector{3, T}(rpy...)
end

function parse_4vec(str, cfg)
    T = cfg.element_type
    rgba = map(split(str, " "; keepempty=false)) do s
        T(parse(Float64, s)) # Parse as Float64, but convert to T. Should work for Duals, etc.
    end
    length(rgba) == 4 || error("Must have 4 elements, separated by spaces. Received: '$rpystr'.")
    SVector{4, T}(rgba...)
end

function parse_joint_limit(node, cfg)    
    lower = haskey(node, "lower") ? parse_scalar_value(node["lower"], cfg) : nothing
    upper = haskey(node, "upper") ? parse_scalar_value(node["upper"], cfg) : nothing
    haskey(node, "effort") || error("Limit node must have 'effort' attribute.")
    haskey(node, "velocity") || error("Limit node must have 'velocity' attribute.")
    effort = parse_scalar_value(node["effort"], cfg)
    velocity = parse_scalar_value(node["velocity"], cfg)
    URDFJointLimit(lower, upper, effort, velocity)
end
