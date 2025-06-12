# TODO this is broken and needs work

# function serialize(mechanism::Mechanism, filename)
#     _, ext = splitext(filename);

#     if ext == ".rson"
#         ;
#     elseif ext == ""
#         filename += ".rson"
#     else
#         @warn("Writing mechanism to file with file extension '$(ext)', instead of '.rson'");
#     end

#     open(filename,"w") do f
#         JSON.print(f, serialize_mechanism(mechanism))
#     end
# end

"""
    serializeRSON(filepath, m::Union{Mechanism, VirtualMechanismSystem})

Takes a mechanism or virtual mechanism system and serializes it to a file at `filepath` in the RSON 
format. 

When handling visuals, the mesh data is serialized as a list of vertices and faces. 

Does not guarantee that a parsed RSON is serialized back into an identical RSON, but the mechanism
should be the same.
"""
function serializeRSON(filepath, m::Union{Mechanism, VirtualMechanismSystem})
    f = open(filepath, "w")
    try
        JSON.print(f, serialize(m), 4) # 4 spaces per indent
    finally
        close(f)
    end
end

function serializeRSON(::Type{String}, m::Union{Mechanism, VirtualMechanismSystem})
    JSON.json(serialize(m), 4) # 4 spaces per indent
end


serialize_joint_dict(dict::OrderedDict) = begin
    OrderedDict(name => serialize_joint(val) for (name, val) in dict)
end
serialize_component_dict(dict) = OrderedDict(name => serialize_component(val) for (name, val) in dict)
function serialize_coord_dict(coordinates)
    # Coordinates are serialized in a way that respects their dependencies
    skip_if = (id) -> length(split(id, ".")) > 1 # TODO this is a hack
    coord_graph, ID_to_vertex_map = VMRobotControl._construct_coordinate_graph(coordinates; skip_if)
    coord_id_computation_order = VMRobotControl.determine_coordinate_computation_order(coord_graph, ID_to_vertex_map, coordinates; is_computed=skip_if)
    
    OrderedDict(
        name => serialize_coord(coordinates[name])
            for names in coord_id_computation_order
                for name in names
    )
end

function serialize(vms::VirtualMechanismSystem)
    OrderedDict(
        "rson_version" => RSON_PARSER_VERSION_STRING,
        "name" => vms.name,
        "type" => "virtual_mechanism_system",
        "robot" => serialize(vms.robot),
        "virtual_mechanism" => serialize(vms.virtual_mechanism),
        "coordinates" => serialize_coord_dict(VMRobotControl.coordinates(vms)),
        "components" => merge(
            serialize_component_dict(storages(vms)),
            serialize_component_dict(dissipations(vms)),
        )
    )
end

function serialize(mechanism::Mechanism)
    OrderedDict(
        "rson_version" => RSON_PARSER_VERSION_STRING,
        "name" => mechanism.name,
        "type" => "mechanism",
        "frames" => frames(mechanism),
        "joints" => serialize_joint_dict(joints(mechanism)),
        "visuals" => serialize_component_dict(visuals(mechanism)),
        "coordinates" => serialize_coord_dict(VMRobotControl.coordinates(mechanism)),
        "components" => merge(
            serialize_component_dict(inertances(mechanism)),
            serialize_component_dict(storages(mechanism)),
            serialize_component_dict(dissipations(mechanism)),
            serialize_component_dict(generic_components(mechanism))
        )
    )
end

#################
# Joints

function serialize_joint(j::MechanismJoint)
    merge(
        serialize_jointdata(j.jointData),
        OrderedDict(
            "parent"=> j.parentFrameID,
            "child"=> j.childFrameID
        )
    )
end

function serialize_jointdata(j::RevoluteData)
    merge(
        OrderedDict(
            "type" => "revolute",
            "axis" => jsonify(j.axis)
        ),
        jsonify(j.transform)
    )
end

function serialize_jointdata(j::PrismaticData)
    merge(
        OrderedDict(
            "type" => "prismatic",
            "axis" => jsonify(j.axis)
        ),
        jsonify(j.transform)
    )
end


function serialize_jointdata(j::Rigid)
    merge(
        OrderedDict("type" => "rigid"),
        jsonify(j.transform)
    )
end

function serialize_jointdata(j::ReferenceJoint)
    @warn "ReferenceJoint cannot be serialiazed by its nature. Treating as a rigid joint when serializing."
    merge(
        OrderedDict("type" => "rigid"),
        jsonify(j.transform[])
    )
end


function jsonify(tf::Transform)
    # TODO if origin is zero, omit
    # TODO if rotation is identity, omit
    OrderedDict(
        "origin" => jsonify(VMRobotControl.origin(tf)),
        "rotation" => jsonify(rotor(tf))
    )
end

jsonify(r::Rotor) = Vector{Float64}(VMRobotControl.rotor_to_svector(r))
jsonify(j::SVector{3, Float64}) = Vector{Float64}(j)
jsonify(j::SMatrix{3, 3, Float64}) = [
    [j[1,1], j[1,2], j[1,3]],
    [j[2,1], j[2,2], j[2,3]],
    [j[3,1], j[3,2], j[3,3]]
]
jsonify(c::RGBA) = Float64[c.r, c.g, c.b, c.alpha]
jsonify(v::Float64) = v

###
# Coordinates
serialize_coord(c::ConstCoord) =        OrderedDict("type" => "const_coord",       "val" => c.val)
serialize_coord(c::CoordDifference) =   OrderedDict("type" => "coord_difference",  "parent" => c.parent,   "child" => c.child)
serialize_coord(c::CoordNorm) =         OrderedDict("type" => "coord_norm",        "coord" => c.coord)
serialize_coord(c::CoordSlice) =        OrderedDict("type" => "coord_slice",       "idxs" => c.idxs,       "coord" => c.coord)
serialize_coord(c::CoordStack) =        OrderedDict("type" => "coord_stack",       "c1" => c.c1,           "c2"=> c.c2)
serialize_coord(c::CoordSum) =          OrderedDict("type" => "coord_sum",         "c1" => c.c1,           "c2" => c.c2)
serialize_coord(c::FrameAngularVelocity) = OrderedDict("type" => "frame_angular_velocity", "frame" => c.frameID)
serialize_coord(c::FrameOrigin) =       OrderedDict("type" => "frame_origin",      "frame" => c.frameID)
serialize_coord(c::FramePoint) =        OrderedDict("type" => "frame_point",       "frame" => c.frameID,   "point" => jsonify(c.point))
serialize_coord(c::JointSubspace) =     OrderedDict("type" => "joint_subspace",    "joint" => c.joint)
serialize_coord(c::QuaternionAttitude) = OrderedDict("type" => "quaternion_attitude", "frame" => c.frameID, "target_rotation" => jsonify(c.target_rotation))
serialize_coord(c::ReferenceCoord) =    OrderedDict("type" => "reference_coord",   "val" => c.val[],       "vel" => c.vel[])
serialize_coord(c::RotatedCoord) =      OrderedDict("type" => "rotated_coord",     "frame" => c.frameID,   "world_frame_coord" => c.world_frame_coord, )
serialize_coord(c::UnrotatedCoord) =    OrderedDict("type" => "unrotated_coord",   "frame" => c.frameID,   "link_frame_coord" => c.link_frame_coord, )


###
# Components

#TODO fix component id and naming
random_component_name() = "$(rand(Int))"

function serialize_component(c::PointMass)
    OrderedDict(
        "type" => "point_mass",
        "coord" => c.coord,
        "mass" => c.mass
    )
end


function serialize_component(c::GravityCompensator)
    OrderedDict(
        "type" => "gravity_compensator",
        "coord" => c.coord,
        "mass" => c.mass,
        "gravity" => jsonify(c.gravity)
    )
end

function serialize_component(c::LinearInerter)
    OrderedDict(
        "type" => "linear_inerter",
        "coord" => c.coord,
        "inertance" => c.inertance
    )
end

function serialize_component(c::Inertia)
    OrderedDict(
        "type" => "inertia",
        "coord" => c.coord,
        "inertia" => [
            [c.inertia[1,1], c.inertia[1,2], c.inertia[1,3],],
            [c.inertia[2,1], c.inertia[2,2], c.inertia[2,3],],
            [c.inertia[3,1], c.inertia[3,2], c.inertia[3,3],],
        ]
    )
end

function serialize_component(c::LinearSpring)
    OrderedDict(
        "type" => "linear_spring",
        "stiffness" => jsonify(c.stiffness),
        "coord" => c.coord
    )
end

function serialize_component(c::TanhSpring)
    OrderedDict(
        "type" => "tanh_spring",
        "stiffness" => jsonify(c.stiffness),
        "width" => jsonify(c.width),
        "coord" => c.coord
    )
end

function serialize_component(c::GaussianSpring)
    OrderedDict(
        "type" => "gaussian_spring",
        "stiffness" => jsonify(c.stiffness),
        "width" => jsonify(c.σ),
        "coord" => c.coord
    )
end


function serialize_component(c::LinearDamper)
    OrderedDict(
        "type" => "linear_damper",
        "damping_coefficient" => jsonify(c.damping),
        "coord" => c.coord
    )
end

function serialize_component(c::RectifiedSpring)
    OrderedDict(
        "type" => "rectified_spring",
        "stiffness" => jsonify(c.stiffness),
        "coord" => c.coord,
        "flipped" => c.flipped
    )
end

function serialize_component(c::RectifiedDamper)
    OrderedDict(
        "type" => "rectified_damper",
        "damping_coefficient" => jsonify(c.damping),
        "bounds" => c.bounds,
        "coord" => c.coord,
        "flipped" => c.flipped,
        "diodic" => c.diodic
    )
end



###
# Visual

function serialize_component(v::Visual)
    if v.geometry isa Mesh
        @warn "Serializing a mesh directly will create a large rson file."
        verts = GeometryBasics.coordinates(v.geometry)
        faces = [Int[face...] for face in GeometryBasics.faces(v.geometry)]
        OrderedDict(
            "type" => "meshdata",
            "frame" => v.frame,
            "material" => jsonify(v.color),
            "vertices" => verts,
            "faces" => faces
        )
    elseif v.geometry isa MeshFromFile
        OrderedDict(
            "type" => "mesh",
            "frame" => v.frame,
            "material" => jsonify(v.color),
            "filename" => v.geometry.path 
        )
    elseif v.geometry isa Rect
        OrderedDict(
            "type" => "box",
            "frame" => v.frame,
            "material" => jsonify(v.color),
            "size" => v.geometry.widths 
        )
    elseif v.geometry isa Cylinder
        OrderedDict(
            "type" => "cylinder",
            "frame" => v.frame,
            "material" => jsonify(v.color),
            "radius" => v.geometry.radius,
            "length" => norm(v.geometry.origin - v.origin.extremity)
        )
    elseif v.geometry isa Sphere 
        OrderedDict(
            "type" => "sphere",
            "frame" => v.frame,
            "material" => jsonify(v.color),
            "radius" => v.geometry.r 
        )
    else
        rson_throw_exception("Unrecognised visual type '$(type)'", cfg)
    end
end