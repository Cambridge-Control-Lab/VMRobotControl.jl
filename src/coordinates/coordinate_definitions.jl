####################################################################################################
# CoordDifference
####################################################################################################

"""
    CoordDifference(parent, child)
    CoordDifference(;parent, child)

Represents the difference between two coordinates. For example if both coordinates are cartesian 
postions, then this represents the vector from one to the other.
"""
struct CoordDifference{ID1<:CoordID, ID2<:CoordID} <: CoordinateData
    parent::ID1
    child::ID2
    function CoordDifference(parent::ID1, child::ID2) where {ID1, ID2}
        C1, C2 = coord_type(ID1), coord_type(ID2)
        if ~ismissing(C1) && ~ismissing(C2)
            N1, N2 = length(C1), length(C2)
            if ~ismissing(N1) && ~ismissing(N2)
                # This check will only work for compiled coordinates.
                @assert N1 == N2 "Coordinates must have the same length to take the difference"
            end
        end
        new{ID1, ID2}(parent, child)
    end
end
# Keyword constructor
CoordDifference(;parent::ID1, child::ID2) where {ID1, ID2} = CoordDifference{ID1, ID2}(parent, child)

_reassign_frames(c::CoordDifference, frameID_map) = c
_reassign_joints(c::CoordDifference, joint_map) = c
function _reassign_coords(c::CoordDifference, coordID_map)
    parent = get_compiled_coordID(coordID_map, c.parent)
    child = get_compiled_coordID(coordID_map, c.child)
    CoordDifference(parent, child)
end
dependencies(c::CoordDifference) = [c.parent, c.child]
cache_size(c::Type{<:CoordDifference}) = length(c)
has_configuration(c::Type{CoordDifference{ID1, ID2}}) where {ID1, ID2} = has_configuration(coord_type(ID1)) && has_configuration(coord_type(ID2))

Base.length(::Type{CoordDifference{ID1, ID2}}) where {ID1, ID2} = length(coord_type(ID1))
Base.eltype(::Type{CoordDifference{ID1, ID2}}) where {ID1, ID2} = promote_type(eltype(coord_type(ID1)), eltype(coord_type(ID2)))

####################################################################################################
# CoordSum
####################################################################################################

"""
    CoordSum(c1, c2)
    CoordSum(;c1, 1child)

Represents the sum of two coordinates. 
"""
struct CoordSum{ID1<:CoordID, ID2<:CoordID} <: CoordinateData
    c1::ID1
    c2::ID2
    function CoordSum(c1::ID1, c2::ID2) where {ID1, ID2}
        C1, C2 = coord_type(ID1), coord_type(ID2)
        if ~ismissing(C1) && ~ismissing(C2)
            N1, N2 = length(C1), length(C2)
            if ~ismissing(N1) && ~ismissing(N2)
                # This check will only work for compiled coordinates.
                @assert N1 == N2 "Coordinates must have the same length to sum"
            end
        end
        new{ID1, ID2}(c1, c2)
    end
end
# Keyword constructor
CoordSum(;c1::ID1, c2::ID2) where {ID1, ID2} = CoordSum{ID1, ID2}(c1, c2)

_reassign_frames(c::CoordSum, frameID_map) = c
_reassign_joints(c::CoordSum, joint_map) = c
function _reassign_coords(c::CoordSum, coordID_map)
    c1 = get_compiled_coordID(coordID_map, c.c1)
    c2 = get_compiled_coordID(coordID_map, c.c2)
    CoordSum(c1, c2)
end
dependencies(c::CoordSum) = [c.c1, c.c2]
cache_size(c::Type{<:CoordSum}) = length(c)
has_configuration(c::Type{CoordSum{ID1, ID2}}) where {ID1, ID2} = has_configuration(coord_type(ID1)) && has_configuration(coord_type(ID2))

Base.length(::Type{CoordSum{ID1, ID2}}) where {ID1, ID2} = length(coord_type(ID1))
Base.eltype(::Type{CoordSum{ID1, ID2}}) where {ID1, ID2} = promote_type(eltype(coord_type(ID1)), eltype(coord_type(ID2)))


####################################################################################################
# CoordStack
####################################################################################################

"""
    CoordStack(c1, c2)
    CoordStack(;c1, c2)

Represents the vertical concatenation of two coordinates.
"""
struct CoordStack{ID1<:CoordID, ID2<:CoordID} <: CoordinateData
    c1::ID1
    c2::ID2
    function CoordStack(c1::ID1, c2::ID2) where {ID1, ID2}
        new{ID1, ID2}(c1, c2)
    end
end
# Keyword constructor
CoordStack(;c1::ID1, c2::ID2) where {ID1, ID2} = CoordStack(c1, c2)

_reassign_frames(c::CoordStack, frd) = c
_reassign_joints(c::CoordStack, jrd) = c
function _reassign_coords(c::CoordStack, coordID_map)
    c1 = get_compiled_coordID(coordID_map, c.c1)
    c2 = get_compiled_coordID(coordID_map, c.c2)
    ID1, ID2 = typeof(c1), typeof(c2)
    CoordStack(c1, c2)
end
dependencies(c::CoordStack) = [c.c1, c.c2]
cache_size(c::Type{<:CoordStack}) = length(c)
has_configuration(c::Type{<:CoordStack{ID1, ID2}}) where {ID1, ID2} = has_configuration(coord_type(ID1)) && has_configuration(coord_type(ID2))

Base.length(::Type{CoordStack{ID1, ID2}}) where {ID1, ID2} = length(coord_type(ID1)) + length(coord_type(ID2))
Base.eltype(::Type{CoordStack{ID1, ID2}}) where {ID1, ID2} = promote_type(eltype(coord_type(ID1)), eltype(coord_type(ID2)))

####################################################################################################
# CoordSlice
####################################################################################################

"""
    CoordSlice(coordid, idxs::SVector{N, Int})

A coordinate made by indexing into another coordinate.
"""
struct CoordSlice{N, ID<:CoordID} <: CoordinateData
    coord::ID
    idxs::SVector{N, Int}
    function CoordSlice(coord::ID, idxs::SVector{N, Int}) where {N, ID}
        if ~ismissing(coord_type(ID))
            for i in idxs
                @assert 1 <= i <= length(coord_type(ID)) "Index $i out of bounds for $(coord_type(ID))"
            end
        end
        new{N, ID}(coord, idxs)
    end
end
# Keyword constructor
CoordSlice(;coord::ID, idxs::SVector{N, Int}) where {N, ID} = CoordSlice{N, ID}(coord, idxs)

_reassign_frames(c::CoordSlice, frd) = c
_reassign_joints(c::CoordSlice, jrd) = c
function _reassign_coords(c::CoordSlice, coordID_map)
    new_id = get_compiled_coordID(coordID_map, c.coord)
    CoordSlice(new_id, c.idxs)
end
dependencies(c::CoordSlice) = [c.coord,]
cache_size(c::Type{<:CoordSlice}) = 0
has_configuration(::Type{<:CoordSlice{N, ID}}) where {N, ID} = has_configuration(coord_type(ID))

Base.length(::Type{CoordSlice{N, ID}}) where {N, ID} = N
Base.eltype(::Type{CoordSlice{N, ID}}) where {N, ID} = eltype(coord_type(ID))

####################################################################################################
# ConstCoord
####################################################################################################

"""
    ConstCoord(::SVector)

A constant coordinate. Can be used with coordinate differences to shift the equilibrium of a spring.
"""
@kwdef struct ConstCoord{N, T} <: CoordinateData
    val::SVector{N, T}
end
ConstCoord(v::Real) = ConstCoord(SVector{1, typeof(v)}(v))

_reassign_frames(c::ConstCoord, _) = c
_reassign_joints(c::ConstCoord, _) = c
_reassign_coords(c::ConstCoord, _) = c

dependencies(c::ConstCoord) = ()
cache_size(c::Type{<:ConstCoord}) = length(c)
has_configuration(c::Type{<:ConstCoord}) = true

Base.length(::Type{ConstCoord{N, T}}) where {N, T} = N
Base.eltype(::Type{ConstCoord{N, T}}) where {N, T} = T

####################################################################################################
# ReferenceCoord
####################################################################################################

"""
    ReferenceCoord(::Base.RefValue{SVector{N, T}})

A reference coordinate. By keeping a reference to a coordinate, it can be 
modified in place.
"""
@kwdef struct ReferenceCoord{N, T} <: CoordinateData
    val::Base.RefValue{SVector{N, T}}
    vel::Base.RefValue{SVector{N, T}}
end
ReferenceCoord(v::Ref{<:SVector}) = ReferenceCoord(v, Ref(zero(v[])))

_reassign_frames(c::ReferenceCoord, _) = c
_reassign_joints(c::ReferenceCoord, _) = c
_reassign_coords(c::ReferenceCoord, _) = c

dependencies(c::ReferenceCoord) = ()
cache_size(c::Type{<:ReferenceCoord}) = length(c)
has_configuration(c::Type{<:ReferenceCoord}) = true


Base.length(::Type{ReferenceCoord{N, T}}) where {N, T} = N
Base.eltype(::Type{ReferenceCoord{N, T}}) where {N, T} = T

####################################################################################################
# JointSubspace
####################################################################################################

"""
    JointSubspace(jointid)

Represents the configuration of the joint identified by `jointid`. 

When compiled, is replaced with the compiled mechanism joint representation.
This is because the joint type is needed to know the size of the joint.
"""
@kwdef struct JointSubspace{J<:JointID} <: CoordinateData
    joint::J
end

_reassign_frames(c::JointSubspace, _) = c
_reassign_joints(c::JointSubspace, jrd) = JointSubspace(get_compiled_joint(jrd, c.joint))
_reassign_coords(c::JointSubspace, _) = c

dependencies(c::JointSubspace) = ()
cache_size(c::Type{<:JointSubspace}) = length(c)
has_configuration(c::Type{<:JointSubspace}) = true

Base.length(::Type{JointSubspace{J}}) where J<:String = missing
Base.length(::Type{<:JointSubspace{J}}) where J<:CompiledJointID = config_size(jointdata_type(J))
Base.length(::Type{<:JointSubspace{J}}) where J<:VMSJointID = ndof(J)
Base.eltype(::Type{<:JointSubspace}) = missing # Eltype is determined by the mechanism

####################################################################################################
# FramePoint
####################################################################################################

"""
    FramePoint(frameID, point::SVector{3})

A point fixed in the frame designated by `frameID`. `point` is the position of
the point in the frame. The configuration of this coordinate is the vector to the point, 
represented in the root frame of the mechanism.
"""
@kwdef struct FramePoint{T, FID} <: CoordinateData
    frameID::FID
    point::SVector{3, T}
end

_reassign_frames(c::FramePoint, frameID_map) = FramePoint(get_compiled_frameID(frameID_map, c.frameID), c.point)
_reassign_joints(c::FramePoint, joint_map) = c
_reassign_coords(c::FramePoint, coordID_map) = c

dependencies(c::FramePoint) = ()
cache_size(c::Type{<:FramePoint}) = 3
has_configuration(c::Type{<:FramePoint}) = true

Base.length(::Type{FramePoint{T, FID}}) where {T, FID} = 3
Base.eltype(::Type{FramePoint{T, FID}}) where {T, FID} = T

####################################################################################################
# FrameOrigin
####################################################################################################

"""
    FrameOrigin(frameID)

The position of the origin of frame `frameID` in the world.
"""
@kwdef struct FrameOrigin{FID} <: CoordinateData
    frameID::FID
end

_reassign_frames(c::FrameOrigin, frameID_map) = FrameOrigin(get_compiled_frameID(frameID_map, c.frameID))
_reassign_joints(c::FrameOrigin, joint_map) = c
_reassign_coords(c::FrameOrigin, coordID_map) = c

dependencies(c::FrameOrigin) = ()
cache_size(c::Type{<:FrameOrigin}) = 3
has_configuration(c::Type{<:FrameOrigin}) = true

Base.length(::Type{FrameOrigin{FID}}) where FID = 3
Base.eltype(::Type{FrameOrigin{FID}}) where FID = missing

####################################################################################################
# QuaternionAttitude
####################################################################################################

"""
    QuaternionAttitude(frameID)

The rotation for frame `frameID` in the world, as the vector part of the 
the quaternion. Can be combined with a linear spring for continuous attitude
control.

See "Quaternion-based impedance with nondiagonal stiffness for robot 
manipulators" by Fabrizio Caccavale et al, 1998 IEEE ACC.
"""
@kwdef struct QuaternionAttitude{T, FID} <: CoordinateData
    frameID::FID
    # Rotor that represents the target orientation.
    # target_rotation * p rotates a point from target frame to world frame
    target_rotation::Rotor{T}
end
QuaternionAttitude{T}(frame::FrameID) where T = QuaternionAttitude{T, String}(frame, zero(Rotor{T}))

_reassign_frames(c::QuaternionAttitude, frameID_map) = QuaternionAttitude(get_compiled_frameID(frameID_map, c.frameID), c.target_rotation)
_reassign_joints(c::QuaternionAttitude, joint_map) = c
_reassign_coords(c::QuaternionAttitude, coordID_map) = c

dependencies(c::QuaternionAttitude) = ()
cache_size(c::Type{<:QuaternionAttitude}) = length(c)
has_configuration(c::Type{<:QuaternionAttitude}) = true


Base.length(::Type{QuaternionAttitude{T, FID}}) where {T, FID} = 3
Base.eltype(::Type{QuaternionAttitude{T, FID}}) where {T, FID} = T

####################################################################################################
# FrameAngularVelocity
####################################################################################################

"""
    FrameAngularVelocity(frameID)

The angular velocity for frame `frameID` in world frame. Used to implement
rotational inertia components..
"""
@kwdef struct FrameAngularVelocity{FID} <: CoordinateData
    frameID::FID
end

_reassign_frames(c::FrameAngularVelocity, frameID_map) = FrameAngularVelocity(get_compiled_frameID(frameID_map, c.frameID))
_reassign_joints(c::FrameAngularVelocity, joint_map) = c
_reassign_coords(c::FrameAngularVelocity, coordID_map) = c

dependencies(c::FrameAngularVelocity) = ()
cache_size(c::Type{<:FrameAngularVelocity}) = 3
has_configuration(c::Type{<:FrameAngularVelocity}) = false

Base.length(::Type{FrameAngularVelocity{FID}}) where FID = 3
Base.eltype(::Type{FrameAngularVelocity{FID}}) where FID = missing

####################################################################################################
#
####################################################################################################

"""
CoordNorm(coord)

Takes the norm of a coordinate `zⁱ`: i.e.
    z = ∥zⁱ∥ 
"""
@kwdef struct CoordNorm{C <: CoordID} <: CoordinateData
    coord::C
end

_reassign_frames(c::CoordNorm, frameID_map) = c
_reassign_joints(c::CoordNorm, joint_map) = c
_reassign_coords(c::CoordNorm, coordID_map) = CoordNorm(get_compiled_coordID(coordID_map, c.coord))

dependencies(c::CoordNorm) = (c.coord,)
cache_size(c::Type{<:CoordNorm}) = 1
has_configuration(c::Type{<:CoordNorm}) = true

Base.length(::Type{CoordNorm{C}}) where {C} = 1
Base.eltype(::Type{CoordNorm{C}}) where {C} = eltype(coord_type(C))

####################################################################################################
# RotatedCoord
####################################################################################################

"""
RotatedCoord(world_frame_coord, frameID)

Rotates world_frame_coordinate z⁽ʷ⁾ to the frame of link designated by frameID,
such that z = R⁻¹ z⁽ʷ⁾.
"""
@kwdef struct RotatedCoord{C<:CoordID, FID<:FrameID} <: CoordinateData
    world_frame_coord::C
    frameID::FID
end

_reassign_frames(c::RotatedCoord, frameID_map) = RotatedCoord(c.world_frame_coord, get_compiled_frameID(frameID_map, c.frameID))
_reassign_joints(c::RotatedCoord, joint_map) = c
_reassign_coords(c::RotatedCoord, coordID_map) = RotatedCoord(get_compiled_coordID(coordID_map, c.world_frame_coord), c.frameID)

dependencies(c::RotatedCoord) = (c.world_frame_coord,)
cache_size(c::Type{<:RotatedCoord}) = length(c)
has_configuration(c::Type{<:RotatedCoord}) = true

Base.length(::Type{RotatedCoord{C, FID}}) where {C, FID} = length(coord_type(C))
Base.eltype(::Type{RotatedCoord{C, FID}}) where {C, FID} = eltype(coord_type(C))

####################################################################################################
# UnrotatedCoord
####################################################################################################

"""
UnrotatedCoord(link_frame_coord, frameID)

Rotates link frame coordinate z⁽ˡ⁾ from the frame of link designated by frameID,
to the world frame, so that  z = Rʷˡ z⁽ˡ⁾.
"""
@kwdef struct UnrotatedCoord{C<:CoordID, FID<:FrameID} <: CoordinateData
    link_frame_coord::C
    frameID::FID
end

_reassign_frames(c::UnrotatedCoord, frameID_map) = UnrotatedCoord(c.link_frame_coord, get_compiled_frameID(frameID_map, c.frameID))
_reassign_joints(c::UnrotatedCoord, joint_map) = c
_reassign_coords(c::UnrotatedCoord, coordID_map) = UnrotatedCoord(get_compiled_coordID(coordID_map, c.link_frame_coord), c.frameID)

dependencies(c::UnrotatedCoord) = (c.link_frame_coord,)
cache_size(c::Type{<:UnrotatedCoord}) = length(c)
has_configuration(c::Type{<:UnrotatedCoord}) = true

Base.length(::Type{UnrotatedCoord{C, FID}}) where {C, FID} = length(coord_type(C))
Base.eltype(::Type{UnrotatedCoord{C, FID}}) where {C, FID} = eltype(coord_type(C))
