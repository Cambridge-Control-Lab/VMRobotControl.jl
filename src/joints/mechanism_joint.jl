######################
# Compiled Frame ID type
######################
abstract type AbstractCompiledIndex end
abstract type AbstractCompiledMechanismIndex <: AbstractCompiledIndex end
abstract type AbstractCompiledVirtualMechanismSystemIndex <: AbstractCompiledIndex end

struct CompiledFrameID <: AbstractCompiledMechanismIndex
    idx::Int
end
Base.to_index(i::CompiledFrameID) = i.idx
Base.show(io::IO, ::Type{CompiledFrameID}) = print(io, "CompiledFrameID")

struct VMSFrameID{S} <: AbstractCompiledVirtualMechanismSystemIndex
    idx::CompiledFrameID
    VMSFrameID{ON_ROBOT}(idx::CompiledFrameID) = new{ON_ROBOT}(idx)
    VMSFrameID{ON_VIRTUAL_MECHANISM}(idx::CompiledFrameID) = new{ON_VIRTUAL_MECHANISM}(idx)
    VMSFrameID{ON_SYSTEM}(idx::CompiledFrameID) = new{ON_SYSTEM}(idx)
end
_vms_location(::VMSFrameID{S}) where {S} = Val{S}()

const FrameID = Union{String, CompiledFrameID, VMSFrameID}

############
# MechanismJoint
############

"""
    MechanismJoint(joint, parentFrameID::Int, childFrameID::Int)

Contains jointdata, and the IDs of parent/child frames that the joint connects.

Before a mechanism is compiled, these frame IDs are strings---the name of the 
frame---, after compilation it is an integer, unique for each frame.

See also [`AbstractJointData`](@ref), [`add_joint!`](@ref)
"""
struct MechanismJoint{JD<:AbstractJointData, FID}
    jointData::JD
    parentFrameID::FID
    childFrameID::FID
end

jointdata_type(::Type{MechanismJoint{JD, FID}}) where {JD, FID} = JD
jointdata_type(::MJ) where MJ <: MechanismJoint = jointdata_type(MJ)

# Keyword constructor
function MechanismJoint(; jointData, parentFrameID, childFrameID)
    T = valuetype(jointData)
    JD = typeof(jointData)
    FID = typeof(parentFrameID)
    @assert typeof(childFrameID) == FID
    MechanismJoint{JD, FID}(jointData, parentFrameID, childFrameID)
end

remaker_of(j::MechanismJoint) = parameterless_type(j)

function _reassign_frames(mjoint::MechanismJoint, frameID_map::Dict)
    pfid = get_compiled_frameID(frameID_map, mjoint.parentFrameID)
    cfid = get_compiled_frameID(frameID_map, mjoint.childFrameID)
    remake(mjoint; parentFrameID = pfid, childFrameID = cfid)
end

function _reassign_joints(mjoint::MechanismJoint, joint_map::Dict)
    # remake(mjoint; jointID=get_compiled_joint(joint_map, mjoint.jointID))
    # Does nothing
    mjoint
end

config_size(::Type{MJ}) where MJ <:MechanismJoint = config_size(jointdata_type(MJ))
velocity_size(::Type{MJ}) where MJ <:MechanismJoint = config_size(jointdata_type(MJ))

config_size(j::MechanismJoint) = config_size(j.jointData)
velocity_size(j::MechanismJoint) = config_size(j.jointData)


#############################################

struct CompiledJointID{J} <: AbstractCompiledMechanismIndex
    idx::TypeStableIdx{J}
    # Note that if the joint has more than 1DOF, then q_idx is only the START of the joint
    # config vector. It's length is determined by the type of the joint/jointdata
    function CompiledJointID(joint_collection_idx::TypeStableIdx{J}) where J
        new{J}(joint_collection_idx)
    end
end

jointdata_type(::Type{CompiledJointID{J}}) where J = jointdata_type(J)
jointdata_type(::JID) where JID <: CompiledJointID = jointdata_type(JID)

Base.getindex(jc::TypeStableCollection, i::CompiledJointID) = jc[i.idx]
Base.setindex!(jc::TypeStableCollection, val::J, i::CompiledJointID{J}) where J = (jc[i.idx] = val)

struct VMSJointID{S, J} <: AbstractCompiledVirtualMechanismSystemIndex
    idx::CompiledJointID{J}
    VMSJointID{ON_ROBOT}(idx::CompiledJointID{T}) where T = new{ON_ROBOT, T}(idx)
    VMSJointID{ON_VIRTUAL_MECHANISM}(idx::CompiledJointID{T}) where T = new{ON_VIRTUAL_MECHANISM, T}(idx)
    VMSJointID{ON_SYSTEM}(idx::CompiledJointID{T}) where T = new{ON_SYSTEM, T}(idx)
end
_vms_location(::VMSJointID{S, J}) where {S, J} = Val{S}()
ndof(::Type{VMSJointID{S, J}}) where {S, J} = ndof(J)

const JointID = Union{String, CompiledJointID, VMSJointID}

struct CompiledMechanismJoint{T, JD}
    jointData::JD
    parentFrameID::CompiledFrameID
    childFrameID::CompiledFrameID
    q_idx::Int    # Where in the mechanism config vector (q) does the joint config vector start
    q̇_idx::Int    # Where in the mechanism velocity vector (q̇) does the joint velocity vector start
    function CompiledMechanismJoint(jointdata::AbstractJointData{T}, parentFrameID, childFrameID, q_idx, q̇_idx) where T
        new{T, typeof(jointdata)}(jointdata, parentFrameID, childFrameID, q_idx, q̇_idx)
    end
    function CompiledMechanismJoint{T, JD}(; jointData::JD, parentFrameID, childFrameID, q_idx, q̇_idx) where {T, JD}
        new{T, JD}(jointData, parentFrameID, childFrameID, q_idx, q̇_idx)
    end    
end

jointdata_type(::Type{CompiledMechanismJoint{T, JD}}) where {T, JD} = JD
jointdata_type(::CMJ) where CMJ <: CompiledMechanismJoint = jointdata_type(CMJ)

remaker_of(j::CompiledMechanismJoint{T, JD}) where {T, JD} = CompiledMechanismJoint{T, JD}

ndof(::Type{CompiledMechanismJoint{T, JD}}) where {T, JD} = config_size(JD)


# const CompiledMechanismJoint = MechanismJoint{T, J, JID, Int} where {T, J, JID<:CompiledJointID}

function q_idxs(j::CompiledMechanismJoint)
    N = config_size(j.jointData)
    start = j.q_idx
    stop  = j.q_idx + N
    return SVector{N, Int}(NTuple{N, Int}(start:stop))
end

function q̇_idxs(j::CompiledMechanismJoint)
    N = velocity_size(typeof(j.jointData))
    start = j.q̇_idx
    stop  = j.q̇_idx + N
    return SVector{N, Int}(NTuple{N, Int}(start:stop))
end

Base.show(io::IO, ::Type{CompiledMechanismJoint{T, JD}}) where {T, JD} = print(io, "CompiledMechanismJoint($T, $JD)")




######################
# Apply forces to frames
######################

# This must come after the definition of CompiledJointID... hence living here

function _apply_joint_force_to_frames!(bundle, jID::CompiledJointID, u::Real)
    # Axis in world frame 
    mj = bundle[jID]
    _apply_joint_force_to_frames!(bundle, mj.jointData, mj.parentFrameID, mj.childFrameID, u)
    nothing
end


function _apply_joint_force_to_frames!(
        bundle::CacheBundle, 
        jointData::RevoluteData, 
        parentFrameID::CompiledFrameID, 
        childFrameID::CompiledFrameID, 
        u::Real)
    # Axis in world frame 
    axis = rotor(get_transform(bundle, parentFrameID)) * jointData.axis
    get_frame_torques(bundle)[parentFrameID] -= axis * u
    get_frame_torques(bundle)[childFrameID] += axis * u
    nothing
end
