#########################
# Kinematics algorithms
#########################

function __kinematics!(transforms, jc, walk, t, qs,)
    foreach(jc) do joint
        q = qs[q_idxs(joint)]
        transforms[joint.childFrameID] = joint_relative_transform(joint.jointData, t, q)
        return nothing
    end
    if isempty(walk) # If no joints, still need to fill in the inertial frame
        transforms[1] = zero(transforms[1])
    else
        transforms[walk[1][1]] = zero(transforms[1])
        for (parent, child) in walk
            transforms[child] = transforms[parent] * transforms[child]
        end
    end
    return nothing
end

const JacobianArray{T} = Array{T, 3}

function __fill_jacobian_column!(
        jacobians::Array{<:Real, 3}, 
        joint_idxs::SVector{N, Int}, 
        frame::Int, 
        Jv_cols::SA, 
        Jw_cols::SA,) where {N, S, L, SA<:SArray{S, <:Real, L}}
    for (k, j::Int) in enumerate(joint_idxs)
        jacobians[1:3, j, frame] = Jv_cols[:, k]
        jacobians[4:6, j, frame] = Jw_cols[:, k]
    end
    return nothing
end

const TransformsContainer{T} = Union{Vector{Transform{T}}, Vector{RigidBodyState{T}}}

@inline function get_transform(tfc::TransformsContainer, i)
    if eltype(tfc) <: Transform
        return tfc[i]
    elseif eltype(tfc) <: RigidBodyState
        return transform(tfc[i])
    else
        error("Something went wrong...")
    end
end


function __jacobians_iter_frames!(
        jacobians::Array{<:Real, 3},
        mjoint::CompiledMechanismJoint,
        descendants::Vector{Vector{Int}},
        transforms::TransformsContainer,
        t::Real,
        q::AbstractVector{<:Real})
    frames = descendants[mjoint.childFrameID]
    for frame in frames
        joint_idxs = q_idxs(mjoint)
        Jv_cols, Jw_cols = jacobian_column(
            mjoint.jointData,
            get_transform(transforms, mjoint.parentFrameID),
            get_transform(transforms, frame),
            t,
            q[joint_idxs])
        __fill_jacobian_column!(jacobians, joint_idxs, frame, Jv_cols, Jw_cols)
    end
    return nothing
end

function __jacobians_iter_joints!(
        jacobians::Array{<:Real, 3}, 
        joint_collection::TypeStableCollection, 
        descendants::Vector{Vector{Int}},
        transforms::TransformsContainer, 
        t::Real, 
        q::AbstractVector{<:Real})
    
    foreach(joint_collection) do joint::CompiledMechanismJoint
        # TODO this check happens for every joint, when it only needs to be checked per type.
        # Maybe it is compiled away? worth checking
        if velocity_size(joint.jointData)::Int > 0
            __jacobians_iter_frames!(jacobians, joint, descendants, transforms, t, q)
        end
        return nothing
    end
    return nothing
end

function __compute_jacobians_fast!(
        jacobians::JacobianArray, 
        joint_collection::TypeStableCollection,
        descendants::Vector{Vector{Int}},
        transforms::TransformsContainer{<:Real},
        t::Real, 
        q::AbstractVector{<:Real})

    # fill!(jacobians, zero(eltype(jacobians)))

    # There are two possible approaches here. The first is to iterate through each joint, 
    # and for that joint iterate through each frame that descends from it, computing the
    # jacobian column for that joint/frame combo.
    # The second is to iterate through each frame, and for that frame iterate through each
    # joint which is it's ancestor.
    # We use the first, as it is more type stable: we use foreach on the JointCollection, which
    # is a tuple of vectors of different types. This allows us to dispatch to the correct
    # method for each joint type. If we used the second approach, we would have to dispatch
    # to the correct method for each joint type in the inner loop, which would nescessarily be 
    # less type stable.
    __jacobians_iter_joints!(jacobians, joint_collection, descendants, transforms, t, q)
    return nothing
end

function inertial_frame_state(T::Type,)
    # We choose not to use inertial frame acceleration to implement gravity, as it breaks
    # inerters. Instead, we use the fact that the inertial frame is always at rest.
    RigidBodyState(zero(Transform{T}), zero(Twist{T}), zero(SpatialAcceleration{T}))
end

function __velocity_kinematics!(
        rbstates::Vector{<:RigidBodyState{<:Real}}, 
        joint_collection::TypeStableCollection, 
        walk::Vector{Tuple{Int, Int}}, 
        t::Real, 
        qs::AbstractVector, 
        q̇s::AbstractVector, 
    )
    foreach(joint_collection) do joint
        jd = joint.jointData
        q = qs[q_idxs(joint)]
        q̇ = q̇s[q̇_idxs(joint)]
        rbstates[joint.childFrameID] = RigidBodyState(
            joint_relative_transform(jd, t, q),
            joint_relative_twist(jd, t, q, q̇),
            joint_relative_vpa(jd, t, q, q̇)
        )
        return nothing
    end

    T = valuetype(eltype(rbstates))
    if isempty(walk) # If no joints, still need to fill in the inertial frame
        rbstates[1] = inertial_frame_state(T)
    else
        rbstates[walk[1][1]] = inertial_frame_state(T)
        for (parent, child) in walk
            rbstates[child] = child_rbstate(rbstates[parent], rbstates[child])
        end
    end
    return nothing
end

function __acceleration_kinematics!(
        rbstates::Vector{<:RigidBodyState}, 
        joint_collection::TypeStableCollection, 
        walk::Vector{Tuple{Int, Int}},                           
        t::Real, 
        qs::AbstractVector, 
        q̇s::AbstractVector, 
        q̈s::AbstractVector, 
    )
    foreach(joint_collection) do joint
        jd = joint.jointData
        q = qs[q_idxs(joint)]
        q̇ = q̇s[q̇_idxs(joint)]
        q̈ = q̈s[q̇_idxs(joint)]
        rbstates[joint.childFrameID] = RigidBodyState(
            joint_relative_transform(jd, t, q),
            joint_relative_twist(jd, t, q, q̇),
            joint_relative_vpa(jd, t, q, q̇) + joint_relative_acceleration(jd, t, q, q̈)
        )
    end
    T = valuetype(eltype(rbstates))
    if isempty(walk) # If no joints, still need to fill in the inertial frame
        rbstates[1] = inertial_frame_state(T)
    else
        rbstates[walk[1][1]] = inertial_frame_state(T)
        for (parent, child) in walk
            rbstates[child] = child_rbstate(rbstates[parent], rbstates[child])
        end
    end
    return nothing
end

function __compute_hessianresult(jc, walk, t, q; warn=true)
    warn && @warn "Deprecated. Replace with precompute(m, q, q̇) or kinematics(m, q)"
    kinematics_closure = let
        Nf = length(walk) + 1
        function f_kinematics(q)
            transforms = Vector{Transform{eltype(q)}}(undef, Nf)
            __kinematics!(transforms, jc, walk, t, q)
            SVector{Nf}(transforms)
        end
    end
    Hessians.my_hessian(kinematics_closure, SVector{length(q)}(q))
end                 
compute_hessianresult(m, t, q; warn=true) = compute_hessianresult(m.rbtree, q, t; warn=warn)

function _autodiff_extended_kinematics(hessianresult, q, q̇)
    # WARNING. does not account for TimeFuncJoints.
    [
        begin
            tf = value(hessianresult)[i]
            J = jacobian(hessianresult)[i]
            H = hessian(hessianresult)[i]
            twist = Twist(tf, J, q̇)
            vpa = SpatialAcceleration(tf, J, H, q̇)
            RigidBodyState(tf, twist, vpa)
        end
        for i in eachindex(value(hessianresult))
    ]
end

function __autodiff_extended_kinematics(
        joint_collection::TypeStableCollection, 
        walk::Vector{Tuple{Int, Int}},
        t::Real,
        q::AbstractVector{<:Real},
        q̇::AbstractVector{<:Real})
    hessianresult = __compute_hessianresult(joint_collection, walk, t, q; warn=false)
    rbstates = _autodiff_extended_kinematics(hessianresult, q, q̇)
    return rbstates
end
