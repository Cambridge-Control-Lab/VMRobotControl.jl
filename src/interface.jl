function validate_args(m::CompiledMechanism; q=nothing, q̇=nothing, q̈=nothing, u=nothing)
    if ~isnothing(q)
        @assert length(q) == ndof(m) "Incorrect length of q for mechanism (got $(length(q)), expected $(ndof(m)))"
    end
    if ~isnothing(q̇)
        @assert length(q̇) == ndof(m) "Incorrect length of q̇ for mechanism (got $(length(q̇)), expected $(ndof(m)))"
    end
    if ~isnothing(q̈)
        @assert length(q̈) == ndof(m) "Incorrect length of q̈ for mechanism (got $(length(q̈)), expected $(ndof(m)))"
    end
    if ~isnothing(u)
        @assert length(u) == ndof(m) "Incorrect length of u for mechanism (got $(length(u)), expected $(ndof(m)))"
    end
end

function validate_args(vms::CompiledVirtualMechanismSystem; q=nothing, q̇=nothing, q̈=nothing, u=nothing)
    robot, vm = vms.robot, vms.virtual_mechanism
    if ~isnothing(q)
        @assert length(q[1]) == ndof(robot) "Incorrect length of q for robot (got $(length(q[1])), expected $(ndof(robot)))"
        @assert length(q[2]) == ndof(vm) "Incorrect length of q for virtual mechanism (got $(length(q[2])), expected $(ndof(vm)))"
    end
    if ~isnothing(q̇)
        @assert length(q̇[1]) == ndof(robot) "Incorrect length of q̇ for robot (got $(length(q̇[1])), expected $(ndof(robot)))"
        @assert length(q̇[2]) == ndof(vm) "Incorrect length of q̇ for virtual mechanism (got $(length(q̇[2])), expected $(ndof(vm)))"
    end
    if ~isnothing(q̈)
        @assert length(q̈[1]) == ndof(robot) "Incorrect length of q̈ for robot (got $(length(q̈[1])), expected $(ndof(robot)))"
        @assert length(q̈[2]) == ndof(vm) "Incorrect length of q̈ for virtual mechanism (got $(length(q̈[2])), expected $(ndof(vm)))"
    end
    if ~isnothing(u)
        @assert length(u[1]) == ndof(robot) "Incorrect length of u for robot (got $(length(u[1])), expected $(ndof(robot)))"
        @assert length(u[2]) == ndof(vm) "Incorrect length of u for virtual mechanism (got $(length(u[2])), expected $(ndof(vm)))"
    end
end

function compute_in_coordinate_order(f, coords::Vector{<:TypeStableCollection})
    for tsc in coords
        foreach(tsc) do coord
            f(coord)
        end
    end
    nothing
end

function compute_in_reverse_coordinate_order(f, coords::Vector{<:TypeStableCollection})
    for tsc in Iterators.reverse(coords)
        foreach(tsc) do coord
            f(coord)
        end
    end
    nothing
end


"""
    kinematics!(bundle, t, q)

Compute the kinematics/transforms for the system.

# Arguments
- `t` time
- `q` joint configuration vector

See also [`get_transform`](@ref)
"""
function kinematics! end

function _kinematics!(bundle::MechKinematicsBundle)
    # Actually compute the kinematics, without copying input arguments
    m, cache = bundle.mechanism, bundle.cache
    __kinematics!(cache.frame_cache.transforms, m.rbtree.joint_collection, m.rbtree.walk, cache.t[], cache.q)
    compute_in_coordinate_order(coordinates(m)) do coord
        __configuration!(bundle, coord)
    end
    return nothing
end

function kinematics!(bundle::MechKinematicsBundle, t, q)
    # Copy input arguments and then call _kinematics!
    m, cache = bundle.mechanism, bundle.cache
    validate_args(m; q)
    cache.t[] = t
    @assert length(q) == ndof(m) "Incorrect length of q for mechanism (got $(length(q)), expected $(ndof(m)))"
    copyto!(cache.q,  q)
    _kinematics!(bundle)
end

function kinematics!(bundle::VMSKinematicsBundle, t, q::Tuple)
    # Copy input arguments and then compute for each part of the system
    vms, cache = bundle.vms, bundle.cache
    validate_args(vms; q)
    cache.t[] = t
    @assert length(q[1]) == ndof(vms.robot) "Incorrect length of q ($(length(q[1]))) for robot (got $(length(q[1])), expected $(ndof(vms.robot)))"
    @assert length(q[2]) == ndof(vms.virtual_mechanism) "Incorrect length of q for virtual mechanism (got $(length(q[2])), expected $(ndof(vms.virtual_mechanism)))"
    copyto!(cache.q[1],  q[1])
    copyto!(cache.q[2],  q[2])

    _kinematics!(robot_cache(bundle))
    _kinematics!(virtual_mechanism_cache(bundle))
    compute_in_coordinate_order(coordinates(vms)) do coord
         __configuration!(bundle, coord)
    end
    return nothing
end


"""
    jacobians!(bundle, t, q)

Compute the Jacobians for the mechanism at time `t` and configuration `q` and store them in
`cache.jacobians`.
"""
function jacobians!(bundle::MechJacobiansBundle, t, q)
    m, cache = bundle.mechanism, bundle.cache
    validate_args(m; q)
    cache.t[] = t
    copyto!(cache.q, q)

    transforms = cache.frame_cache.transforms
    __kinematics!(transforms, m.rbtree.joint_collection, m.rbtree.walk, cache.t[], cache.q)
    jacobians = cache.frame_cache.jacobians
    __compute_jacobians_fast!(jacobians, m.rbtree.joint_collection, m.rbtree.descendants,
                              transforms, cache.t[], cache.q)
    compute_in_coordinate_order(coordinates(m)) do coord
        __configuration!(bundle, coord)
        __jacobian!(bundle, coord)
    end
    return nothing
end

"""
    acceleration_kinematics!(bundle, t, q, q̇, q̈)

Compute the accelerations for the mechanism with known joint-accelerations `q̈`.

Also computes the kinematics/velocities as prerequisites.

# Arguments
- `t` time
- `q` joint configuration vector
- `q̇` joint velocity vector
- `q̈` joint acceleration vector
"""
function acceleration_kinematics! end

function _acceleration_kinematics!(bundle::Union{MechRBStatesBundle, MechRNEBundle})
    m, cache = bundle.mechanism, bundle.cache
    __acceleration_kinematics!(cache.frame_cache.rbstates, m.rbtree.joint_collection, m.rbtree.walk,
                               cache.t[], cache.q, cache.q̇, cache.q̈)
    compute_in_coordinate_order(coordinates(m)) do coord
        __configuration!(bundle, coord)
        __velocity!(bundle, coord)
        __acceleration!(bundle, coord)
    end
    return nothing
end

function acceleration_kinematics!(bundle::Union{MechRBStatesBundle, MechRNEBundle}, t, q, q̇, q̈)
    m, cache = bundle.mechanism, bundle.cache
    validate_args(m; q, q̇, q̈)
    cache.t[] = t
    copyto!(cache.q, q)
    copyto!(cache.q̇, q̇)
    copyto!(cache.q̈, q̈)
    _acceleration_kinematics!(bundle)
    return nothing
end

"""
    velocity_kinematics!(cache, t, q, q̇)

Compute the velocities of the system at time `t` and configuration `q`, with
velocities `q̇`, and store them in `cache.rbstates`. Also computes the velocity
product accelerations (i.e. accelerations assuming q̈ = 0).

Also computes the kinematics as a prerequisite.
"""
function velocity_kinematics! end

function _velocity_kinematics!(bundle::Union{MechRBStatesBundle, MechDynamicsBundle})
    # Actually compute the velocity kinematics, without copying input arguments
    m, cache = bundle.mechanism, bundle.cache
    validate_args(m; cache.q, cache.q̇)
    __velocity_kinematics!(cache.frame_cache.rbstates, m.rbtree.joint_collection, m.rbtree.walk, 
                           cache.t[], cache.q, cache.q̇)

    compute_in_coordinate_order(coordinates(m)) do coord
        __configuration!(bundle, coord)
        __velocity!(bundle, coord)
        __acceleration!(bundle, coord)
    end
end

function velocity_kinematics!(bundle::Union{MechRBStatesBundle, MechDynamicsBundle}, t, q, q̇)
    # Copy input arguments and then call _velocity_kinematics!
    m, cache = bundle.mechanism, bundle.cache
    validate_args(m; q, q̇)
    cache.t[] = t
    copyto!(cache.q, q)
    copyto!(cache.q̇, q̇)
    _velocity_kinematics!(bundle)
    return nothing
end

"""
    precompute!(cache, t, q, q̇, gravity, u)

Precompute the necessary quantities for the dynamics of the mechanism at time `t` and configuration 
`q`, with velocities `q̇`, gravitational acceleration `gravity`, and generalized forces `u`.

This includes the velocity kinematics and Jacobians, and stores the results in `cache`. 
The accelerations q̈ are not computed.
"""
function precompute! end

function _precompute!(bundle::MechDynamicsBundle)
    _velocity_kinematics!(bundle)
    m, cache = bundle.mechanism, bundle.cache
    __compute_jacobians_fast!(cache.frame_cache.jacobians, m.rbtree.joint_collection, m.rbtree.descendants, 
                              cache.frame_cache.rbstates, cache.t[], cache.q)   
    compute_in_coordinate_order(coordinates(m)) do coord
        __jacobian!(bundle, coord)
    end
end

function precompute!(bundle::MechDynamicsBundle, t, q, q̇, gravity)
    m, cache = bundle.mechanism, bundle.cache
    validate_args(m; q, q̇)
    cache.t[] = t
    copyto!(cache.q, q)
    copyto!(cache.q̇, q̇)
    cache.gravity[] = gravity
    _precompute!(bundle)
end

function _precompute!(bundle::VMSDynamicsBundle)
    _precompute!(robot_cache(bundle))
    _precompute!(virtual_mechanism_cache(bundle))
    vms, cache = bundle.vms, bundle.cache
    compute_in_coordinate_order(coordinates(vms)) do coord
        __configuration!(bundle, coord)
        __velocity!(bundle, coord)
        __acceleration!(bundle, coord)
        __jacobian!(bundle, coord)
    end
end

function precompute!(bundle::VMSDynamicsBundle, t, q::Tuple, q̇::Tuple, gravity)
    vms, cache = bundle.vms, bundle.cache
    validate_args(vms; q, q̇)
    cache.t[] = t
    Nr, Nv = ndof(vms.robot), ndof(vms.virtual_mechanism)
    copyto!(cache.q[1], q[1])
    copyto!(cache.q[2], q[2])
    copyto!(cache.q̇[1], q̇[1])
    copyto!(cache.q̇[2], q̇[2])
    cache.gravity[] = gravity
    _precompute!(bundle)
    return nothing
end

function _inertance_matrix!(bundle::MechDynamicsBundle)
    # Assumes jacobians etc. have been computed
    m, cache = bundle.mechanism, bundle.cache
    M = get_inertance_matrix(cache)
    fill!(M, zero(eltype(M)))
    inertance_matrix!(M, bundle, get_inertance_components(m))
    
    # Now symmetricize
    # Mw = inertance_matrix_workspace(cache)
    # @turbo for i ∈ axes(M, 1), j ∈ axes(M, 2)
    #     Mw[j, i] = M[i, j]    
    # end
    # @assert norm(M - M') == 0.0
    M
end

function _inertance_matrix!(bundle::VirtualMechanismSystemCacheBundle)
    # TODO consider the interface here. One possible way to return these
    # types is as a sparse block diagonal matrix. Fundamentally, the data
    # returned would be the same, but it would represent the inertance_matrix
    # of the combined system, and allow operations to be performed on the
    # combined system.
    Mʳ = _inertance_matrix!(robot_cache(bundle)) 
    Mᵛ = _inertance_matrix!(virtual_mechanism_cache(bundle))
    Mʳ, Mᵛ
end

"""
    inertance_matrix!(cache, t, q)

Compute the inertance matrix for the mechanism at time `t` and configuration `q`, and store the 
results in the cache.

In general, [`dynamics!`](@ref) should be used instead of this function, which computes the 
inertance_matrix and other useful quantities in one call.
"""
function inertance_matrix!(bundle::MechDynamicsBundle, t, q)
    cache_t, cache_q = get_t(bundle), get_q(bundle)
    cache_t[] = t
    copyto!(cache_q, q)
    _precompute!(bundle)
    _inertance_matrix!(bundle)
end

function _generalized_force!(bundle::MechDynamicsBundle)
    τ = get_generalized_force(bundle)
    fill!(τ, zero(eltype(τ)))
    generalized_force!(τ, bundle, get_force_components(bundle.mechanism))
    τ
end

function generalized_force!(bundle::MechDynamicsBundle, t, q, q̇, gravity)
    cache_t, cache_q, cache_q̇, cache_gravity = get_t(bundle), get_q(bundle), get_q̇(bundle), get_gravity(bundle)
    cache_t[] = t
    copyto!(cache_q, q)
    copyto!(cache_q̇, q̇)
    cache_gravity[] = gravity
    _precompute!(bundle)
    _generalized_force!(bundle)
end

function _generalized_force!(bundle::VirtualMechanismSystemCacheBundle)
    fʳ = _generalized_force!(robot_cache(bundle))             # This zeros the generalized force, before adding to it
    fᵛ = _generalized_force!(virtual_mechanism_cache(bundle)) # This zeros the generalized force, before adding to it
    generalized_force!((fʳ, fᵛ), bundle, components(bundle.vms)) # This adds to the generalized force
    fʳ, fᵛ
end


"""
    generalized_force!(cache, t, q, q̇, gravity)

Compute the generalized forces for the mechanism at time `t` and configuration `q`, with velocities `q̇`,
gravitational acceleration `gravity`, and store the results in the cache.

In general, [`dynamics!`](@ref) should be used instead of this function, which computes the 
generalized forces and other useful quantities in one call.
"""
function generalized_force!(bundle::VirtualMechanismSystemCacheBundle, t, q, q̇, gravity)
    cache_t, cache_q, cache_q̇, cache_gravity = get_t(bundle), get_q(bundle), get_q̇(bundle), get_gravity(bundle)
    cache_t[] = t
    copyto!(cache_q[1], q[1])
    copyto!(cache_q[2], q[2])
    copyto!(cache_q̇[1], q̇[1])
    copyto!(cache_q̇[2], q̇[2])
    cache_gravity[] = gravity
    _precompute!(bundle)
    _generalized_force!(bundle)
end



"""
    dynamics!(cache, t, q, q̇, gravity, [u])

Compute the accelerations for the mechanism at time `t` and configuration `q`, with velocities `q̇`,
gravitational acceleration `gravity`, and generalized forces `u`, and store the results in the 
cache.

If `u` is not provided, it is assumed to be zero.
"""
function dynamics! end

function _solve_dynamics_cholesky!(q̈::Vector, M, f, u, M_workspace::Matrix, τ_workspace::Vector)
    # Solve for q̈ = M_workspace \ f_workspace
    # M_workspace must be positive definite
    # Writes over M_workspace, f_workspace
    copyto!(M_workspace, M)
    τ_workspace .= f .+ u 
    factorization = cholesky!(M_workspace; check=false)
    ldiv!(q̈, factorization, τ_workspace)
    q̈
end

function _dynamics!(bundle::MechDynamicsBundle) 
    _precompute!(bundle)
    M = _inertance_matrix!(bundle)
    f = _generalized_force!(bundle)
        
    # # Working non-allocating version, broken on enzyme
    # q̈_svec = inv(M) * (f + SVector{ndof(m)}(cache.u))
    # copyto!(cache.q̈, q̈_svec)

    # # Alternative that probably does the same??
    # q̈_svec = M \ (f + SVector{ndof(m)}(cache.u))
    # copyto!(cache.q̈, q̈_svec)

    # # This alternate version works for enzyme
    # copyto!(cache.q̈, cache.inertance_matrix \ (cache.generalized_force + cache.u))
    
    # # Using LinearSolve, still allocates
    # prob = LinearProblem(cache.inertance_matrix, cache.generalized_force + cache.u)
    # sol = solve(prob)
    # copyto!(cache.q̈, sol)

    # Using cholesky!
    q̈ = get_q̈(bundle)
    u = get_u(bundle)
    M_w, τ_w = get_inertance_matrix_workspace(bundle), get_generalized_force_workspace(bundle)
    _solve_dynamics_cholesky!(q̈, M, f, u, M_w, τ_w)
    get_q̈(bundle)
end

function dynamics!(bundle::MechDynamicsBundle, t, q, q̇, gravity, u=nothing)
    t_cache, q_cache, q̇_cache, g_cache, u_cache, = 
        get_t(bundle), get_q(bundle), get_q̇(bundle), get_gravity(bundle), get_u(bundle)

    t_cache[] = t
    copyto!(q_cache, q)
    copyto!(q̇_cache, q̇)
    g_cache[] = gravity
    if isnothing(u)
        fill!(u_cache, zero(eltype(u_cache)))
    else
        copyto!(u_cache, u)
    end
    _dynamics!(bundle)
end

function _dynamics!(bundle::VMSDynamicsBundle)
    _precompute!(bundle)
    robot_bundle = robot_cache(bundle)
    virtual_mechanism_bundle = virtual_mechanism_cache(bundle)
    Mʳ, Mᵛ = _inertance_matrix!(bundle)
    fʳ, fᵛ = _generalized_force!(bundle)
    q̈ = get_q̈(bundle)
    u = get_u(bundle)
    _solve_dynamics_cholesky!(q̈[1], Mʳ, fʳ, u[1], 
        get_inertance_matrix_workspace(robot_bundle), get_generalized_force_workspace(robot_bundle))
    _solve_dynamics_cholesky!(q̈[2], Mᵛ, fᵛ, u[2], 
        get_inertance_matrix_workspace(virtual_mechanism_bundle), get_generalized_force_workspace(virtual_mechanism_bundle))
    q̈
end

function dynamics!(bundle::VMSDynamicsBundle, t, q, q̇, gravity, u=nothing)
    t_cache, q_cache, q̇_cache, g_cache, u_cache, = 
        get_t(bundle), get_q(bundle), get_q̇(bundle), get_gravity(bundle), get_u(bundle)
    t_cache[] = t
    copyto!(q_cache[1], q[1])
    copyto!(q_cache[2], q[2])
    copyto!(q̇_cache[1], q̇[1])
    copyto!(q̇_cache[2], q̇[2])
    g_cache[] = gravity
    if isnothing(u)
        fill!(u_cache[1], zero(eltype(u_cache[1])))
        fill!(u_cache[2], zero(eltype(u_cache[2])))
    else
        copyto!(u_cache[1], u[1])
        copyto!(u_cache[2], u[2])
    end
    _dynamics!(bundle)
end

#===============================================================================
                Recursive Newton-Euler Inverse Dynamics
================================================================================
 This should be implemented based using the acceleration kinematics 
 algorithm for the forward pass, opspace force computation for the
 backward pass.

 There are two main challenges to implementing RNEA in this package.
 - one must create an interface to relate component opspace forces to 
   forces/torques on each frame
 - one must create an interface to transform forces/torques in the backwards
   pass, and project them into generalized forces on each joint.

 To do so, we will need to create a new type of cache, which stores the
 forces and torques on each frame, and the generalized forces on each joint.

 Then, a function such as apply_frame_wrenches!(cache, component) will add
 the forces and torques for a component to the cache, based upon the
 component's coordinate.

 In the cache, wrenches will be stored in the robots frame, and methods for 
 transforming wrenches will be required for the backwards pass.

 We will also need to assess the implementation of the joint jacobia, to see 
 if we can avoid duplication when projecting forces/torques from the frame
 to the joint.
===============================================================================#
function inverse_dynamics! end

function _inverse_dynamics!(bundle::MechRNEBundle)   
    _inverse_dynamics_forward_pass!(bundle)
    _inverse_dynamics_zero!(bundle)
    _inverse_dynamics_backward_pass_a!(bundle)
    _inverse_dynamics_backward_pass_b!(bundle)
    _inverse_dynamics_backward_pass_c!(bundle)
    _inverse_dynamics_backward_pass_d!(bundle)
    nothing
end

function _inverse_dynamics_set_inputs!(bundle, t, q, q̇, q̈, gravity)!
    t_cache, q_cache, q̇_cache, q̈_cache, g_cache, u_cache, = get_t(bundle), get_q(bundle), get_q̇(bundle), get_q̈(bundle), get_gravity(bundle), get_u(bundle)

    t_cache[] = t
    copyto!(q_cache, q)
    copyto!(q̇_cache, q̇)
    copyto!(q̈_cache, q̈)
    g_cache[] = gravity
end

function _inverse_dynamics_forward_pass!(bundle)
    _acceleration_kinematics!(bundle)
    m, cache = bundle.mechanism, bundle.cache
    compute_in_coordinate_order(coordinates(m)) do coord
        __configuration!(bundle, coord)
        __velocity!(bundle, coord)
        __acceleration!(bundle, coord)
    end
    nothing
end

function _inverse_dynamics_zero!(bundle)
    u = get_u(bundle)
    frame_forces, frame_torques = get_frame_forces(bundle), get_frame_torques(bundle)   
    # Zero all coordinate/frame forces and torques.
    fill!(bundle.cache.coord_cache.f, zero(eltype(bundle.cache.coord_cache.f)))
    fill!(frame_forces, zero(eltype(frame_forces)))
    fill!(frame_torques, zero(eltype(frame_torques)))
    fill!(u, zero(eltype(u)))
    nothing
end

function _inverse_dynamics_backward_pass_a!(bundle)
    # Add forces for each component to coordinates
    foreach(get_force_components(bundle.mechanism)) do component 
        _add_opspace_force!(bundle, component)
    end
    # Now the force for each coordinate with a component *directly* attached to it is set
    nothing
end

function _inverse_dynamics_backward_pass_b!(bundle)
    # Propagate forces through coordinates.
    compute_in_reverse_coordinate_order(coordinates(bundle.mechanism)) do coord 
        __propagate_opspace_force!(bundle, coord)
    end
    # Now, *frame* forces/torques contain the force/torque applied to each frame by the components
    nothing
end

function _inverse_dynamics_backward_pass_c!(bundle)
    frame_forces, frame_torques = get_frame_forces(bundle), get_frame_torques(bundle)   
    # propagate forces through frames/joints
    for (parent, child) in Iterators.reverse(bundle.mechanism.rbtree.walk) # TODO check this order is correct... might not work
        frame_forces[parent] += frame_forces[child]
        δ = origin(get_transform(bundle, CompiledFrameID(child))) - origin(get_transform(bundle, CompiledFrameID(parent)))
        frame_torques[parent] += frame_torques[child] + cross(δ, frame_forces[child])
    end
end

function _inverse_dynamics_backward_pass_d!(bundle)
    u = get_u(bundle)
    frame_forces, frame_torques = get_frame_forces(bundle), get_frame_torques(bundle)   
    # Project forces to joints
    foreach(joints(bundle.mechanism)) do joint
        length(q_idxs(joint)) == 0 && return nothing
        p, c = joint.parentFrameID, joint.childFrameID
        tf_p, tf_c = get_transform(bundle, p), get_transform(bundle, c)
        f_p, τ_p, f_c, τ_c = frame_forces[p], frame_torques[p], frame_forces[c], frame_torques[c]
        u[q_idxs(joint)] .= _project_forces_to_jointspace(joint.jointData, tf_p, tf_c, f_p, f_c, τ_p, τ_c)
        nothing
    end
end

function inverse_dynamics!(bundle::MechRNEBundle, t, q, q̇, q̈, gravity)
    _inverse_dynamics_set_inputs!(bundle, t, q, q̇, q̈, gravity)
    _inverse_dynamics!(bundle)
    get_u(bundle)
end

function _RNE_inertance_matrix!(M::Matrix, bundle::MechRNEBundle, t, q)
    N = ndof(bundle.mechanism)
    @assert size(M) == (N, N)

    q̇ = get_q̇(bundle)
    q̈ = get_q̈(bundle)
    g = get_gravity(bundle)[]
    for j = 1:N
        # Zero velocity and gravity
        fill!(q̇, zero(eltype(q̇)))
        g = zero(g)
        # Set acceleration for one joint
        fill!(q̈, zero(eltype(q̈)))
        q̈[j] = 1.0
        # Compute inverse dynamics, on inertance/generic components only.
        u = begin
            _inverse_dynamics_set_inputs!(bundle, t, q, q̇, q̈, g)
            _inverse_dynamics_forward_pass!(bundle)
            _inverse_dynamics_zero!(bundle)
            # WARNING any generic components which applies a force other than due
            # to an inertance will break this computation.
            inertance_components = get_inertance_components(bundle.mechanism)
            foreach(inertance_components) do component 
                _add_opspace_force!(bundle, component)
            end
            _inverse_dynamics_backward_pass_b!(bundle)
            _inverse_dynamics_backward_pass_c!(bundle)
            _inverse_dynamics_backward_pass_d!(bundle)
            get_u(bundle)
        end
        M[:, j] .= u
    end
    M
end


#==============================================================================#
const VMS_CONTROL_NOTE = """
!!! note
    Components attached to the *robot* will *NOT* be used for control. 
    The components on the robot represent the *model* of the robot, and are not considered part of 
    the controller. If you wish to control the robot, you must add components to the *`VirtualMechanismSystem`*.
    If your component uses a coordinate from the robot, you can add it to the `VirtualMechanismSystem`
    and refer to the coordinate using the id `".robot.<coord_id>"`
"""

"""
    control_step!(bundle, t, qʳ, q̇ʳ)

Perform a step of the control loop for the system.
    
Inputs are the current time `t`, the robot joint configuration `qʳ`, and the robot joint velocity 
`q̇ʳ`.

The elapsed time `dt` is computed from the previous time  and `t`. The virtual mechanism state is updated
using a forward Euler step with the computed acceleration `q̈ᵛ` and the elapsed time `dt`.

$VMS_CONTROL_NOTE

Returns the torques to be applied to the robot.

See also [`control_step!`](@ref)
"""
function control_step! end

function control_step!(bundle::VMSDynamicsBundle, t_in, qʳ_in, q̇ʳ_in)
    vms, cache = bundle.vms, bundle.cache
    
    t = get_t(bundle)
    qʳ, qᵛ = get_q(bundle)
    q̇ʳ, q̇ᵛ = get_q̇(bundle)
    uʳ, uᵛ = get_u(bundle)

    dt = t_in - t[]
    @assert dt >= 0 "dt of less than zero is not allowed, got $(t_in - t[])"
    t[] = t_in
    copyto!(qʳ, qʳ_in)
    copyto!(q̇ʳ, q̇ʳ_in)

    _precompute!(bundle)
    robot = vms.robot
    virtual_mechanism = vms.virtual_mechanism
    robot_bundle = robot_cache(bundle)
    virtual_mechanism_bundle = virtual_mechanism_cache(bundle)
    Mᵛ = _inertance_matrix!(virtual_mechanism_bundle)
    
    # Zero generalized forces applied to robot
    fʳ = get_generalized_force(robot_bundle)
    fill!(fʳ, zero(eltype(fʳ)))
    # DO NOT compute _generalized_force!(robot_bundle), as this represents forces due to components
    # that exist on the actual robot. We do not need to apply these forces again!
    # Instead, we only need to compute the forces on the virtual mechanism...
    fᵛ = _generalized_force!(virtual_mechanism_bundle)
    # ...and the add the forces due to the interconnecting storages/dissipations
    generalized_force!((fʳ, fᵛ), bundle, components(vms)) # This adds to fʳ and fᵛ
    
    # Now we add to the generalized forces the additional control input,
    # u, as set by the user with f_control
    uʳ, uᵛ = get_u(bundle)
    fʳ .+= uʳ
    fᵛ .+= uᵛ

    # Solve VM accelerationand update VM state
    _, q̈ᵛ = get_q̈(bundle)
    _solve_dynamics_cholesky!(q̈ᵛ, Mᵛ, fᵛ, uᵛ, get_inertance_matrix_workspace(virtual_mechanism_bundle), get_generalized_force_workspace(virtual_mechanism_bundle))
    for i in eachindex(cache.q[2])
        qᵛ[i] += q̇ᵛ[i] * dt
        q̇ᵛ[i] += q̈ᵛ[i] * dt
    end
    
    # Return the torques for the robot
    return fʳ
end


"""
    new_kinematics_cache(m[, T])

Creates a cache with element type `::T` (defaulting to the eltype of the mechanism) for performing 
kinematics computations for a compiled mechanism or compiled virtual mechanism system. 

This cache supports computation of frame transforms and coordinate [`configuration`](@ref)'s, but
do NOT support computing jacobians or velocities.

This type of cache is commonly used for plotting, where only the frame transforms are needed.

See also [`kinematics!`](@ref), [`new_jacobians_cache`](@ref), [`new_rbstates_cache`](@ref), 
[`new_dynamics_cache`](@ref), 
"""
function new_kinematics_cache end

"""
    new_jacobians_cache(m[, T])

Creates a cache with element type `::T` (defaulting to the eltype of the mechanism) for performing
kinematics and jacobian computations for a compiled mechanism or compiled virtual mechanism system.

This cache supports computation of frame transforms and jacobians, [`configuration`](@ref) and 
[`jacobian`](@ref) for coordinates, but do NOT support velocities.

See also [jacobians!](@ref), [`new_rbstates_cache`](@ref), [`new_dynamics_cache`](@ref), 
"""
function new_jacobians_cache end

"""
    new_rbstates_cache(m[, T])

Creates a cache with element type `::T` (defaulting to the eltype of the mechanism) for performing
kinematics and velocity computations for a compiled mechanism or compiled virtual mechanism system.

This cache supports computation of frame transforms, velocities and accelerations.
Depending on which computation is performed, [`velocity_kinematics!`](@ref) or 
[`acceleration_kinematics!`](@ref), the acceleration will either be a true acceleration dependant
upon joint accelerations q̈, or the velocity product acceleration, which is the acceleration assuming
q̈ = 0.
It supports [`configuration`](@ref), [`velocity`](@ref), and [`acceleration`](@ref), for coordinates.
It does NOT support jacobians or generalized forces/torques.

See also [`velocity_kinematics!`](@ref) [`new_dynamics_cache`](@ref), 
"""
function new_rbstates_cache end

"""
    new_dynamics_cache(m[, T])

Creates a cache with element type `::T` (defaulting to the eltype of the mechanism) for performing
kinematics, jacobian, and velocity computations for a compiled mechanism or compiled virtual mechanism system,
as well as generalized forces/torques from components, the inertance matrix, of the system, and
the acceleration of the system, q̈.

This cache supports computation of frame transforms, velocities, velocity-product-accelerations, and
jacobians.
It also supports [`configuration`](@ref), [`velocity`](@ref), [`acceleration`](@ref), and [`jacobian`](@ref)
for coordinates.

See also [`dynamics!`](@ref)
"""
function new_dynamics_cache end


# TODO
function new_inverse_dynamics_cache end

"""
    new_control_cache(m[, T])

Creates a cache with element type `::T` (defaulting to the eltype of the mechanism) for controlling
a robot with a virtual mechanism controller.

$VMS_CONTROL_NOTE

See also [`control_step!`](@ref)
"""
function new_control_cache end

# Compiled mechanisms cache bundles
new_kinematics_cache(m::CompiledMechanism; initialize::Bool=true) = new_kinematics_cache(m, eltype(m); initialize)
new_jacobians_cache(m::CompiledMechanism; initialize::Bool=true) = new_jacobians_cache(m, eltype(m); initialize)
new_rbstates_cache(m::CompiledMechanism; initialize::Bool=true) = new_rbstates_cache(m, eltype(m); initialize)
new_dynamics_cache(m::CompiledMechanism; initialize::Bool=true) = new_dynamics_cache(m, eltype(m); initialize)
new_inverse_dynamics_cache(m::CompiledMechanism; initialize::Bool=true) = new_inverse_dynamics_cache(m, eltype(m); initialize)

function new_kinematics_cache(m::CompiledMechanism, ::Type{T}; initialize::Bool=true) where T 
    cache = MechKinematicsCache{T}(m)
    bundle = MechanismCacheBundle(m, cache)
    # As the kinematics cache is used for plotting, it would be confusing if when we plot with 
    # a fresh cache, things look broken because the cache is full of invalid data. Therefore
    # the default behaviour is to initialize the cache by calling `kinematics!` once.
    initialize && kinematics!(bundle, zero(T), zero_q(m)) 
    bundle
end

function new_jacobians_cache(m::CompiledMechanism, ::Type{T}; initialize::Bool=true) where T
    cache = MechJacobiansCache{T}(m)
    bundle = MechanismCacheBundle(m, cache)
    initialize && jacobians!(bundle, zero(T), zero_q(m))
    bundle
end

function new_rbstates_cache(m::CompiledMechanism, ::Type{T}; initialize::Bool=true) where T
    cache = MechRBStatesCache{T}(m)
    bundle = MechanismCacheBundle(m, cache)
    initialize && velocity_kinematics!(bundle, zero(T), zero_q(m), zero_q̇(m))
    bundle
end

function new_dynamics_cache(m::CompiledMechanism, ::Type{T}; initialize::Bool=true) where T
    cache = MechDynamicsCache{T}(m)
    bundle = MechanismCacheBundle(m, cache)
    initialize && dynamics!(bundle, zero(T), zero_q(m), zero_q̇(m), DEFAULT_GRAVITY, zero_u(m))
    bundle
end

function new_inverse_dynamics_cache(m::CompiledMechanism, ::Type{T}; initialize::Bool=true) where T
    cache = MechRNECache{T}(m)
    bundle = MechanismCacheBundle(m, cache)
    initialize && inverse_dynamics!(bundle, zero(T), zero_q(m), zero_q̇(m), zero_q̈(m), DEFAULT_GRAVITY)
    bundle
end

# VirtualMechanismSystem cache bundles
function new_kinematics_cache(vms::CVMSystem, ::Type{T}; initialize::Bool=true) where T
    cache = VMSKinematicsCache{T}(vms)
    bundle = VirtualMechanismSystemCacheBundle(vms, cache)
    if initialize
        kinematics!(bundle, 0.0, zero_q(T, vms))
    end
    bundle
end
new_kinematics_cache(vms::CVMSystem; initialize::Bool=true) = new_kinematics_cache(vms, eltype(vms))

new_dynamics_cache(vms::CVMSystem, ::Type{T}) where T = VirtualMechanismSystemCacheBundle(vms, VMSDynamicsCache{T}(vms))
new_dynamics_cache(vms::CVMSystem) = new_dynamics_cache(vms, eltype(vms))

function new_control_cache(vms::CVMSystem, qᵛ₀, gravity)
    bundle = new_dynamics_cache(vms)
    cache = bundle.cache
    qʳ, qᵛ = get_q(bundle)
    q̇ʳ, q̇ᵛ = get_q̇(bundle)
    uʳ, uᵛ = get_u(bundle)
    fill!(qʳ, 0)
    copyto!(qᵛ, qᵛ₀)
    fill!(q̇ʳ, 0)
    fill!(q̇ᵛ, 0)
    fill!(uʳ, 0)
    fill!(uᵛ, 0)
    cache.gravity[] = gravity
    bundle
end

"""
    get_ode_dynamics(cache, gravity; [f_setup, f_control])

Returns a function for the autonomous ODE dynamics of the mechanism in the form
    `̇x = f(x, p, t)`
where the parameters p are ignored.

This form is used for DifferentialEquations.jl

# `f_setup`, `f_control`, and `f_shutdown`

The optional keyword arguments `f_setup` and `f_control` are functions that can be used 
to implement custom behaviour.
- `f_setup(cache)` is called once before starting.
- `f_control(cache, t, setup_ret, extra)` is called at each time step to allow for custom control 
  logic. `setup_ret` is the return value of `f_setup`, and can be used to initialize variables 
  before the controller starts. You can directly modify the robot/virtual mechanism state in this
  function, or modify the vectors returned by `get_u(cache)` to apply additional actuation to the
  robot or virtual mechanism.
"""
function get_ode_dynamics end


DEFAULT_F_SETUP(cachebundle) = nothing
DEFAULT_F_CONTROL(cachebundle, t, setup_ret, extra) = false
DEFAULT_F_SHUTDOWN(cachebundle, setup_ret) = nothing

function get_ode_dynamics(
        bundle::MechDynamicsBundle, 
        gravity; 
        f_setup=DEFAULT_F_SETUP, 
        f_control=DEFAULT_F_CONTROL,
    )
    m = bundle.mechanism
    iq, iq̇ = state_idxs(m)
    t_cache = get_t(bundle)
    q_cache = get_q(bundle)
    q̇_cache = get_q̇(bundle)
    u_cache = get_u(bundle)
    fill!(u_cache, zero(eltype(u_cache)))
    g_cache = get_gravity(bundle)
    g_cache[] = gravity
    args = f_setup(bundle)
    function ode_dynamics!(dx, x, p, t)
        copyto!(q_cache, 1, x, iq.start, length(iq))
        copyto!(q̇_cache, 1, x, iq̇.start, length(iq̇))
        t_cache[] = t
        q̈_cache = _dynamics!(bundle) # Mutates
        copyto!(dx, iq.start, q̇_cache, 1, length(iq))
        copyto!(dx, iq̇.start, q̈_cache, 1, length(iq̇))
        f_control(bundle, t, args, (dx, x, p))
        nothing
    end
end

function get_ode_dynamics(
        bundle::VMSDynamicsBundle, 
        gravity; 
        f_setup=DEFAULT_F_SETUP,
        f_control=DEFAULT_F_CONTROL
    )
    system = bundle.vms
    q_idxs, q̇_idxs = state_idxs(system)
    qʳ_idxs, qᵛ_idxs = q_idxs.mechanism_idxs, q_idxs.vm_idxs
    q̇ʳ_idxs, q̇ᵛ_idxs = q̇_idxs.mechanism_idxs, q̇_idxs.vm_idxs
    t_cache = get_t(bundle)
    qʳ, qᵛ = get_q(bundle)
    q̇ʳ, q̇ᵛ = get_q̇(bundle)
    uʳ, uᵛ = get_u(bundle)
    fill!(uʳ, zero(eltype(uʳ)))
    fill!(uᵛ, zero(eltype(uᵛ)))
    g = get_gravity(bundle)
    g[] = gravity
    args = f_setup(bundle)
    function ode_dynamics!(dx, x, p, t)
        t_cache[] = t
        copyto!(qʳ, 1, x, qʳ_idxs.start, length(qʳ_idxs))
        copyto!(qᵛ, 1, x, qᵛ_idxs.start, length(qᵛ_idxs))
        copyto!(q̇ʳ, 1, x, q̇ʳ_idxs.start, length(q̇ʳ_idxs))
        copyto!(q̇ᵛ, 1, x, q̇ᵛ_idxs.start, length(q̇ᵛ_idxs))      
        
        q̈ʳ, q̈ᵛ = _dynamics!(bundle)

        copyto!(dx, qʳ_idxs.start, q̇ʳ, 1, length(qʳ_idxs))
        copyto!(dx, qᵛ_idxs.start, q̇ᵛ, 1, length(qᵛ_idxs))
        copyto!(dx, q̇ʳ_idxs.start, q̈ʳ, 1, length(q̇ʳ_idxs))
        copyto!(dx, q̇ᵛ_idxs.start, q̈ᵛ, 1, length(q̇ᵛ_idxs))
        f_control(bundle, t, args, (dx, x, p))
        nothing
    end
end

"""
    get_ode_problem(cache, gravity, q, q̇, tspan, [p, f_setup, f_control]; kwargs...)

Construct an ODE problem. See also [`VMRobotControl.get_ode_dynamics`](@ref).
initial conditions are given by `q` and `q̇`.

DifferentialEquations.jl must be loaded for this method to be defined: `using DifferentialEquations`
Implementations are in in ./ext/VMRobotControlDifferentialEquationsExt.jl

`p` and `kwargs` are optional arguments that can be used to customize the `ODEProblem`.
`f_setup` and `f_control` are optional arguments that can be used to customize the dynamics function.
"""
function get_ode_problem end

stored_energy(cache::MechanismCacheBundle) = stored_energy(cache, get_energy_components(cache.mechanism))

function stored_energy(bundle::VirtualMechanismSystemCacheBundle)
    Eʳ = stored_energy(robot_cache(bundle))
    Eᵛ = stored_energy(virtual_mechanism_cache(bundle))
    Eˢ = stored_energy(bundle, storages(bundle.vms))
    Eʳ + Eᵛ + Eˢ
end

