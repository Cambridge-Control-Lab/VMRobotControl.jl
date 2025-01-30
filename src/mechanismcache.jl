##########################
# Frame Caches
##########################
# Frame caches contain computation results/inputs for rigid body frames, but not for coordinates.
# They are used to store the results of kinematics and dynamics computations.

abstract type AbstractFrameCache{T} end
# TODO  move t, q, q̇ etc. to mechanism cache

struct FrameKinematicsCache{T} <: AbstractFrameCache{T}
    transforms::Vector{Transform{T}}
    function FrameKinematicsCache{T}(Nframes) where T
        transforms = zeros(Transform{T}, Nframes)
        new{T}(transforms)
    end
end

struct FrameJacobiansCache{T} <: AbstractFrameCache{T}
    transforms::Vector{Transform{T}}
    jacobians::Array{T, 3}
    function FrameJacobiansCache{T}(Nframes, NDOF) where T
        transforms = zeros(Transform{T}, Nframes)
        jacobians = zeros(T, 6, NDOF, Nframes)
        new{T}(transforms, jacobians)
    end
end

struct FrameRBStatesCache{T} <: AbstractFrameCache{T}
    rbstates::Vector{RigidBodyState{T}}
    function FrameRBStatesCache{T}(Nframes) where T
        rbstates = zeros(RigidBodyState{T}, Nframes)
        new{T}(rbstates)
    end
end

struct FrameRBStatesJacobianCache{T} <: AbstractFrameCache{T}
    rbstates::Vector{RigidBodyState{T}}
    jacobians::Array{T, 3}
    function FrameRBStatesJacobianCache{T}(Nframes, NDOF) where T
        rbstates = zeros(RigidBodyState{T}, Nframes)
        jacobians = zeros(T, 6, NDOF, Nframes)
        new{T}(rbstates, jacobians)
    end
end

struct FrameRBStatesWrenchesCache{T} <: AbstractFrameCache{T}
    rbstates::Vector{RigidBodyState{T}}
    forces::Vector{SVector{3, T}}
    torques::Vector{SVector{3, T}}
    function FrameRBStatesWrenchesCache{T}(Nframes) where T
        rbstates = zeros(RigidBodyState{T}, Nframes)
        forces = zeros(SVector{3, T}, Nframes)
        torques = zeros(SVector{3, T}, Nframes)
        new{T}(rbstates, forces, torques)
    end
end


##########################
# Mechanism Cache
##########################
# Mechanism caches contain computation results/inputs for rigid body mechanisms and coordinates.
# TODO check dimensions of jacobia in frame caches against ndof in inner constructors, for extra safety

function coord_cache_size(m)
    mapreduce(+, coordinates(m); init=0) do tsc
        mapreduce(cache_size, +, tsc; init=0)
    end
end
zero_q(::Type{T}, m::CompiledMechanism) where T = zeros(T, ndof(m))
zero_q̇(::Type{T}, m::CompiledMechanism) where T = zeros(T, ndof(m))
zero_q̈(::Type{T}, m::CompiledMechanism) where T = zeros(T, ndof(m))
zero_u(::Type{T}, m::CompiledMechanism) where T = zeros(T, ndof(m))

zero_q(::Type{T}, m::CompiledVirtualMechanismSystem) where T = (zero_q(T, m.robot), zero_q(T, m.virtual_mechanism))
zero_q̇(::Type{T}, m::CompiledVirtualMechanismSystem) where T = (zero_q̇(T, m.robot), zero_q̇(T, m.virtual_mechanism))
zero_q̈(::Type{T}, m::CompiledVirtualMechanismSystem) where T = (zero_q̈(T, m.robot), zero_q̈(T, m.virtual_mechanism))
zero_u(::Type{T}, m::CompiledVirtualMechanismSystem) where T = (zero_u(T, m.robot), zero_u(T, m.virtual_mechanism))

zero_q(m::Union{CompiledMechanism, CompiledVirtualMechanismSystem}) = zero_q(eltype(m), m)
zero_q̇(m::Union{CompiledMechanism, CompiledVirtualMechanismSystem}) = zero_q̇(eltype(m), m)
zero_q̈(m::Union{CompiledMechanism, CompiledVirtualMechanismSystem}) = zero_q̈(eltype(m), m)
zero_u(m::Union{CompiledMechanism, CompiledVirtualMechanismSystem}) = zero_u(eltype(m), m)

Random.rand!(rng::AbstractRNG, q::Tuple{Vector{T}, Vector{T}}) where T<:Real = (rand!(rng, q[1]), rand!(rng, q[2]))
Random.rand!(q::Tuple{Vector{T}, Vector{T}}) where T<:Real = (rand!(q[1]), rand!(q[2]))

struct MechKinematicsCache{T} <: MechanismCache{T}
    t::Base.RefValue{T}
    q::Vector{T}
    frame_cache::FrameKinematicsCache{T}
    coord_cache::CoordKinematicsCache{T}
    function MechKinematicsCache(
            t::Base.RefValue{T},
            q::Vector{T},
            frame_cache::FrameKinematicsCache{T},
            coord_cache::CoordKinematicsCache{T}) where T
        new{T}(t, q, frame_cache, coord_cache)
    end
    function MechKinematicsCache{T}(NDOF, Nf, Nc) where T
        t = Base.RefValue{T}(0.0)
        q = zeros(T, NDOF)
        fcache = FrameKinematicsCache{T}(Nf)
        ccache = CoordKinematicsCache{T}(Nc)
        MechKinematicsCache(t, q, fcache, ccache)
    end
    function MechKinematicsCache{T}(m::CompiledMechanism) where T
        NDOF, Nf, Nc = ndof(m), num_frames(m), coord_cache_size(m)
        MechKinematicsCache{T}(NDOF, Nf, Nc)
    end
end
const MechKinematicsBundle = MechanismCacheBundle{<:Any, <:MechKinematicsCache}
Base.show(io::IO, b::MechKinematicsBundle) = print(io, "MechKinematicsBundle{...}(\"$(b.mechanism.name)\", ...)")

struct MechJacobiansCache{T} <: MechanismCache{T}
    t::Base.RefValue{T}
    q::Vector{T}
    frame_cache::FrameJacobiansCache{T}
    coord_cache::CoordJacobiansCache{T}
    function MechJacobiansCache(
            t::Base.RefValue{T},
            q::Vector{T},
            frame_cache::FrameJacobiansCache{T},
            coord_cache::CoordJacobiansCache{T}) where T
        new{T}(t, q, frame_cache, coord_cache)
    end
    function MechJacobiansCache{T}(NDOF, Nf, Nc) where T
        t = Base.RefValue{T}(0.0)
        q = zeros(T, NDOF)
        fcache = FrameJacobiansCache{T}(Nf, NDOF)
        ccache = CoordJacobiansCache{T}(Nc, NDOF)
        MechJacobiansCache(t, q, fcache, ccache)
    end
    function MechJacobiansCache{T}(m::CompiledMechanism) where T
        NDOF, Nf, Nc = ndof(m), num_frames(m), coord_cache_size(m)
        MechJacobiansCache{T}(NDOF, Nf, Nc)
    end
end
const MechJacobiansBundle = MechanismCacheBundle{<:Any, <:MechJacobiansCache}
Base.show(io::IO, b::MechJacobiansBundle) = print(io, "MechJacobiansBundle{...}(\"$(b.mechanism.name)\", ...)")


struct MechRBStatesCache{T} <: MechanismCache{T}
    t::Base.RefValue{T}
    q::Vector{T}
    q̇::Vector{T}
    q̈::Vector{T}
    gravity::Base.RefValue{SVector{3, T}}
    frame_cache::FrameRBStatesCache{T}
    coord_cache::CoordRBStatesCache{T}
    function MechRBStatesCache(
            t::Base.RefValue{T},
            q::Vector{T},
            q̇::Vector{T},
            q̈::Vector{T},
            gravity::Base.RefValue{SVector{3, T}},
            frame_cache::FrameRBStatesCache{T},
            coord_cache::CoordRBStatesCache{T}) where T
        new{T}(t, q, q̇, q̈, gravity, frame_cache, coord_cache)
    end
    function MechRBStatesCache{T}(NDOF, Nf, Nc) where T
        t = Base.RefValue{T}(0.0)
        q = zeros(T, NDOF)
        q̇ = zeros(T, NDOF)
        q̈ = zeros(T, NDOF)
        gravity = Base.RefValue(zero(SVector{3, T}))
        fcache = FrameRBStatesCache{T}(Nf)
        ccache = CoordRBStatesCache{T}(Nc)
        new{T}(t, q, q̇, q̈, gravity, fcache, ccache)
    end    
    function MechRBStatesCache{T}(m::CompiledMechanism) where T
        NDOF, Nf, Nc = ndof(m), num_frames(m), coord_cache_size(m)
        MechRBStatesCache{T}(NDOF, Nf, Nc)
    end
end
const MechRBStatesBundle = MechanismCacheBundle{<:Any, <:MechRBStatesCache}
Base.show(io::IO, b::MechRBStatesBundle) = print(io, "MechRBStatesBundle{...}(\"$(b.mechanism.name)\", ...)")

struct MechDynamicsCache{T} <: MechanismCache{T}
    t::Base.RefValue{T}
    q::Vector{T}
    q̇::Vector{T}
    q̈::Vector{T}
    gravity::Base.RefValue{SVector{3, T}}
    u::Vector{T}
    inertance_matrix::Matrix{T}
    inertance_matrix_workspace::Matrix{T}
    generalized_force::Vector{T}
    generalized_force_workspace::Vector{T}
    frame_cache::FrameRBStatesJacobianCache{T}
    coord_cache::CoordRBStatesJacobianCache{T}
    function MechDynamicsCache(
            t::Base.RefValue{T},
            q::Vector{T},
            q̇::Vector{T},
            q̈::Vector{T},
            gravity::Base.RefValue{SVector{3, T}},
            u::Vector{T},
            inertance_matrix::Matrix{T},
            inertance_matrix_workspace::Matrix{T},
            generalized_force::Vector{T},
            generalized_force_workspace::Vector{T},
            frame_cache::FrameRBStatesJacobianCache{T},
            coord_cache::CoordRBStatesJacobianCache{T}) where T
        NDOF = length(q)
        @assert NDOF == length(q̇) == length(q̈) == length(generalized_force) == length(u) == 
                size(inertance_matrix, 1) == size(inertance_matrix, 2) == 
                size(inertance_matrix_workspace, 1) == size(inertance_matrix_workspace, 2) ==
                size(generalized_force, 1) == size(generalized_force_workspace, 1) ==
                size(coord_cache.J, 2)
        new{T}(t, q, q̇, q̈, gravity, u, inertance_matrix, inertance_matrix_workspace, 
            generalized_force, generalized_force_workspace, frame_cache, coord_cache)
    end
    function MechDynamicsCache{T}(NDOF, Nf, Nc) where T
        t = Base.RefValue{T}(0.0)
        q = zeros(T, NDOF)
        q̇ = zeros(T, NDOF)
        q̈ = zeros(T, NDOF)
        u = zeros(T, NDOF)
        gravity = Base.RefValue(zero(SVector{3, T}))
        inertance_matrix = zeros(T, NDOF, NDOF)
        inertance_matrix_workspace = zeros(T, NDOF, NDOF)
        generalized_force = zeros(T, NDOF)
        generalized_force_workspace = zeros(T, NDOF)
        fcache = FrameRBStatesJacobianCache{T}(Nf, NDOF)
        ccache = CoordRBStatesJacobianCache{T}(Nc, NDOF)
        MechDynamicsCache(t, q, q̇, q̈, gravity, u, inertance_matrix, inertance_matrix_workspace,
            generalized_force, generalized_force_workspace, fcache, ccache)
    end       
    function MechDynamicsCache{T}(m::CompiledMechanism) where T
        NDOF, Nf, Nc = ndof(m), num_frames(m), coord_cache_size(m)
        MechDynamicsCache{T}(NDOF, Nf, Nc)
    end 
end
const MechDynamicsBundle = MechanismCacheBundle{<:Any, <:MechDynamicsCache}
Base.show(io::IO, b::MechDynamicsBundle) = print(io, "MechDynamicsBundle{...}(\"$(b.mechanism.name)\", ...)")

struct MechRNECache{T} <: MechanismCache{T}
    t::Base.RefValue{T}
    q::Vector{T}
    q̇::Vector{T}
    q̈::Vector{T}
    gravity::Base.RefValue{SVector{3, T}}
    u::Vector{T}
    generalized_force::Vector{T}
    generalized_force_workspace::Vector{T}
    frame_cache::FrameRBStatesWrenchesCache{T}
    coord_cache::CoordRBStatesWrenchesCache{T}
    function MechRNECache(
            t::Base.RefValue{T},
            q::Vector{T},
            q̇::Vector{T},
            q̈::Vector{T},
            gravity::Base.RefValue{SVector{3, T}},
            u::Vector{T},
            generalized_force::Vector{T},
            generalized_force_workspace::Vector{T},
            frame_cache::FrameRBStatesWrenchesCache{T},
            coord_cache::CoordRBStatesWrenchesCache{T}) where T
        NDOF = length(q)
        @assert NDOF == length(q̇) == length(q̈) == length(generalized_force) == length(u) == 
                size(generalized_force, 1) == size(generalized_force_workspace, 1)
        new{T}(t, q, q̇, q̈, gravity, u, generalized_force, generalized_force_workspace,
               frame_cache, coord_cache)
    end
    function MechRNECache{T}(NDOF, Nf, Nc) where T
        t = Base.RefValue{T}(0.0)
        q = zeros(T, NDOF)
        q̇ = zeros(T, NDOF)
        q̈ = zeros(T, NDOF)
        u = zeros(T, NDOF)
        gravity = Base.RefValue(zero(SVector{3, T}))
        generalized_force = Vector{T}(undef, NDOF)
        generalized_force_workspace = Vector{T}(undef, NDOF)
        fcache = FrameRBStatesWrenchesCache{T}(Nf)
        ccache = CoordRBStatesWrenchesCache{T}(Nc)
        MechRNECache(t, q, q̇, q̈, gravity, u, generalized_force, generalized_force_workspace, fcache, ccache)
    end       
    function MechRNECache{T}(m::CompiledMechanism) where T
        NDOF, Nf, Nc = ndof(m), num_frames(m), coord_cache_size(m)
        MechRNECache{T}(NDOF, Nf, Nc)
    end 
end
const MechRNEBundle = MechanismCacheBundle{<:Any, <:MechRNECache}
Base.show(io::IO, b::MechRNEBundle) = print(io, "MechRNEBundle{...}(\"$(b.mechanism.name)\", ...)")


#################
# Accessor docs #
#################
"""
    get_rbstate(cache, frame::CompiledFrameID)

Returns the rigid body state of `frame` from the cache.
"""
function get_rbstate end

"""
    get_transform(cache, frame::CompiledFrameID)

Returns the transform of the `frame` from the cache.
"""
function get_transform end

"""
    get_linear_vel(cache, frame::CompiledFrameID)

Returns the linear velocity of the origin of `frame` from the cache.
"""
function get_linear_vel end

"""
    get_angular_vel(cache, frame::CompiledFrameID)

Returns the angular velocity of the `frame` from the cache.
"""
function get_angular_vel end

"""
    get_linear_vpa(cache, frame::CompiledFrameID)

Returns the linear velocity product acceleration of the origin of `frame` from the cache.
"""
function get_linear_vpa end

"""
    get_angular_vpa(cache, frame::CompiledFrameID)

Returns the angular velocity product acceleration of `frame` from the cache.
"""
function get_angular_vpa end

"""
    get_linear_acceleration(cache, frame::CompiledFrameID)

Returns the total linear acceleration of the origin of `frame` from the cache.
"""
function get_linear_acceleration end

"""
    get_angular_acceleration(cache, frame::CompiledFrameID)

Returns the total angular acceleration of `frame` from the cache.
"""
function get_angular_acceleration end


"""
    get_linear_jacobian(cache, frame::CompiledFrameID)

Returns the linear Jacobian of `frame` from the cache.
"""
function get_linear_jacobian end

"""
    get_angular_jacobian(cache, frame::CompiledFrameID)

Returns the angular Jacobian of `frame` from the cache.
"""
function get_angular_jacobian end

"""
    get_t(cache)

Returns the time from the cache.
"""
function get_t end

"""
    get_q(cache)

Returns the joint configurations from the cache.
"""
function get_q end

"""
    get_q̇(cache)

Returns the joint velocities from the cache.
"""
function get_q̇ end

"""
    get_q̈(cache)

Returns the joint accelerations from the cache.
"""
function get_q̈ end

"""
    get_u(cache)

Returns the additional joint torques from the cache. 

This can be used during `f_control` to apply additional torques to the system, by modifying the 
returned torques inplace. By default, these torques are zero until set otherwise.
"""
function get_u end


"""
    get_generalized_force(cache)

Returns the generalized forces in the cache.

For a virtual mechanism system, this is a tuple of vectors: one for the robot and one for the 
virtual mechanism.
"""
function get_generalized_force end

"""
    get_inertance_matrix(cache)

Returns the inertance matrix in the cache.

For a virtual mechanism system, this is a tuple of inertance matrices: one for the robot and one 
for the virtual mechanism.
"""
function get_inertance_matrix end

#=============================================================================
# CacheBundle Accessors
==============================================================================
A cache bundle simply contains a mechanism/vms and a cache for performing
computations.

The following section in its entirety is a set of methods to access various 
fields of the mechanism/vms or cache from the bundle in a unified interface
=============================================================================#
mechanism_type(::Type{<:CacheBundle{M, C}}) where {M, C} = M
mechanism_type(c::CacheBundle) = mechanism_type(typeof(c))
cache_type(::Type{<:CacheBundle{M, C}}) where {M, C} = C
cache_type(c::CacheBundle) = cache_type(typeof(c))
Base.eltype(::Type{CB}) where CB<:CacheBundle = eltype(cache_type(CB))
Base.eltype(c::CacheBundle) = eltype(typeof(c))

function get_vms_subcache(cache::VMSCacheBundle, c::Union{VMSCoordID, VMSFrameID, VMSJointID, VMSComponentID})
    loc = _vms_location(c)
    if loc == Val{ON_ROBOT}()
        return robot_cache(cache)
    elseif loc == Val{ON_VIRTUAL_MECHANISM}()
        return virtual_mechanism_cache(cache)
    elseif loc == Val{ON_SYSTEM}()
        cache
    else
        throw(ErrorException("Invalid VMSCoordIdx: loc:`$loc`"))
    end
end

# get_vms_subcache(cache::MechanismCacheBundle, ::Union{CompiledFrameID, CompiledJointID, CompiledCoordID, CompiledComponentID}) = cache # Not a vms cache, so no need to get subcache

#=============================================================================
The following section forwards many methods so that if they are called on the
bundle they get forwarded either to the mechanism or the cache, as is 
appropriate.

These functions are generated from the function signatures below.
They follow the pattern: foo(bundle, args...) = foo(bundle.bar, args...)

For example, the first line becomes:
num_frames(bundle::CacheBundle) = num_frames(bundle.cache, )
=============================================================================#
signatures = [
    # Forward to cache
    (:num_frames,                   :cache,     CacheBundle, ()),
    (:get_t,                        :cache,     CacheBundle, ()),
    (:get_q,                        :cache,     CacheBundle, ()),
    (:get_q̇,                        :cache,     CacheBundle, ()),
    (:get_q̈,                        :cache,     CacheBundle, ()),
    (:get_u,                        :cache,     CacheBundle, ()),
    (:get_torques,                  :cache,     CacheBundle, ()),
    (:get_gravity,                  :cache,     CacheBundle, ()),
    (:get_inertance_matrix,         :cache,     CacheBundle, ()),
    (:get_inertance_matrix_workspace, :cache,   CacheBundle, ()),
    (:get_generalized_force,        :cache,     CacheBundle, ()),
    (:get_generalized_force_workspace, :cache,  CacheBundle, ()),
    (:get_frame_forces,             :cache,     MechanismCacheBundle,   ()),
    (:get_frame_torques,            :cache,     MechanismCacheBundle,   ()),  
    (:get_q,                        :cache,     CacheBundle,            (CompiledMechanismJoint,)),
    (:get_q̇,                        :cache,     CacheBundle,            (CompiledMechanismJoint,)),
    (:get_rbstate,                  :cache,     MechanismCacheBundle,   (CompiledFrameID,)),
    (:get_transform,                :cache,     MechanismCacheBundle,   (CompiledFrameID,)),
    (:get_angular_vel,              :cache,     MechanismCacheBundle,   (CompiledFrameID,)),
    (:_get_angular_acc,             :cache,     MechanismCacheBundle,   (CompiledFrameID,)),
    (:get_angular_vpa,              :cache,     MechanismCacheBundle,   (CompiledFrameID,)),
    (:get_angular_acceleration,     :cache,     MechanismCacheBundle,   (CompiledFrameID,)),
    (:get_linear_vel,               :cache,     MechanismCacheBundle,   (CompiledFrameID,)),
    (:_get_linear_acc,              :cache,     MechanismCacheBundle,   (CompiledFrameID,)),
    (:get_linear_vpa,               :cache,     MechanismCacheBundle,   (CompiledFrameID,)),
    (:get_linear_acceleration,      :cache,     MechanismCacheBundle,   (CompiledFrameID,)),
    (:get_linear_jacobian,          :cache,     MechanismCacheBundle,   (CompiledFrameID,)),
    (:get_angular_jacobian,         :cache,     MechanismCacheBundle,   (CompiledFrameID,)),
    (:get_frame_force,              :cache,     MechanismCacheBundle,   (CompiledFrameID,)),
    (:get_frame_torque,             :cache,     MechanismCacheBundle,   (CompiledFrameID,)),  
    # Forwards to mechanism
    (:name,                         :mechanism, MechanismCacheBundle,   ()),
    (:frames,                       :mechanism, MechanismCacheBundle,   ()),
    (:num_frames,                   :mechanism, MechanismCacheBundle,   ()),
    (:root_frame,                   :mechanism, MechanismCacheBundle,   ()),
    (:joints,                       :mechanism, MechanismCacheBundle,   ()),
    (:coordinates,                  :mechanism, MechanismCacheBundle,   ()),
    (:components,                   :mechanism, MechanismCacheBundle,   ()),
    (:config_size,                  :mechanism, MechanismCacheBundle,   ()),
    (:velocity_size,                :mechanism, MechanismCacheBundle,   ()),
    (:ndof,                         :mechanism, MechanismCacheBundle,   ()),
    (:inertances,                   :mechanism, MechanismCacheBundle,   ()),
    (:storages,                     :mechanism, MechanismCacheBundle,   ()),
    (:dissipations,                 :mechanism, MechanismCacheBundle,   ()),
    (:generic_components,           :mechanism, MechanismCacheBundle,   ()),
    (:visuals,                      :mechanism, MechanismCacheBundle,   ()),
    (:q_idxs,                       :mechanism, MechanismCacheBundle,   ()),
    (:q̇_idxs,                       :mechanism, MechanismCacheBundle,   ()),
    (:state_idxs,                   :mechanism, MechanismCacheBundle,   ()),
    (:zero_q,                       :mechanism, MechanismCacheBundle,   ()),
    (:zero_q̇,                       :mechanism, MechanismCacheBundle,   ()),
    (:zero_u,                       :mechanism, MechanismCacheBundle,   ()),
    # Forwards to vms
    (:ndof,                         :vms,       VMSCacheBundle,         ()),
    (:coordinates,                  :vms,       VMSCacheBundle,         ()),
    (:components,                   :vms,       VMSCacheBundle,         ()),
    (:inertances,                   :vms,       VMSCacheBundle,         ()),
    (:storages,                     :vms,       VMSCacheBundle,         ()),
    (:dissipations,                 :vms,       VMSCacheBundle,         ()),
    (:generic_components,           :vms,       VMSCacheBundle,         ()),
    (:visuals,                      :vms,       VMSCacheBundle,         ()),
    (:q_idxs,                       :vms,       VMSCacheBundle,         ()),
    (:q̇_idxs,                       :vms,       VMSCacheBundle,         ()),
    (:state_idxs,                   :vms,       VMSCacheBundle,         ()),
    (:zero_q,                       :vms,       VMSCacheBundle,         ()),
    (:zero_q̇,                       :vms,       VMSCacheBundle,         ()),
    (:zero_u,                       :vms,       VMSCacheBundle,         ()),
    # CompiledID interface
    (:get_compiled_frameID,         :mechanism, MechanismCacheBundle,   (String,)),
    (:get_compiled_jointID,         :mechanism, MechanismCacheBundle,   (String,)),
    (:get_compiled_coordID,         :mechanism, MechanismCacheBundle,   (String,)),
    (:get_compiled_componentID,     :mechanism, MechanismCacheBundle,   (String,)),
    (:get_compiled_frameID,         :vms,       VMSCacheBundle,         (String,)),
    (:get_compiled_jointID,         :vms,       VMSCacheBundle,         (String,)),
    (:get_compiled_coordID,         :vms,       VMSCacheBundle,         (String,)),
    (:get_compiled_componentID,     :vms,       VMSCacheBundle,         (String,)),
    (:(Base.getindex),              :mechanism, MechanismCacheBundle,   (CompiledJointID,)),
    (:(Base.getindex),              :mechanism, MechanismCacheBundle,   (CompiledCoordID,)),
    (:(Base.getindex),              :mechanism, MechanismCacheBundle,   (CompiledComponentID,)),
    (:(Base.setindex!),             :mechanism, MechanismCacheBundle,   (Any, CompiledJointID)),
    (:(Base.setindex!),             :mechanism, MechanismCacheBundle,   (Any, CompiledCoordID)),
    (:(Base.setindex!),             :mechanism, MechanismCacheBundle,   (Any, CompiledComponentID)),
    (:(Base.getindex),              :vms,       VMSCacheBundle,         (VMSJointID,)),
    (:(Base.getindex),              :vms,       VMSCacheBundle,         (VMSCoordID,)),
    (:(Base.getindex),              :vms,       VMSCacheBundle,         (VMSComponentID,)),
    (:(Base.setindex!),             :vms,       VMSCacheBundle,         (Any, VMSJointID)),
    (:(Base.setindex!),             :vms,       VMSCacheBundle,         (Any, VMSCoordID)),
    (:(Base.setindex!),             :vms,       VMSCacheBundle,         (Any, VMSComponentID)),
]

const ARGNAMES = [:a, :b, :c, :d, :e, :f, :g, :h, :i, :j]

# For an example call with inputs:
# (:get_rbstate, :cache, MechanismCacheBundle, (CompiledFrameID,)),
# read the comments
for sig ∈ signatures
    func, bundle_field, BUNDLE, additional_arg_types = sig
    # 1 additional argument will be given name :a, which is ARGNAMES[1]
    additional_arg_names = [ARGNAMES[i] for i in 1:length(additional_arg_types)]
    # type assert expression `a::CompiledFrameID`
    additional_call_args = [:($a::$A) for (a, A) in zip(additional_arg_names, additional_arg_types)]
    # Left hand side `get_rbstate(bundle::MechanismCacheBundle, a::CompiledFrameID)`
    func_lhs = Expr(:call, func, :(bundle::$BUNDLE), additional_call_args...)
    # Right hand side `get_rbstate(bundle.cache, a)`
    func_rhs = Expr(:call, func, :(bundle.$bundle_field), additional_arg_names...)
    # Full expression `get_rbstate(bundle::MechanismCacheBundle, a::CompiledFrameID) = get_rbstate(bundle.cache, a)`
    f = Expr(:(=), func_lhs, func_rhs)
    # Evaluate
    expr = Expr(:block, f)
    @eval $expr
end

####################################################################################################
# Some other bundle forwarding functions

@inline get_q(c::MechanismCacheBundle, joint::CompiledJointID) = get_q(c)[q_idxs(joints(c)[joint.idx])]
@inline get_q̇(c::MechanismCacheBundle, joint::CompiledJointID) = get_q̇(c)[q_idxs(joints(c)[joint.idx])]
@inline coordinate(c::MechanismCacheBundle, id::CompiledCoordID) = coordinates(c.mechanism)[id]

@inline get_frame_forces(c::FrameRBStatesWrenchesCache) = c.forces
@inline get_frame_torques(c::FrameRBStatesWrenchesCache) = c.torques

# VMS bundle
robot_ndof(bundle::VMSCacheBundle) = robot_ndof(bundle.vms)
virtual_mechanism_ndof(bundle::VMSCacheBundle) = virtual_mechanism_ndof(bundle.vms)

@inline get_q(c::VirtualMechanismSystemCacheBundle, i::VMSJointID) = get_q(get_vms_subcache(c, i), i.idx)
@inline get_q̇(c::VirtualMechanismSystemCacheBundle, i::VMSJointID) = get_q̇(get_vms_subcache(c, i), i.idx)
@inline get_transform(c::VirtualMechanismSystemCacheBundle, i::VMSFrameID) = get_transform(get_vms_subcache(c, i), i.idx)
@inline get_angular_vel(c::VirtualMechanismSystemCacheBundle, i::VMSFrameID) = get_angular_vel(get_vms_subcache(c, i), i.idx)
@inline _get_angular_acc(c::VirtualMechanismSystemCacheBundle, i::VMSFrameID) = _get_angular_acc(get_vms_subcache(c, i), i.idx)
@inline get_linear_vel(c::VirtualMechanismSystemCacheBundle, i::VMSFrameID) = get_linear_vel(get_vms_subcache(c, i), i.idx)
@inline _get_linear_acc(c::VirtualMechanismSystemCacheBundle, i::VMSFrameID) = _get_linear_acc(get_vms_subcache(c, i), i.idx)
@inline get_linear_jacobian(c::VirtualMechanismSystemCacheBundle, i::VMSFrameID) = get_linear_jacobian(get_vms_subcache(c, i), i.idx)
@inline get_angular_jacobian(c::VirtualMechanismSystemCacheBundle, i::VMSFrameID) = get_angular_jacobian(get_vms_subcache(c, i), i.idx)
@inline get_frame_force(c::FrameRBStatesWrenchesCache, i::VMSFrameID) = error()
@inline get_frame_torque(c::FrameRBStatesWrenchesCache, i::VMSFrameID) = error()


@inline coordinate(c::VMSCacheBundle, id::VMSCoordID{C, S}) where {C, S} = coordinates(get_vms_subcache(c, id))[id.idx]

#===================================================================================================
 Cache accessors
====================================================================================================
Here we define the access methods not for cache bundles, but for mechanism caches themselves. 
Depending on the type of cache, and its purpose, different fields will be available. For example,
a MechKinematicsCache will not have a field to store accelerations.
===================================================================================================#

#= Every single accessor is defined here, with a default implemetation which typically throws
an error message. More specific methods override this behaviour for cache types which provide the
required fields =#
Base.eltype(::Type{<:ComputeCache{T}}) where T = T
Base.eltype(c::ComputeCache) = eltype(typeof(c))
get_t(c::ComputeCache) = c.t
get_q(c::ComputeCache) = c.q
get_q̇(c::ComputeCache) =                      error("q̇ not available from type $(typeof(c))")
get_q̈(c::ComputeCache) =                      error("q̈ not available from type $(typeof(c))")
get_u(c::ComputeCache) =                      error("u not available from type $(typeof(c))")

get_q(c::ComputeCache, joint::CompiledMechanismJoint) = get_q(c)[q_idxs(joint)]
get_q̇(c::ComputeCache, joint::CompiledMechanismJoint) = get_q̇(c)[q_idxs(joint)]

get_gravity(c::ComputeCache) =                              error("gravity not available from type $(typeof(c))")
get_torques(c::ComputeCache) =                              error("torques not available from type $(typeof(c))")
get_rbstate(c::ComputeCache, i::CompiledFrameID) =          error("rbstate not available from type $(typeof(c))")
get_transform(c::ComputeCache, i::CompiledFrameID) =        error("transform not available from type $(typeof(c))")
get_angular_vel(c::ComputeCache, i::CompiledFrameID) =      error("angular_vel not available from type $(typeof(c))")
get_linear_vel(c::ComputeCache, i::CompiledFrameID) =       error("linear_vel not available from type $(typeof(c))")
_get_angular_acc(c::ComputeCache, i::CompiledFrameID) =      error("_angular_acc not available from type $(typeof(c))")
_get_linear_acc(c::ComputeCache, i::CompiledFrameID) =       error("_linear_acc not available from type $(typeof(c))")
get_angular_vpa(c::ComputeCache, i::CompiledFrameID) =      error("angular_vpa not available from type $(typeof(c))")
get_linear_vpa(c::ComputeCache, i::CompiledFrameID) =       error("linear_vpa not available from type $(typeof(c))")
get_angular_acceleration(c::ComputeCache, i::CompiledFrameID) = error("angular_acceleration not available from type $(typeof(c))")
get_linear_acceleration(c::ComputeCache, i::CompiledFrameID) = error("linear_acceleration not available from type $(typeof(c))")
get_linear_jacobian(c::ComputeCache, i::CompiledFrameID) =  error("linear_jacobian not available from type $(typeof(c))")
get_angular_jacobian(c::ComputeCache, i::CompiledFrameID) = error("angular_jacobian not available from type $(typeof(c))")
get_frame_force(c::ComputeCache, i::CompiledFrameID) =      error("frame_force not available from type $(typeof(c))")
get_frame_torque(c::ComputeCache, i::CompiledFrameID) =     error("frame_torque not available from type $(typeof(c))")
get_frame_forces(c::ComputeCache) =                        error("frame_forces not available from type $(typeof(c))")
get_frame_torques(c::ComputeCache) =                       error("frame_torques not available from type $(typeof(c))")
get_inertance_matrix(c::ComputeCache) =                     error("inertance_matrix not available from type $(typeof(c))")
get_inertance_matrix_workspace(c::ComputeCache) =           error("inertance_matrix_workspace not available from type $(typeof(c))")
get_generalized_force(c::ComputeCache) =                    error("generalized_force not available from type $(typeof(c))")
get_generalized_force_workspace(c::ComputeCache) =          error("generalized_force_workspace not available from type $(typeof(c))")

#= From here onwards, access methods for specific cache types are defined =#
# TransformCacheUnion - any cache with a transforms vector
const TransformCacheUnion = Union{MechKinematicsCache, MechJacobiansCache}

get_transform(c::TransformCacheUnion, i::CompiledFrameID) = c.frame_cache.transforms[i]

# JacobiansCacheUnion - any cache with jacobians
const JacobiansCacheUnion = Union{MechJacobiansCache, MechDynamicsCache}
get_linear_jacobian(c::JacobiansCacheUnion, i::CompiledFrameID) = @views c.frame_cache.jacobians[1:3, :, i]
get_angular_jacobian(c::JacobiansCacheUnion, i::CompiledFrameID) = @views c.frame_cache.jacobians[4:6, :, i]

# RBStateCacheUnion - any cache with rbstates vector
const RBStateCacheUnion = Union{MechRBStatesCache, MechDynamicsCache, MechRNECache}
get_q̇(c::RBStateCacheUnion) = c.q̇
get_q̈(c::RBStateCacheUnion) = c.q̈
get_gravity(c::RBStateCacheUnion) = c.gravity

get_rbstate(c::RBStateCacheUnion, i::CompiledFrameID) = c.frame_cache.rbstates[i]
get_transform(c::RBStateCacheUnion, i::CompiledFrameID) = transform(get_rbstate(c, i))
get_angular_vel(c::RBStateCacheUnion, i::CompiledFrameID) = angular_vel(get_rbstate(c, i))
get_linear_vel(c::RBStateCacheUnion, i::CompiledFrameID) = linear_vel(get_rbstate(c, i))


#= There are two different interpretations of acceleration. The first is velocity-product 
acceleration, also known as bias acceleration. This is the acceleration due to generalized 
velocities q̇ which exist even in the absence of generalized coordinate accelerations (q̈=0)

The second is total accelerations, which are the sum of vpa and J(q)q̈. Depending on the type of
cache, the interpretation of acceleration may differ. 

_get_<linear/angular>_acc is defined for all caches, but may be either vpa or total acceleration
get_linear_vpa is only defined for MechDynamicsCache which is guaranteed to represent a vpa
get_linear_acceleration is only defined for MechRNECache which is guaranteed to represent a total acceleration
=#
_get_angular_acc(c::RBStateCacheUnion, i::CompiledFrameID) = angular_acc(get_rbstate(c, i))
_get_linear_acc(c::RBStateCacheUnion, i::CompiledFrameID) = linear_acc(get_rbstate(c, i))
get_angular_vpa(c::MechDynamicsCache, i::CompiledFrameID) = angular_acc(get_rbstate(c, i))
get_linear_vpa(c::MechDynamicsCache, i::CompiledFrameID) = linear_acc(get_rbstate(c, i))
get_linear_acceleration(c::MechRNECache, i::CompiledFrameID) = angular_acc(get_rbstate(c, i))
get_angular_acceleration(c::MechRNECache, i::CompiledFrameID) = linear_acc(get_rbstate(c, i))

# DynamicsCache - any cache with u, inertance_matrix and generalized_force
get_u(c::MechDynamicsCache) = c.u
get_inertance_matrix(c::MechDynamicsCache) = c.inertance_matrix
get_inertance_matrix_workspace(c::MechDynamicsCache) = c.inertance_matrix_workspace
get_generalized_force(c::MechDynamicsCache) = c.generalized_force
get_generalized_force_workspace(c::MechDynamicsCache) = c.generalized_force_workspace

# RNECache - cache with u
get_u(c::MechRNECache) = c.u
get_frame_forces(c::MechRNECache) = c.frame_cache.forces
get_frame_torques(c::MechRNECache) =c.frame_cache.torques
get_frame_force(c::MechRNECache, i::CompiledFrameID) = c.frame_cache.forces[i]
get_frame_torque(c::MechRNECache, i::CompiledFrameID) = c.frame_cache.torques[i]

# Methods for num frames to avoid ambiguity
num_frames(c::MechanismCache) = num_frames(c.frame_cache)
num_frames(c::FrameKinematicsCache) = length(c.transforms) 
num_frames(c::FrameJacobiansCache) = length(c.transforms)
num_frames(c::FrameRBStatesCache) = length(c.rbstates)
num_frames(c::FrameRBStatesJacobianCache) = length(c.rbstates)

####################################################################################################

struct VMSKinematicsCache{T} <: VirtualMechanismSystemCache{T}
    t::Base.RefValue{T}
    q::Tuple{Vector{T}, Vector{T}}
    robot_frame_cache::FrameKinematicsCache{T}
    robot_coord_cache::CoordKinematicsCache{T}
    vm_frame_cache::FrameKinematicsCache{T}
    vm_coord_cache::CoordKinematicsCache{T}
    coord_cache::CoordKinematicsCache{T}
    function VMSKinematicsCache(
            t::Base.RefValue{T},
            q::Tuple{Vector{T}, Vector{T}},
            robot_frame_cache::FrameKinematicsCache{T},
            robot_coord_cache::CoordKinematicsCache{T},
            vm_frame_cache::FrameKinematicsCache{T},
            vm_coord_cache::CoordKinematicsCache{T},
            coord_cache::CoordKinematicsCache{T}) where T
        new{T}(t, q, robot_frame_cache, robot_coord_cache, vm_frame_cache, vm_coord_cache, coord_cache)
    end
    function VMSKinematicsCache{T}(NDOF_robot, NDOF_vm, Nf_robot, Nf_vm, Nc_robot, Nc_vm, Nc) where T
        t = Base.RefValue{T}(0.0)
        qʳ = zeros(T, NDOF_robot)
        qᵛ = zeros(T, NDOF_vm)
        q = (qʳ, qᵛ)
        robot_frame_cache = FrameKinematicsCache{T}(Nf_robot)
        robot_coord_cache = CoordKinematicsCache{T}(Nc_robot)
        vm_frame_cache = FrameKinematicsCache{T}(Nf_vm)
        vm_coord_cache = CoordKinematicsCache{T}(Nc_vm)
        coord_cache = CoordKinematicsCache{T}(Nc)
        VMSKinematicsCache(t, q, robot_frame_cache, robot_coord_cache, vm_frame_cache, vm_coord_cache, coord_cache)
    end
    function VMSKinematicsCache{T}(vms::CompiledVirtualMechanismSystem) where T
        NDOF_robot, NDOF_vm = ndof(vms.robot), ndof(vms.virtual_mechanism)
        Nf_robot, Nf_vm = num_frames(vms.robot), num_frames(vms.virtual_mechanism)
        Nc_robot, Nc_vm, Nc = coord_cache_size(vms.robot), coord_cache_size(vms.virtual_mechanism), coord_cache_size(vms)
        VMSKinematicsCache{T}(NDOF_robot, NDOF_vm, Nf_robot, Nf_vm, Nc_robot, Nc_vm, Nc)
    end
end

const VMSKinematicsBundle = VirtualMechanismSystemCacheBundle{M, <:VMSKinematicsCache} where M

function robot_cache(bundle::VMSKinematicsBundle)
    c = bundle.cache
    MechanismCacheBundle(
        bundle.vms.robot,
        MechKinematicsCache(c.t, c.q[1], c.robot_frame_cache, c.robot_coord_cache)
    )
end
function virtual_mechanism_cache(bundle::VMSKinematicsBundle)
    c = bundle.cache
    MechanismCacheBundle(
        bundle.vms.virtual_mechanism,
        MechKinematicsCache(c.t, c.q[2], c.vm_frame_cache, c.vm_coord_cache)
    )
end

function Base.show(io::IO, b::VMSKinematicsBundle)
    print(io, "VMSKinematicsBundle{...}(\"$(name(b.vms))\", ...)")
end

######################

struct VMSDynamicsCache{T} <: VirtualMechanismSystemCache{T}
    t::Base.RefValue{T}
    q::Tuple{Vector{T}, Vector{T}}
    q̇::Tuple{Vector{T}, Vector{T}}
    q̈::Tuple{Vector{T}, Vector{T}}
    u::Tuple{Vector{T}, Vector{T}}
    gravity::Base.RefValue{SVector{3, T}}
    inertance_matrix::Tuple{Matrix{T}, Matrix{T}}
    inertance_matrix_workspace::Tuple{Matrix{T}, Matrix{T}}
    generalized_force::Tuple{Vector{T}, Vector{T}}
    generalized_force_workspace::Tuple{Vector{T}, Vector{T}}
    robot_frame_cache::FrameRBStatesJacobianCache{T}
    vm_frame_cache::FrameRBStatesJacobianCache{T}
    robot_coord_cache::CoordRBStatesJacobianCache{T}
    vm_coord_cache::CoordRBStatesJacobianCache{T}
    coord_cache::CoordRBStatesJacobianCache{T}
    function VMSDynamicsCache(
            t::Base.RefValue{T},
            q::Tuple{Vector{T}, Vector{T}},
            q̇::Tuple{Vector{T}, Vector{T}},
            q̈::Tuple{Vector{T}, Vector{T}},
            u::Tuple{Vector{T}, Vector{T}},
            gravity::Base.RefValue{SVector{3, T}},
            inertance_matrix::Tuple{Matrix{T}, Matrix{T}},
            inertance_matrix_workspace::Tuple{Matrix{T}, Matrix{T}},
            generalized_force::Tuple{Vector{T}, Vector{T}},
            generalized_force_workspace::Tuple{Vector{T}, Vector{T}},
            robot_frame_cache::FrameRBStatesJacobianCache{T},
            vm_frame_cache::FrameRBStatesJacobianCache{T},
            robot_coord_cache::CoordRBStatesJacobianCache{T},
            vm_coord_cache::CoordRBStatesJacobianCache{T},
            coord_cache::CoordRBStatesJacobianCache{T}) where T
        new{T}(t, q, q̇, q̈, u, gravity, inertance_matrix, inertance_matrix_workspace,
         generalized_force, generalized_force_workspace, robot_frame_cache, vm_frame_cache,
         robot_coord_cache, vm_coord_cache, coord_cache)
    end
    function VMSDynamicsCache{T}(NDOF_robot, NDOF_vm, Nf_robot, Nf_vm, Nc_robot, Nc_vm, Nc) where T
        t = Base.RefValue{T}(0.0)
        NDOF = NDOF_robot + NDOF_vm
        q = (zeros(T, NDOF_robot), zeros(NDOF_vm))
        q̇ = (zeros(T, NDOF_robot), zeros(NDOF_vm))
        q̈ = (zeros(T, NDOF_robot), zeros(NDOF_vm))
        u = (zeros(T, NDOF_robot), zeros(NDOF_vm))
        gravity = Base.RefValue(zero(SVector{3, T}))
        robot_frame_cache = FrameRBStatesJacobianCache{T}(Nf_robot, NDOF_robot)
        robot_coord_cache = CoordRBStatesJacobianCache{T}(Nc_robot, NDOF_robot)
        vm_frame_cache = FrameRBStatesJacobianCache{T}(Nf_vm, NDOF_vm)
        vm_coord_cache = CoordRBStatesJacobianCache{T}(Nc_vm, NDOF_vm)
        coord_cache = CoordRBStatesJacobianCache{T}(Nc, NDOF) # TODO generalize
        inertance_matrix = (zeros(T, NDOF_robot, NDOF_robot), zeros(T, NDOF_vm, NDOF_vm))
        inertance_matrix_workspace = (Matrix{T}(undef, NDOF_robot, NDOF_robot), Matrix{T}(undef, NDOF_vm, NDOF_vm))
        generalized_force = (zeros(T, NDOF_robot), zeros(T, NDOF_vm))
        generalized_force_workspace = (zeros(T, NDOF_robot), zeros(T, NDOF_vm))
        new{T}(t, q, q̇, q̈, u, gravity, inertance_matrix, inertance_matrix_workspace, 
               generalized_force, generalized_force_workspace, robot_frame_cache,
                vm_frame_cache, robot_coord_cache, vm_coord_cache, coord_cache)
    end
    function VMSDynamicsCache{T}(vms::CompiledVirtualMechanismSystem) where T
        NDOF_robot, NDOF_vm = ndof(vms.robot), ndof(vms.virtual_mechanism)
        Nf_robot, Nf_vm = num_frames(vms.robot), num_frames(vms.virtual_mechanism)
        Nc_robot, Nc_vm, Nc = coord_cache_size(vms.robot), coord_cache_size(vms.virtual_mechanism), coord_cache_size(vms)
        VMSDynamicsCache{T}(NDOF_robot, NDOF_vm, Nf_robot, Nf_vm, Nc_robot, Nc_vm, Nc)
    end    
end

const VMSDynamicsBundle = VirtualMechanismSystemCacheBundle{M, <:VMSDynamicsCache} where M
function robot_cache(bundle::VMSDynamicsBundle)
    c = bundle.cache
    MechanismCacheBundle(
        bundle.vms.robot,
        MechDynamicsCache(
            c.t, c.q[1], c.q̇[1], c.q̈[1], c.gravity, c.u[1],
            c.inertance_matrix[1], c.inertance_matrix_workspace[1], 
            c.generalized_force[1], c.generalized_force_workspace[1],
            c.robot_frame_cache, c.robot_coord_cache
        )
    )
end
function virtual_mechanism_cache(bundle::VMSDynamicsBundle) 
    c = bundle.cache
    MechanismCacheBundle(
        bundle.vms.virtual_mechanism,
        MechDynamicsCache(
            c.t, c.q[2], c.q̇[2], c.q̈[2], c.gravity, c.u[2], 
            c.inertance_matrix[2], c.inertance_matrix_workspace[2],
            c.generalized_force[2], c.generalized_force_workspace[2],
             c.vm_frame_cache, c.vm_coord_cache
        )
    )
end

function Base.show(io::IO, b::VMSDynamicsBundle)
    print(io, "VMSDynamicsBundle{...}(\"$(name(b.vms))\", ...)")
end


######################
# VMSKinematicsCache
@inline get_transform(c::VMSKinematicsCache, i::VMSFrameID) = get_transform(get_vms_subcache(c, i), i.idx)

# VMSDynamicsCache
@inline get_q̇(c::VMSDynamicsCache) = c.q̇
@inline get_q̈(c::VMSDynamicsCache) = c.q̈
@inline get_u(c::VMSDynamicsCache) = c.u
@inline get_gravity(c::VMSDynamicsCache) = c.gravity

@inline get_inertance_matrix(c::VMSDynamicsCache) = c.inertance_matrix
@inline get_inertance_matrix_workspace(c::VMSDynamicsCache) = c.inertance_matrix_workspace
@inline get_generalized_force(c::VMSDynamicsCache) = c.generalized_force 
@inline get_generalized_force_workspace(c::VMSDynamicsCache) = c.generalized_force_workspace
