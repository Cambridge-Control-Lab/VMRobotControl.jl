module VMRobotControl

#####################
# Exports
#####################

export # Mechanisms 
    Mechanism,
    VirtualMechanismSystem,
    compile,
    CompiledMechanism,
    CompiledVirtualMechanismSystem
export # Mechanism accessor methods 
    name,
    frames,
    num_frames,
    root_frame, 
    joints,
    coordinates,
    components,
    config_size, 
    velocity_size, 
    ndof, 
    inertances,
    storages,
    dissipations,
    generic_components,
    visuals
export # Parsing mechanisms
    parseRSON,
    parseRSONString,
    RSONParserConfig,
    serializeRSON,
    parseURDF,
    parseURDFString,
    URDFParserConfig
export # Plotting - From MakieExtensions
    jointsketch,
    jointsketch!,
    componentsketch,
    componentsketch!,
    robotsketch,
    robotsketch!,
    robotvisualize,
    robotvisualize!,
    annotateframes,
    annotateframes!,
    animate_robot,
    link_cache_observable_to_time_observable,
    animate_robot_odesolution,
    recolor_virtual_mechanism!,
    animate_robot_kinematics!

#####################################################

export # Functions for building mechanisms 
    add_frame!,
    add_joint!,
    add_coordinate!,
    add_component!
export # Joint constructors
    Rigid,
    Revolute,
    Prismatic,
    Helical,
    Rail,
    Spherical,
    ReferenceJoint,
    TimeFuncJoint
export # JointData types
    RevoluteData,
    PrismaticData,
    HelicalData,
    RailData,
    SphericalData
export # Coordinates
    ConstCoord,    
    CoordDifference,
    CoordNorm,
    CoordSlice,
    CoordStack,
    CoordSum,
    FrameAngularVelocity,
    FrameOrigin,
    FramePoint,
    JointSubspace,
    QuaternionAttitude,
    ReferenceCoord,
    RotatedCoord,
    UnrotatedCoord
export # Components 
    PointMass,
    GravityCompensator,
    Inertia,
    LinearInerter,
    LinearSpring,
    LinearDamper,
    TanhSpring,
    TanhDamper,
    GaussianSpring,
    ReLUSpring,
    RectifiedSpring,
    RectifiedDamper,
    DiodicDamper,
    ForceSource,
    PowerSource
export # Component adding methods
    add_gravity_compensator!,
    add_gravity_compensation!,
    add_deadzone_springs!,
    add_inertia!
export # Visual Components
    Visual

#####################################################

export # CompiledMechanism/CompiledVirtualMechanismSystem methods
    q_idxs, 
    q̇_idxs,
    state_idxs,
    get_compiled_frameID,
    get_compiled_joint,
    get_compiled_jointID,
    get_compiled_coordID,
    get_compiled_componentID
export # Operation space coordinate methods
    configuration,
    velocity,
    acceleration,
    vpa, # same as acceleration
    jacobian

#####################################################

export # Algorithms 
    new_kinematics_cache,
    new_jacobians_cache,
    new_rbstates_cache,
    new_dynamics_cache,
    new_inverse_dynamics_cache,
    new_control_cache
export 
    kinematics!,
    velocity_kinematics!,
    acceleration_kinematics!,
    jacobians!,
    precompute!,
    precompute_transforms!, # Deprecated
    inertance_matrix!,
    generalized_force!,
    dynamics!,
    inverse_dynamics!,
    control_step!
export # For setting up ODEs
    get_ode_dynamics,
    assemble_state,
    get_ode_problem,
    zero_q, # Useful to create a vector for q
    zero_q̇,
    zero_q̈,
    zero_u
export 
    opspace_force,
    stored_energy

#####################################################

export # Rigid body transforms/velocities/accelerations 
    Rotor,
    Transform,
    Twist,
    SpatialAcceleration
export # Additional Rotor contructors
    XRotor,
    YRotor,
    ZRotor
export rotor_to_svector
export # Accessors for Transform
    origin,
    rotor
export # Accessors for caches
    get_rbstate,
    get_transform,
    get_linear_vel,
    get_angular_vel, 
    _get_linear_acc,
    _get_angular_acc,
    get_linear_vpa,
    get_angular_vpa,
    get_linear_acceleration,
    get_angular_acceleration,
    get_linear_jacobian, 
    get_angular_jacobian,
    get_t,
    get_q,
    get_q̇,
    get_q̈,
    get_u,
    get_gravity,
    get_torques,
    get_inertance_matrix,
    get_generalized_force
  

##################
# Imports
##################

using InteractiveUtils: @code_warntype

include("hessians.jl")
include("splines.jl")
include("matmul_utils.jl")

####################
# Usings
####################

using
 .Hessians,
 .Splines

# using DifferentialEquations: DifferentialEquations
import DiffResults: 
    DiffResult, 
    value, 
    derivative, 
    hessian
import
    GeometryBasics # Use import to avoid name conflicts

using GeometryBasics:
    normal_mesh

using 
    DigitalAssetExchangeFormatIO,
    FileIO,
    Graphs,
    LinearAlgebra,
    LoopVectorization,
    Observables,
    OrderedCollections,
    Random,
    ResultTypes,
    StaticArrays

using Colors:
    RGBA
const RGBAf = RGBA{Float32}

using UUIDs: 
    UUID, # For FileIO registration of DAEs
    uuid4 # For unique joint IDs if unnamed

@inline jacobian(r::DiffResult) = derivative(r::DiffResult)

#################
# Transforms
#################

# Maybe todo in the future... switch to CoordinateTransforms.jl?
include("./transforms/transforms.jl")
include("./transforms_hessians_ext.jl")

using .Transforms
using .TransformsHessiansExt

valuetype(::Type{Transform{T}}) where T = T
valuetype(::Transform{T}) where T = T
valuetype(::Type{RigidBodyState{T}}) where T = T
valuetype(::RigidBodyState{T}) where T = T

zero3(::Type{T}) where T = zero(SVector{3, T})

# Some Abstract Types. TODO Organize these better
abstract type CacheBundle{M, C} end
abstract type ComputeCache{T} end
abstract type MechanismCache{T} <: ComputeCache{T} end
abstract type VirtualMechanismSystemCache{T} <: ComputeCache{T} end

include("./coordinates/coordinate_cache.jl")
include("./type_stable_collection.jl")
using .TypeStableCollections
include("./joints/joint_definitions.jl")

# Regarding Virtual Mechanism, these 3 symbolss are used to distinguish between frames/joints/coordinates
# that are on the robot, on the virtual mechanism, or on the VirtualMechanismSystem (inbetween the robot/virtual mechanism)
const ON_ROBOT = :robot_coord
const ON_VIRTUAL_MECHANISM = :virtual_mechanism_coord
const ON_SYSTEM = :system_coord

include("./joints/mechanism_joint.jl")
include("./coordinates/coordinates.jl")
include("./coordinates/coordinate_definitions.jl")

# TODO move definitions so that these can be dependent on CompiledMechanism and CompiledVirtualMechanismSystem
struct MechanismCacheBundle{M, C} <: CacheBundle{M, C}
    mechanism::M
    cache::C
end

struct VirtualMechanismSystemCacheBundle{M, C} <: CacheBundle{M, C} # CacheBundle{M<:CVMSystem, C::VirtualMechanismSystemCache}
    vms::M
    cache::C
end

const VMSCacheBundle = VirtualMechanismSystemCacheBundle # Short alias

include("./components/components.jl")
include("./components/inertances.jl")
include("./components/storages.jl")
include("./components/dissipations.jl")
include("./components/generic_components.jl")
include("./components/component_collection.jl")


###########################
# Mechanism Parsing/Construction
###########################
include("./mechanism.jl")
include("./mesh_utils.jl")
include("./rson/rson.jl")
using .RSON
include("./urdf/urdf.jl")
using .URDF

###########################
include("./compiled_mechanisms.jl")
include("./compiled_virtual_mechanisms.jl")

include("./mechanismcache.jl")

include("./kinematics.jl")
include("./coordinates/coordinate_interface.jl")
include("./coordinates/coordinate_implementations.jl")
include("./interface.jl")
include("./plotting.jl")

end
