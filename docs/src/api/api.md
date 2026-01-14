# API
```@index
Pages= ["api/api.md]"
```

## Loading from RSON/URDF

```@docs
parseRSON
parseURDF
RSONParserConfig
URDFParserConfig
serializeRSON
```

## Building
### Types
```@docs
Mechanism
VirtualMechanismSystem
```
### Accessing Mechanisms/Virtual Mechanism Systems
```@docs
name
frames
num_frames
root_frame
joints
coordinates
components
config_size
velocity_size
ndof
inertances
storages
dissipations
generic_components
visuals
```
### Building methods
```@docs
add_frame!
add_joint!
add_coordinate!
add_component!
add_inertia!
add_gravity_compensation!
add_gravity_compensator!
```

## Using
### Compiling and Cache-bundles
```@docs
compile
new_kinematics_cache
new_jacobians_cache
new_rbstates_cache
new_dynamics_cache
new_control_cache
```
### Accessing data from cache-bundles
```@docs
get_rbstate
get_transform
get_linear_vel
get_angular_vel
get_linear_vpa
get_angular_vpa
get_linear_acceleration
get_angular_acceleration
get_linear_jacobian
get_angular_jacobian
get_t
get_q
get_q̇
get_q̈
get_u
get_inertance_matrix
get_generalized_force
```

### Computations
```@docs
kinematics!
jacobians!
velocity_kinematics!
acceleration_kinematics!
precompute!
generalized_force!
inertance_matrix!
dynamics!
control_step!
stored_energy
opspace_force
```
### Accessing compiled frames/coords/joints/components
```@docs
get_compiled_frameID
get_compiled_coordID
get_compiled_jointID
get_compiled_componentID
```

### Setting up simulations
```@docs
get_ode_dynamics
assemble_state
get_ode_problem
zero_q
zero_q̇
zero_q̈
zero_u
```

## Plotting


## Joints
### AbstractJointData types
```@docs
Rigid
Revolute
Prismatic
Rail
ReferenceJoint
```

## Coordinates
```@docs
VMRobotControl.CoordinateData
```

### Coordinate Interface
```@docs
configuration
velocity
jacobian
acceleration
```

### Coordinate types
```@docs
CoordDifference
CoordNorm
CoordStack
CoordSlice
CoordSum
ConstCoord
JointSubspace
FrameOrigin
FramePoint
FrameAngularVelocity
QuaternionAttitude
ReferenceCoord
RotatedCoord
UnrotatedCoord
ShadowCoord
```

## Components
```@docs
VMRobotControl.Storage
LinearSpring
TanhSpring
GaussianSpring
RectifiedSpring
GravityCompensator
ForceSource
PowerSource
LinearDamper
TanhDamper
RectifiedDamper
DiodicDamper
LinearInerter
Inertia
PointMass
```
