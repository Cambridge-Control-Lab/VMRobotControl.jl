# Additional info for Developers

```@index
Pages= ["developer/developer.md]"
```

# Testing

To test, navigate to the folder "VMRobotControl.jl", hit "]" to enter the Pkg context, and run
```
test
```

# Kinematics computations

```@docs
VMRobotControl.RBTree
VMRobotControl.joint_relative_transform
VMRobotControl.joint_relative_twist
VMRobotControl.child_frame_twist
VMRobotControl.joint_relative_vpa
VMRobotControl.joint_relative_acceleration
VMRobotControl.autodiff_jacobian_column
VMRobotControl.child_rbstate
```

# TypeStableCollection

Let me illustrate the challenge: you have a robot with a mix of types of joint: 
revolute, prismatic and potentially other types too.
You want to write a function that can take any such robot and compute its kinematics.
If you simply make a list of joints, then it will not be concretely typed (e.g. `Vector{Any}. It is 
possible that small union optimisations would work for this example, but that doesn't scale to the 
number of different coordinate types or component types.)

Instead, we could use a tuple of the joints, but this would mean a different type for the robot for 
every arrangement of joints.
Instead we use a tuple of vectors: one vector for each different type.
This means that any robot made of only revolute and prismatic joints will be of the same type!
We store the order of computation separately, by storing indices into the tuple of vectors.

The tuple-of-vectors structure is called a `TypeStableCollection` and has drawn inspiration 
from [`TypeSortedCollection`](https://github.com/tkoolen/TypeSortedCollections.jl), but has some
difference in construction and indexing.

```@autodocs
Modules = [VMRobotControl.TypeStableCollections,]
```

# Transforms

```@autodocs
Modules = [VMRobotControl.Transforms,]
```

# Other API documentation

Here are the remaining docs not included in the main API

```@docs
VMRobotControl.CompiledMechanism
VMRobotControl.CompiledVirtualMechanismSystem
VMRobotControl.CompiledCoordID
VMRobotControl.reassign_coords
VMRobotControl.GenericComponent
VMRobotControl.Splines.curve_derivative
VMRobotControl.Splines.curve_second_derivative
VMRobotControl.ComponentData
VMRobotControl.frame_ancestor_matrix
VMRobotControl.reassign_joints
VMRobotControl.TimeFuncJoint
VMRobotControl._construct_coordinate_graph
VMRobotControl.URDF.parseURDFString
VMRobotControl.RSON.parseRSONString
VMRobotControl.RSON.parseRSONVersion
VMRobotControl.AbstractJointData
VMRobotControl.Spherical
VMRobotControl.Inertance
VMRobotControl.rigid_joint_twist
VMRobotControl._add_jacobian_transpose_times_force!
VMRobotControl._add_opspace_force!
VMRobotControl.jacobian_column
VMRobotControl.state_idxs
VMRobotControl.remake
VMRobotControl.reassign_frames
VMRobotControl.MechanismJoint
VMRobotControl.Splines.curve_position
VMRobotControl.EltypeCache
VMRobotControl.Dissipation
```
