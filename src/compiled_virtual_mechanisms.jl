"""
    CompiledVirtualMechanismSystem{T, M1, M2, TSC1, TSC2}

A compiled virtual mechanism system is a [`VirtualMechanismSystem`](@ref) that has been compiled into a form that is
efficient for computations/simulation. 
"""
struct CompiledVirtualMechanismSystem{T, M1<:CompiledMechanism, M2<:CompiledMechanism, TSC1<:TypeStableCollection, TSC2<:TypeStableCollection}
    name::String
    robot::M1
    virtual_mechanism::M2
    coordinates::Vector{TSC1}
    components::TSC2
    frame_id_map::Dict{String, VMSFrameID}
    joint_id_map::OrderedDict{String, VMSJointID}
    coord_id_map::Dict{String, VMSCoordID}
    component_id_map::Dict{String, VMSComponentID}
    function CompiledVirtualMechanismSystem(
            name::String,
            robot,
            virtual_mechanism,
            coordinates,
            components,
            frame_id_map,
            joint_id_map,
            coord_id_map,
            component_id_map)
        M1 = typeof(robot)
        M2 = typeof(virtual_mechanism)
        T = eltype(robot)
        @assert T<:Real
        @assert eltype(virtual_mechanism) == T
        TSC1 = eltype(coordinates)
        TSC2 = typeof(components)
        new{T, M1, M2, TSC1, TSC2}(
            name, 
            robot,
            virtual_mechanism,
            coordinates,
            components,
            frame_id_map,
            joint_id_map,
            coord_id_map,
            component_id_map
        )
    end
end
const CVMSystem{T, M1, M2, TSC1, TSC2} = CompiledVirtualMechanismSystem{T, M1, M2, TSC1, TSC2}

Base.eltype(::Type{CVMSystem{T, M1, M2, TSC1, TSC2}}) where {T, M1, M2, TSC1, TSC2} = T

name(vms::CVMSystem) = vms.name
robot_mechanism_type(::Type{CVMSystem{T, M1, M2, TSC1, TSC2}}) where {T, M1, M2, TSC1, TSC2} = M1
robot_mechanism_type(vms::CVMSystem) = robot_mechanism_type(typeof(vms))
virtual_mechanism_type(::Type{CVMSystem{T, M1, M2, TSC1, TSC2}}) where {T, M1, M2, TSC1, TSC2} = M2
virtual_mechanism_type(vms::CVMSystem) = virtual_mechanism_type(typeof(vms))

robot_ndof(vms::CVMSystem) = ndof(vms.robot)
virtual_mechanism_ndof(vms::CVMSystem) = ndof(vms.virtual_mechanism)
ndof(vms::CVMSystem) = robot_ndof(vms) + virtual_mechanism_ndof(vms)
coordinates(vms::CVMSystem) = vms.coordinates
components(vms::CVMSystem) = vms.components
storages(vms::CVMSystem) = filtertype(components(vms), Storage)
dissipations(vms::CVMSystem) = filtertype(components(vms), Dissipation)
visuals(vms::CVMSystem) = filtertype(components(vms), Visual)

function compile(vms::VirtualMechanismSystem)
    robot = compile(vms.robot)
    virtual_mechanism = compile(vms.virtual_mechanism)
    
    frame_id_map = _build_vms_frame_id_map(robot, virtual_mechanism)
    joint_id_map = _build_vms_joint_id_map(robot, virtual_mechanism)    
    coord_id_map, _coordinates = compile_vms_coordinates(
        coordinates(vms),
        robot,
        virtual_mechanism,
        frame_id_map,
        joint_id_map
    )
    component_id_map, _components = compile_vms_components(
        components(vms),
        robot, 
        virtual_mechanism,
        frame_id_map,
        joint_id_map,
        coord_id_map
    )
    CompiledVirtualMechanismSystem(
        name(vms),
        robot,
        virtual_mechanism,
        _coordinates,
        _components,
        frame_id_map,
        joint_id_map,
        coord_id_map,
        component_id_map,
    )
end

function compile_vms_coordinates(
        coordinates,
        robot,
        virtual_mechanism,
        frame_id_map,
        joint_id_map
    )
    for (id, coord_data) in coordinates
        @assert length(split(id, ".")) == 1
    end
    skip_if = (id) -> length(split(id, ".")) > 1
    coord_graph, ID_to_vertex_map = _construct_coordinate_graph(coordinates; skip_if)
    coord_id_computation_order = determine_coordinate_computation_order(
                                    coord_graph, ID_to_vertex_map, coordinates; is_computed=skip_if)
    compile_vms_coordinates_in_order(coord_id_computation_order, coordinates, robot, virtual_mechanism, frame_id_map, joint_id_map)
end

#################################
# VMS index dict
#################################
# These dictionaries allow us to reference frames, joints and coordinates within the robot or 
# virtual mechanism of a virtual-mechanism system. The coordinate index dict is constructed with
# only the coordinates of the robot and virtual mechanism and populated with the coordinates
# of the VMS later, in order.
function _build_vms_coord_id_map(robot, virtual_mechanism)
    coord_id_map = Dict{String, VMSCoordID}()
    for (str_id, compiled_id) in robot.rbtree.coord_id_map
        coord_id_map[".robot.$(str_id)"] = VMSCoordID{ON_ROBOT}(compiled_id)
    end
    for (str_id, compiled_id) in virtual_mechanism.rbtree.coord_id_map
        coord_id_map[".virtual_mechanism.$(str_id)"] = VMSCoordID{ON_VIRTUAL_MECHANISM}(compiled_id)
    end
    coord_id_map
end

function _build_vms_frame_id_map(robot, virtual_mechanism)
    frame_id_map = Dict{String, VMSFrameID}()
    for (str_id, compiled_id) in robot.rbtree.frame_id_map
        frame_id_map[".robot.$(str_id)"] = VMSFrameID{ON_ROBOT}(compiled_id)
    end
    for (str_id, compiled_id) in virtual_mechanism.rbtree.frame_id_map
        frame_id_map[".virtual_mechanism.$(str_id)"] = VMSFrameID{ON_VIRTUAL_MECHANISM}(compiled_id)
    end
    frame_id_map
end

function _build_vms_joint_id_map(robot, virtual_mechanism)
    joint_id_map = OrderedDict{String, VMSJointID}()
    for (str_id, compiled_id) in robot.rbtree.joint_id_map
        joint_id_map[".robot.$(str_id)"] = VMSJointID{ON_ROBOT}(compiled_id)
    end
    for (str_id, compiled_id) in virtual_mechanism.rbtree.joint_id_map
        joint_id_map[".virtual_mechanism.$(str_id)"] = VMSJointID{ON_VIRTUAL_MECHANISM}(compiled_id)
    end
    joint_id_map
end

function compile_vms_coordinates_in_order(
        coord_id_computation_order,
        coordinates,
        robot,
        virtual_mechanism,
        frame_id_map,
        joint_id_map
    )
    compiled_coords = TypeStableCollection[TypeStableCollection() for i in 1:length(coord_id_computation_order)]
    coord_id_map = _build_vms_coord_id_map(robot, virtual_mechanism)
    coord_cache_idx = 1
    for (depth, coord_ids) in enumerate(coord_id_computation_order)
        coord_data = getindex.((coordinates,),  coord_ids)
                    
        # Now we must replace any references to Coord ID strings with compiled coord IDs. We must do 
        # this in order starting with leafs, which do not depend on any other coordinates, so are trivial
        # to replace.        
        for (i, (id, coord_data_raw)) in enumerate(
                            zip(coord_ids, coord_data))
            coord_data_1 = _reassign_coords(coord_data_raw, coord_id_map)
            coord_data_2 = _reassign_frames(coord_data_1, frame_id_map)
            coord_data_3 = _reassign_joints(coord_data_2, joint_id_map)
            Nc = cache_size(coord_data_3)
            compiled_coord = CompiledCoord(coord_data_3, coord_cache_idx:(coord_cache_idx+Nc-1))
            coord_cache_idx += Nc
            compiled_coords[depth], idx = extend(compiled_coords[depth], compiled_coord)
            coord_id_map[id] = VMSCoordID{ON_SYSTEM}(CompiledCoordID(depth, idx))
        end
    end
    compiled_coords_ret = _promote_vec_of_typestable_collections_to_common_type(compiled_coords)
    coord_id_map, compiled_coords_ret
end

function _build_vms_component_id_map(robot, virtual_mechanism)
    component_id_map = Dict{String, VMSComponentID}()
    for (str_id, compiled_id) in robot.component_id_map
        component_id_map[".robot.$(str_id)"] = VMSComponentID{ON_ROBOT}(compiled_id)
    end
    for (str_id, compiled_id) in virtual_mechanism.component_id_map
        component_id_map[".virtual_mechanism.$(str_id)"] = VMSComponentID{ON_VIRTUAL_MECHANISM}(compiled_id)
    end
    component_id_map
end

function compile_vms_components(
        _components, 
        robot, 
        virtual_mechanism, 
        frame_id_map::AbstractDict, 
        joint_id_map::AbstractDict, 
        coord_id_map::AbstractDict
    )
    component_id_map = _build_vms_component_id_map(robot, virtual_mechanism)
    system_components = Dict{String, CompiledComponentID}()
    compiled_components = _compile_components!(
        _components, 
        system_components, 
        frame_id_map, 
        joint_id_map, 
        coord_id_map
    )
    for (id, compiled_id) in system_components
        component_id_map[id] = VMSComponentID{ON_SYSTEM}(compiled_id)
    end
    component_id_map, compiled_components
end

get_compiled_frameID(vms::CVMSystem, id::String)::VMSFrameID = get_compiled_frameID(vms.frame_id_map, id)
get_compiled_jointID(vms::CVMSystem, id::String)::VMSJointID = get_compiled_jointID(vms.joint_id_map, id)
get_compiled_coordID(vms::CVMSystem, id::String)::VMSCoordID = get_compiled_coordID(vms.coord_id_map, id)
get_compiled_componentID(vms::CVMSystem, id::String)::VMSComponentID = get_compiled_componentID(vms.component_id_map, id)


function get_correct_jointcollection(m::CVMSystem, jointID::VMSJointID)
    if _vms_location(jointID) == Val{ON_ROBOT}()
        return joints(m.robot)
    elseif _vms_location(jointID) == Val{ON_VIRTUAL_MECHANISM}()
        return joints(m.virtual_mechanism)
    else
        error("Invalid jointID type: $(typeof(jointID))")
    end
end

function get_correct_coordcollection(m::CVMSystem, coordID::VMSCoordID)
    if _vms_location(coordID) == Val{ON_ROBOT}()
        return coordinates(m.robot)
    elseif _vms_location(coordID) == Val{ON_VIRTUAL_MECHANISM}()
        return coordinates(m.virtual_mechanism)
    elseif _vms_location(coordID) == Val{ON_SYSTEM}()
        return coordinates(m)
    else
        error("Invalid coordID type: $(typeof(coordID))")
    end
end

function get_correct_componentcollection(m::CVMSystem, componentID::VMSComponentID)
    if _vms_location(componentID) == Val{ON_ROBOT}()
        return components(m.robot)
    elseif _vms_location(componentID) == Val{ON_VIRTUAL_MECHANISM}()
        return components(m.virtual_mechanism)
    elseif _vms_location(componentID) == Val{ON_SYSTEM}()
        return components(m)
    else
        error("Invalid componentID type: $(typeof(componentID))")
    end
end

Base.getindex(m::CVMSystem, id::VMSJointID) = get_correct_jointcollection(m, id)[id.idx]
Base.getindex(m::CVMSystem, id::VMSCoordID) = get_correct_coordcollection(m, id)[id.idx]
Base.getindex(m::CVMSystem, id::VMSComponentID) = get_correct_componentcollection(m, id)[id.idx]

Base.setindex!(m::CVMSystem, val, id::VMSJointID{J}) where J = (get_correct_jointcollection(m, id)[id.idx] = val)
Base.setindex!(m::CVMSystem, val, id::VMSCoordID{C}) where C = (get_correct_coordcollection(m, id)[id.idx] = val)
Base.setindex!(m::CVMSystem, val, id::VMSComponentID{C}) where C = (get_correct_componentcollection(m, id)[id.idx] = val)

#################################

struct VirtualMechanismSystemStateIdxs
    mechanism_idxs::UnitRange{Int}
    vm_idxs::UnitRange{Int}
end

Base.getindex(v::AbstractVector, idxs::VirtualMechanismSystemStateIdxs) = (v[idxs.mechanism_idxs], v[idxs.vm_idxs])

"""
    state_idxs(Σ::CompiledVirtualMechanismSystem)

Returns the indices for getting the state from the ODE vector.

# Returns
qʳ_idxs::UnitRange{Int} - The indices for the robot configuration
qᵛ_idxs::UnitRange{Int} - The indices for the virtual mechanism configuration
q̇ʳ_idxs::UnitRange{Int} - The indices for the robot velocity
q̇ᵛ_idxs::UnitRange{Int} - The indices for the virtual mechanism velocity
"""
function state_idxs(Σ::CompiledVirtualMechanismSystem)
    # Returns the idxs for getting the state from the ODE vector. idxs are also given for the reference
    # states, 
    N_robot_configs, N_control_configs = config_size(Σ.robot), config_size(Σ.virtual_mechanism)
    N_robot_velocities, N_control_velocities = velocity_size(Σ.robot), velocity_size(Σ.virtual_mechanism)

    qʳ_idxs = (1:N_robot_configs)
    qᵛ_idxs = (1:N_control_configs)  .+ N_robot_configs
    q̇ʳ_idxs = (1:N_robot_velocities) .+ (N_robot_configs+N_control_configs)
    q̇ᵛ_idxs = (1:N_control_velocities) .+ (N_robot_configs+N_control_configs+N_robot_velocities)
    VirtualMechanismSystemStateIdxs(qʳ_idxs, qᵛ_idxs), VirtualMechanismSystemStateIdxs(q̇ʳ_idxs, q̇ᵛ_idxs)
end

function q_idxs(Σ::CompiledVirtualMechanismSystem)
    N_robot_configs, N_control_configs = config_size(Σ.robot), config_size(Σ.virtual_mechanism)
    qʳ_idxs = (1:N_robot_configs)
    qᵛ_idxs = (1:N_control_configs)  .+ N_robot_configs
    VirtualMechanismSystemStateIdxs(qʳ_idxs, qᵛ_idxs)
end

function q̇_idxs(Σ::CompiledVirtualMechanismSystem)
    N_robot_configs, N_control_configs = config_size(Σ.robot), config_size(Σ.virtual_mechanism)
    N_robot_velocities, N_control_velocities = velocity_size(Σ.robot), velocity_size(Σ.virtual_mechanism)
    q̇ʳ_idxs = (1:N_robot_velocities) .+ (N_robot_configs+N_control_configs)
    q̇ᵛ_idxs = (1:N_control_velocities) .+ (N_robot_configs+N_control_configs+N_robot_velocities)
    VirtualMechanismSystemStateIdxs(q̇ʳ_idxs, q̇ᵛ_idxs)
end

function assemble_state(Σ::CVMSystem; q_robot::Vector{T}, q_vm::Vector{T}, q̇_robot::Vector{T}, q̇_vm::Vector{T}) where T
    # Converts the state of the robot/controller system into a vector for use as an ODE state.
    # Note that because the reference states are not part of the ODE state, they are not included here.
    q_idxs, q̇_idxs = state_idxs(Σ)
    qʳ_idxs, qᵛ_idxs = q_idxs.mechanism_idxs, q_idxs.vm_idxs
    q̇ʳ_idxs, q̇ᵛ_idxs = q̇_idxs.mechanism_idxs, q̇_idxs.vm_idxs

    @assert length(q_robot) == length(qʳ_idxs) "q_robot must have length $(length(qʳ_idxs))"
    @assert length(q_vm) == length(qᵛ_idxs) "q_vm must have length $(length(qᵛ_idxs))"
    @assert length(q̇_robot) == length(q̇ʳ_idxs) "q̇_robot must have length $(length(q̇ʳ_idxs))"
    @assert length(q̇_vm) == length(q̇ᵛ_idxs) "q̇_vm must have length $(length(q̇ᵛ_idxs))"

    x0 = zeros(T, length(qʳ_idxs) + length(qᵛ_idxs) + length(q̇ʳ_idxs) + length(q̇ᵛ_idxs))

    x0[qʳ_idxs] .= q_robot
    x0[qᵛ_idxs] .= q_vm
    x0[q̇ʳ_idxs] .= q̇_robot
    x0[q̇ᵛ_idxs] .= q̇_vm
    x0
end

function assemble_state(Σ::CVMSystem, q::Tuple{Vector{T}, Vector{T}}, q̇::Tuple{Vector{T}, Vector{T}}) where T
    assemble_state(Σ; q_robot=q[1], q_vm=q[2], q̇_robot=q̇[1], q̇_vm=q̇[2])
end
