opspaceConfiguration(args...) = error("Please use new interface: 'configuration(cache, coord)'`")
opspaceVelocity(args...) = error("Please use new interface: 'velocity(cache, coord)'`")
opspaceVpa(args...) = error("Please use new interface: 'vpa(cache, coord)'`")
opspaceJacobian(args...) = error("Please use new interface: 'jacobian(cache, coord)'`")


const __COMPILED_COORD_TYPE = Union{CompiledCoord, CompiledCoordID, VMSCoordID}

@inline function _to_mechanism_coord(cache, c::__COMPILED_COORD_TYPE)::CompiledCoord
    if isa(c, CompiledCoord)
        c
    else
        c::CompiledCoordID
        coordinate(cache, c)
    end
end

"""
configuration(cache, coordinate)

Returns the configuration of the operation space coordinate `coordinate` z.
"""
function configuration(cache::CacheBundle, c::__COMPILED_COORD_TYPE)
    C = coord_type(c)
    N, T = length(C), eltype(cache)
    _configuration(cache, c)::SVector{N, T}
end

"""
    velocity(cache, coordinate)

Returns the velocity of the operation space coordinate `coordinate` ż, the 
derivative of the configuration.
"""
function velocity(cache::CacheBundle, c::__COMPILED_COORD_TYPE)
    C = coord_type(c)
    N, T = length(C), eltype(cache)
    _velocity(cache, c)::SVector{N, T}
end
"""
    jacobian(cache, coordinate)

Return the jacobian of the operation space coordinate `coordinate` J, such that
ż = Jq̇, where ż is the opspaceVelocity and q̇ are the joint velocities.
"""
function jacobian(cache::CacheBundle, c::__COMPILED_COORD_TYPE)
    C = coord_type(c)
    N, T = length(C), eltype(cache)
    _jacobian(cache, c) # TODO add type assertion
end


"""
    acceleration(cache, coordinate)

Returns the acceleration of the operation space coordinate `coordinate`
z̈, the second derivative of the configuration. Depending on the type of cache,
this may either be velocity product acceleration or total acceleration.
"""
function acceleration(cache::CacheBundle, c::__COMPILED_COORD_TYPE)
    C = coord_type(c)
    N, T = length(C), eltype(cache)
    _acceleration(cache, c)::SVector{N, T}
end

# Legacy, vpa is the same as acceleration
vpa(cache::CacheBundle, c::__COMPILED_COORD_TYPE) = acceleration(cache, c)

# If not implemented for the coordinate data type, fallback error message
_configuration(cache::MechanismCacheBundle, ::CoordinateData) = error("Not implemented")
_velocity(cache::MechanismCacheBundle, ::CoordinateData) = error("Not implemented")
_jacobian(cache::MechanismCacheBundle, ::CoordinateData) = error("Not implemented")
_acceleration(cache::MechanismCacheBundle, ::CoordinateData) = error("Not implemented")

# If called with a compiled coordinate ID, get the coordinate data from the cache
_configuration(cache::MechanismCacheBundle, c::CompiledCoordID) = _configuration(cache, coordinate(cache, c))
_velocity(cache::MechanismCacheBundle, c::CompiledCoordID) = _velocity(cache, coordinate(cache, c))
_jacobian(cache::MechanismCacheBundle, c::CompiledCoordID) = _jacobian(cache, coordinate(cache, c))
_acceleration(cache::MechanismCacheBundle, c::CompiledCoordID) = _acceleration(cache, coordinate(cache, c))

 # If called with a compiled VMS coordinate, get the coordinate data from the cache, and the appropriate subcache
_configuration(cache::VirtualMechanismSystemCacheBundle, c::VMSCoordID) = _configuration(get_vms_subcache(cache, c), coordinate(cache, c))
_velocity(cache::VirtualMechanismSystemCacheBundle, c::VMSCoordID) = _velocity(get_vms_subcache(cache, c), coordinate(cache, c))
_acceleration(cache::VirtualMechanismSystemCacheBundle, c::VMSCoordID) = _acceleration(get_vms_subcache(cache, c), coordinate(cache, c))

# Useful error messages
const VMS_ACCESS_WITH_COMPILEDCOORDID_ERRMSG = "Attempting to access a mechanism coordinate from a VirtualMechanismSystemCacheBundle. A VMSCoordID must be used instead. Did you add a component to a VirtualMechanismSystem with the wrong coordinate?"
_configuration(cache::VirtualMechanismSystemCacheBundle, c::CompiledCoordID) = error(VMS_ACCESS_WITH_COMPILEDCOORDID_ERRMSG)
_velocity(cache::VirtualMechanismSystemCacheBundle, c::CompiledCoordID) = error(VMS_ACCESS_WITH_COMPILEDCOORDID_ERRMSG)
_jacobian(cache::VirtualMechanismSystemCacheBundle, c::CompiledCoordID) = error(VMS_ACCESS_WITH_COMPILEDCOORDID_ERRMSG)
_acceleration(cache::VirtualMechanismSystemCacheBundle, c::CompiledCoordID) = error(VMS_ACCESS_WITH_COMPILEDCOORDID_ERRMSG)

torques_vector_offset(cache::VMSCacheBundle, loc::Val{ON_ROBOT}) = 0
torques_vector_offset(cache::VMSCacheBundle, loc::Val{ON_VIRTUAL_MECHANISM}) = ndof(cache.vms.robot)
torques_vector_offset(cache::VMSCacheBundle, loc::Val{ON_SYSTEM}) = 0

# TODO Jacobians relate explicitly to the mechanism.
# The vms subcaches have a space for the jacobian for each coordinate, but when we return
# from this function we need to return the jacobian and we lose the information about the
# mechanism. 
# A general solution to this might be to return a tuple of the jacobians for each mechanism
function _jacobian(cache::VirtualMechanismSystemCacheBundle, c::VMSCoordID)
    vms = cache.vms
    Nr, Nv, T = ndof(vms.robot), ndof(vms.virtual_mechanism), eltype(cache)
    Nc = length(coord_type(c))
    loc = _vms_location(c)
    if loc == Val{ON_ROBOT}()
        J = _jacobian(robot_cache(cache), coordinate(cache, c))
        @assert size(J) == (Nc, Nr)
    elseif loc == Val{ON_VIRTUAL_MECHANISM}()
        J = _jacobian(virtual_mechanism_cache(cache), coordinate(cache, c))
        @assert size(J) == (Nc, Nv)
    elseif loc == Val{ON_SYSTEM}()
        J = _jacobian(cache, coordinate(cache, c))
        # @assert _loc == Val{ON_SYSTEM}() "Unexpected location \"$_loc\" for coordinate \"$c\""
        @assert size(J) == (Nc, Nr + Nv)
    else
        throw(ErrorException("Invalid VMSCoordIdx loc=`$loc`"))
    end
    return J

    # _jacobian(get_vms_subcache(cache, c), coordinate(cache, c))
end
