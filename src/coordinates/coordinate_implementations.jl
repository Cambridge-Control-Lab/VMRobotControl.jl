const CMC{C} = CompiledCoord{C} where C

####################################################################################################
# Utils
####################################################################################################

function _vms_jacobian_result_view(cache, J, id)
    # Often, when constructing a jacobian for a virtual mechanism system, we only need to 
    # update the part of the jacobian that corresponds to the robot, or to the virtual mechanism.
    # This function returns a view of the jacobian that corresponds to that part
    if cache isa MechanismCacheBundle
        @assert id isa AbstractCompiledMechanismIndex
        return J
    elseif id isa AbstractCompiledVirtualMechanismSystemIndex
        @assert cache isa VirtualMechanismSystemCacheBundle
        vms = cache.vms
        Nr, Nv = ndof(vms.robot), ndof(vms.virtual_mechanism)
        @assert size(J, 2) == Nr + Nv
        Nc = size(J, 1)
        loc = _vms_location(id)
        if loc == Val{ON_ROBOT}()
            ret = @view J[1:Nc, 1:Nr] 
            @assert size(ret) == (Nc, Nr)
            return ret
        elseif loc == Val{ON_VIRTUAL_MECHANISM}()
            ret = @view J[1:Nc, Nr+1:Nr+Nv]
            @assert size(ret) == (Nc, Nv)
            return ret
        elseif loc == Val{ON_SYSTEM}()
            ret = @view J[1:Nc, 1:Nr+Nv] # View of the whole matrix
            @assert size(ret) == (Nc, Nr+Nv)
            return ret
        else
            throw(ErrorException("Invalid VMSCoordIdx loc=`$loc`"))
        end
    else
        error("Unknown source type")
    end
end

####################################################################################################
# CoordDifference
####################################################################################################

@inline __configuration!(cache::CacheBundle, c::CMC{<:CoordDifference}) = nothing 
@inline __velocity!(cache::CacheBundle, c::CMC{<:CoordDifference}) = nothing 

function __jacobian!(cache::CacheBundle, c::CMC{<:CoordDifference})
    J_ret = J_cache_view(cache, c)
    J_ret_p = _vms_jacobian_result_view(cache, J_ret, c.coord_data.parent)
    J_ret_c = _vms_jacobian_result_view(cache, J_ret, c.coord_data.child)

    fill!(J_ret, zero(eltype(J_ret)))
    J_p = _jacobian(cache, c.coord_data.parent)
    J_c = _jacobian(cache, c.coord_data.child)

    # J_ret = J_p - J_c
    J_ret_p .+= J_p
    J_ret_c .-= J_c
    nothing
end

@inline __acceleration!(cache::CacheBundle, c::CMC{<:CoordDifference}) = nothing

function _configuration(cache::CacheBundle, c::CMC{<:CoordDifference})
    z_p = _configuration(cache, c.coord_data.parent)
    z_c = _configuration(cache, c.coord_data.child)
    z_p .- z_c
end
function _velocity(cache::CacheBundle, c::CMC{<:CoordDifference})
    ż_p = _velocity(cache, c.coord_data.parent)
    ż_c = _velocity(cache, c.coord_data.child)
    ż_p .- ż_c
end
function _jacobian(cache::CacheBundle, c::CMC{<:CoordDifference})
    J_cache_view(cache, c)
end

function _acceleration(cache::CacheBundle, c::CMC{<:CoordDifference})
    v̇_p = _acceleration(cache, c.coord_data.parent)
    v̇_c = _acceleration(cache, c.coord_data.child)
    v̇_p .- v̇_c
end


####################################################################################################
# CoordSum
####################################################################################################

@inline __configuration!(cache::CacheBundle, c::CMC{<:CoordSum}) = nothing 
@inline __velocity!(cache::CacheBundle, c::CMC{<:CoordSum}) = nothing 
function __jacobian!(cache::CacheBundle, c::CMC{<:CoordSum})
    J_ret = J_cache_view(cache, c)
    J_ret_1 = _vms_jacobian_result_view(cache, J_ret, c.coord_data.c1)
    J_ret_2 = _vms_jacobian_result_view(cache, J_ret, c.coord_data.c2)
    
    J1 = _jacobian(cache, c.coord_data.c1)
    J2 = _jacobian(cache, c.coord_data.c2)
    
    # J_ret = J1 + J2
    fill!(J_ret, zero(eltype(J_ret)))
    J_ret_1 .+= J1
    J_ret_2 .+= J2
    nothing
end

@inline __acceleration!(cache::CacheBundle, c::CMC{<:CoordSum}) = nothing

function _configuration(cache::CacheBundle, c::CMC{<:CoordSum})
    z_1 = _configuration(cache, c.coord_data.c1)
    z_2 = _configuration(cache, c.coord_data.c2)
    z_1 .+ z_2
end
function _velocity(cache::CacheBundle, c::CMC{<:CoordSum})
    ż_1 = _velocity(cache, c.coord_data.c1)
    ż_2 = _velocity(cache, c.coord_data.c2)
    ż_1 .+ ż_2
end
function _jacobian(cache::MechanismCacheBundle, c::CMC{<:CoordSum})
    J_cache_view(cache, c)
end
_jacobian(cache::VirtualMechanismSystemCacheBundle, c::CMC{<:CoordSum}) = J_cache_view(cache, c)

function _acceleration(cache::CacheBundle, c::CMC{<:CoordSum})
    v̇_1 = _acceleration(cache, c.coord_data.c1)
    v̇_2 = _acceleration(cache, c.coord_data.c2)
    v̇_1 .+ v̇_2
end

####################################################################################################
# CoordStack
####################################################################################################

@inline __configuration!(cache::CacheBundle, c::CMC{<:CoordStack}) = nothing
@inline __velocity!(cache::CacheBundle, c::CMC{<:CoordStack}) = nothing
function __jacobian!(cache::CacheBundle, c::CMC{<:CoordStack})
    J_ret = J_cache_view(cache, c)
    fill!(J_ret, zero(eltype(J_ret)))
    J_ret_1 = _vms_jacobian_result_view(cache, J_ret, c.coord_data.c1)
    J_ret_2 = _vms_jacobian_result_view(cache, J_ret, c.coord_data.c2)
    J1 = _jacobian(cache, c.coord_data.c1)
    J2 = _jacobian(cache, c.coord_data.c2)
    # Instead, do it manually
    Nc1 = size(J1, 1)
    J_ret_1[1:Nc1, :] .= J1
    J_ret_2[Nc1+1:end, :] .= J2
    nothing
end
@inline __acceleration!(cache::CacheBundle, c::CMC{<:CoordStack}) = nothing

function _configuration(cache::CacheBundle, c::CMC{<:CoordStack})
    z1 = _configuration(cache, c.coord_data.c1)
    z2 = _configuration(cache, c.coord_data.c2)
    ret = vcat(z1, z2)
    ret
end
function _velocity(cache::CacheBundle, c::CMC{<:CoordStack})
    ż1 = _velocity(cache, c.coord_data.c1)
    ż2 = _velocity(cache, c.coord_data.c2)
    ret = vcat(ż1, ż2)
    ret
end
function _jacobian(cache::CacheBundle, c::CMC{<:CoordStack})
    J_cache_view(cache, c)
end
function _acceleration(cache::CacheBundle, c::CMC{<:CoordStack})
    v̇1 = _acceleration(cache, c.coord_data.c1)
    v̇2 = _acceleration(cache, c.coord_data.c2)
    ret = vcat(v̇1, v̇2)
    ret
end

####################################################################################################
# CoordSlice
####################################################################################################

@inline __configuration!(cache::CacheBundle, c::CMC{<:CoordSlice}) = nothing
@inline __velocity!(cache::CacheBundle, c::CMC{<:CoordSlice}) = nothing
@inline __jacobian!(cache::CacheBundle, c::CMC{<:CoordSlice}) = nothing
@inline __acceleration!(cache::CacheBundle, c::CMC{<:CoordSlice}) = nothing

function _configuration(cache::CacheBundle, c::CMC{<:CoordSlice})
    _configuration(cache, c.coord_data.coord)[c.coord_data.idxs]
end
function _velocity(cache::CacheBundle, c::CMC{<:CoordSlice})
    _velocity(cache, c.coord_data.coord)[c.coord_data.idxs]
end
function _jacobian(cache::CacheBundle, c::CMC{<:CoordSlice})
    @assert _vms_location(c.coord_data.coord) == Val{ON_SYSTEM}()
    J = _vms_jacobian_result_view(cache, _jacobian(cache, c.coord_data.coord), c.coord_data.coord)
    @view J[c.coord_data.idxs, :] # TODO test
end
function _acceleration(cache::CacheBundle, c::CMC{<:CoordSlice})
    _acceleration(cache, c.coord_data.coord)[c.coord_data.idxs]
end

####################################################################################################
# ConstCoord
####################################################################################################

@inline __configuration!(cache::CacheBundle, c::CMC{<:ConstCoord}) = nothing
@inline __velocity!(cache::CacheBundle, c::CMC{<:ConstCoord}) = nothing
function __jacobian!(cache::CacheBundle, c::CMC{<:ConstCoord})
    J = J_cache_view(cache, c)
    fill!(J, zero(eltype(J))) # TODO do this once rather than every time
    nothing
end
@inline __acceleration!(cache::CacheBundle, c::CMC{<:ConstCoord}) = nothing

_configuration(cache::CacheBundle, c::CMC{<:ConstCoord{Nc, Tc}}) where {Tc, Nc} = SVector{Nc, eltype(cache)}(c.coord_data.val)
_velocity(cache::CacheBundle, c::CMC{<:ConstCoord{Nc, Tc}}) where {Tc, Nc} = zero(SVector{Nc, eltype(cache)})
_acceleration(cache::CacheBundle, c::CMC{<:ConstCoord{Nc, Tc}}) where {Tc, Nc} = zero(SVector{Nc, eltype(cache)})
_jacobian(cache::CacheBundle, c::CMC{<:ConstCoord}) = J_cache_view(cache, c)

####################################################################################################
# ReferenceCoord
####################################################################################################

@inline __configuration!(cache::CacheBundle, c::CMC{<:ReferenceCoord}) = nothing
@inline __velocity!(cache::CacheBundle, c::CMC{<:ReferenceCoord}) = nothing
function __jacobian!(cache::CacheBundle, c::CMC{<:ReferenceCoord})
    J = J_cache_view(cache, c)
    fill!(J, zero(eltype(J))) # TODO do this once rather than every time
    nothing
end
@inline __acceleration!(cache::CacheBundle, c::CMC{<:ReferenceCoord}) = nothing

_configuration( cache::CacheBundle, c::CMC{<:ReferenceCoord{Nc, Tc}}) where {Tc, Nc} = SVector{Nc, eltype(cache)}(c.coord_data.val[])
_velocity(      cache::CacheBundle, c::CMC{<:ReferenceCoord{Nc, Tc}}) where {Tc, Nc} = SVector{Nc, eltype(cache)}(c.coord_data.vel[])
_acceleration(           cache::CacheBundle, c::CMC{<:ReferenceCoord{Nc, Tc}}) where {Tc, Nc} = zero(SVector{Nc, eltype(cache)})
_jacobian(      cache::MechanismCacheBundle, c::CMC{<:ReferenceCoord}) = J_cache_view(cache, c)
_jacobian(      cache::VMSCacheBundle,       c::CMC{<:ReferenceCoord}) = J_cache_view(cache, c)

####################################################################################################
# JointSubspace
####################################################################################################
@inline __configuration!(cache::CacheBundle, c::CMC{<:JointSubspace}) = nothing
@inline __velocity!(cache::CacheBundle, c::CMC{<:JointSubspace}) = nothing
function __jacobian!(cache::CacheBundle, c::CMC{<:JointSubspace})
    J = _vms_jacobian_result_view(cache, J_cache_view(cache, c), c.coord_data.joint)
    fill!(J, zero(eltype(J))) # TODO do this once rather than every time
    joint = cache[c.coord_data.joint]
    for (i, idx) in enumerate(q̇_idxs(joint))
        J[i, idx] = one(eltype(J))
    end
    nothing
end
@inline __acceleration!(cache::CacheBundle, c::CMC{<:JointSubspace}) = nothing

_configuration(cache::CacheBundle, c::CMC{<:JointSubspace}) = get_q(cache, c.coord_data.joint)
_velocity(cache::CacheBundle, c::CMC{<:JointSubspace}) = get_q̇(cache, c.coord_data.joint)
_acceleration(cache::CacheBundle, c::CMC{<:JointSubspace}) = zero(get_q̇(cache, c.coord_data.joint))
_jacobian(cache::CacheBundle, c::CMC{<:JointSubspace}) = J_cache_view(cache, c)

####################################################################################################
# FramePoint
####################################################################################################

function __configuration!(cache::CacheBundle, mc::CMC{<:FramePoint})
    c = mc.coord_data
    pᵃ = c.point
    T⁰ᵃ = get_transform(cache, c.frameID)
    
    z = transform(T⁰ᵃ, pᵃ)

    z_cache_view(cache, mc) .= z
    nothing
end

function __velocity!(cache::CacheBundle, mc::CMC{<:FramePoint})
    c = mc.coord_data
    pᵃ = c.point
    T⁰ᵃ = get_transform(cache, c.frameID)
    R⁰ᵃ = rotor(T⁰ᵃ)
    v⁰ᵃ = get_linear_vel(cache, c.frameID)
    ω⁰ᵃ = get_angular_vel(cache, c.frameID)
    
    ż = v⁰ᵃ + cross(ω⁰ᵃ, rotate(R⁰ᵃ, pᵃ))
    
    ż_cache_view(cache, mc) .= ż
    nothing
end

function __jacobian!(cache::CacheBundle, mc::CMC{<:FramePoint})
    c = mc.coord_data
    Jz = _vms_jacobian_result_view(cache, J_cache_view(cache, mc), c.frameID)
    Jᵥ⁰ᵃ = get_linear_jacobian(cache, c.frameID)
    Jω⁰ᵃ = get_angular_jacobian(cache, c.frameID)
    T⁰ᵃ = get_transform(cache, c.frameID)
    
    pᵃ = c.point
    R⁰ᵃ = rotor(T⁰ᵃ)
    δᵒ = rotate(R⁰ᵃ, pᵃ)

    #= 
    Equivalent method
    z = configuration(cache, mc)
    o = origin(T⁰ᵃ)
    δ = z - o
    This way is slightly slower as we have to get z from the cache, which takes time
    =#

    # Jz = Jᵥ⁰ᵃ - skew(R⁰ᵃ(pᵃ)) * Jω⁰ᵃ
    skew_p::SMatrix{3, 3} = skew(δᵒ)
    for i in axes(Jz, 1), j in axes(Jz, 2)
        Jz[i, j] = Jᵥ⁰ᵃ[i, j] - skew_p[i, :]' * Jω⁰ᵃ[SVector(1, 2, 3), j]
    end
    nothing
end

function __acceleration!(bundle::CacheBundle, mc::CMC{<:FramePoint})
    c = mc.coord_data
    pᵃ = c.point
    T⁰ᵃ = get_transform(bundle, c.frameID)
    R⁰ᵃ = rotor(T⁰ᵃ)
    ω⁰ᵃ = get_angular_vel(bundle, c.frameID)
    α⁰ᵃ = _get_linear_acc(bundle, c.frameID)
    Ω⁰ᵃ = _get_angular_acc(bundle, c.frameID)

    δ⁰ = rotate(R⁰ᵃ, pᵃ)
    αᵢ = α⁰ᵃ + cross(Ω⁰ᵃ, δ⁰) + cross(ω⁰ᵃ, cross(ω⁰ᵃ, δ⁰))

    α_cache_view(bundle, mc) .= αᵢ
    nothing
end

function __propagate_opspace_force!(cache::CacheBundle, mc::CMC{<:FramePoint})
    c = mc.coord_data
    f = f_cache_view(cache, mc)[SVector(1, 2, 3)] # Get the force applied to the coord as an SVector
    fID = c.frameID
    T⁰ᵃ = get_transform(cache, fID)
    R⁰ᵃ = rotor(T⁰ᵃ)
    pᵃ = c.point
    δ⁰ = rotate(R⁰ᵃ, pᵃ)
    τ = cross(δ⁰, f)

    get_frame_forces(cache)[fID] += f
    get_frame_torques(cache)[fID] += τ

    nothing
end

_configuration(bundle::CacheBundle, mc::CMC{<:FramePoint}) = z_cache_view(bundle, mc)[SVector(1, 2, 3)]
_velocity(bundle::CacheBundle, mc::CMC{<:FramePoint}) = ż_cache_view(bundle, mc)[SVector(1, 2, 3)]
_jacobian(bundle::CacheBundle, mc::CMC{<:FramePoint}) = J_cache_view(bundle, mc)
_acceleration(bundle::CacheBundle, mc::CMC{<:FramePoint}) = α_cache_view(bundle, mc)[SVector(1, 2, 3)]

####################################################################################################
# FrameOrigin
####################################################################################################
__configuration!(cache::CacheBundle, c::CMC{<:FrameOrigin}) = nothing
__velocity!(cache::CacheBundle, c::CMC{<:FrameOrigin}) = nothing
function __jacobian!(cache::CacheBundle, c::CMC{<:FrameOrigin})
    J = J_cache_view(cache, c)
    fill!(J, zero(eltype(J))) # TODO do this once rather than every time
    J_ret = _vms_jacobian_result_view(cache, J, c.coord_data.frameID)
    Jp = get_linear_jacobian(cache, c.coord_data.frameID)
    J_ret .= Jp
    nothing
end
__acceleration!(cache::CacheBundle, c::CMC{<:FrameOrigin}) = nothing
__propagate_opspace_force!(cache::CacheBundle, c::CMC{<:FrameOrigin}) = get_frame_forces(cache)[c.coord_data.frameID] += f_cache_view(cache, c)

_configuration(cache::CacheBundle, c::CMC{<:FrameOrigin}) = origin(get_transform(cache, c.coord_data.frameID))
_velocity(cache::CacheBundle, c::CMC{<:FrameOrigin}) = get_linear_vel(cache, c.coord_data.frameID)
_jacobian(cache::CacheBundle, c::CMC{<:FrameOrigin}) = J_cache_view(cache, c)
_acceleration(cache::CacheBundle, c::CMC{<:FrameOrigin}) = _get_linear_acc(cache, c.coord_data.frameID)

####################################################################################################
# QuaternionAttitude
####################################################################################################

function __configuration!(cache::CacheBundle, cmc::CMC{<:QuaternionAttitude})
    c = cmc.coord_data
    rotor_wf = rotor(get_transform(cache, c.frameID)) # Rotate from `frame` to `world`
    rotor_tw = inv(c.target_rotation) # Rotate from `world` to `target`
    rotor_tf = rotate(rotor_tw, rotor_wf)
    z_cache_view(cache, cmc) .= bivector(rotor_tf)
    nothing
end
function __velocity!(cache::CacheBundle, cmc::CMC{<:QuaternionAttitude})
    c = cmc.coord_data
    rotor_wf = rotor(get_transform(cache, c.frameID))
    drotor_wf = quaternion_derivative(rotor_wf, get_angular_vel(cache, c.frameID))
    
    rotor_tw = inv(c.target_rotation)
    drotor_tf = rotate(rotor_tw, drotor_wf, Val{false}()) # Dont normalize as this is a velocity
    ż_cache_view(cache, cmc) .= bivector(drotor_tf)
    nothing
end
function __jacobian!(cache::CacheBundle, cmc::CMC{<:QuaternionAttitude})
    c = cmc.coord_data
    rotor_wf = rotor(get_transform(cache, c.frameID))
    rotor_tw = inv(c.target_rotation)
    E_wf = Transforms.quaternion_derivative_propagation(rotor_wf)
    E_tw = Transforms.quatmul_matrix(rotor_tw)
    E = E_tw * E_wf
    Jω = get_angular_jacobian(cache, c.frameID) 
    # J_wf = E * Jω
    # P = @SMatrix [0 1 0 0; 0 0 1 0; 0 0 0 1]
    # P * J_tf

    # Update jacobian for cmc, parts corresponding to c.frameID
    J = J_cache_view(cache, cmc)
    fill!(J, zero(eltype(J)))
    J_ret = _vms_jacobian_result_view(cache, J, c.frameID)

    for i in axes(J_ret, 1), j in axes(J_ret, 2)
        J_ij = zero(eltype(J_ret))
        for k in 1:3
            # Ignore first row
            J_ij += E[i+1, k] * Jω[k, j]
        end
        J_ret[i, j] = J_ij
    end
    nothing
end
function __acceleration!(cache::CacheBundle, cmc::CMC{<:QuaternionAttitude})
    c = cmc.coord_data
    rotor_wf = rotor(get_transform(cache, c.frameID))
    drotor_wf = quaternion_derivative(rotor_wf, get_angular_vel(cache, c.frameID))
    

    ω_wf = get_angular_vel(cache, c.frameID)
    ẇ_wf = _get_angular_acc(cache, c.frameID)
    ddrotor_wf_1 = quaternion_derivative(rotor_wf, ẇ_wf)
    ddrotor_wf_2 =  quaternion_derivative(drotor_wf, ω_wf)
    ddrotor_wf = Rotor(scalar(ddrotor_wf_1) + scalar(ddrotor_wf_2), bivector(ddrotor_wf_1) + bivector(ddrotor_wf_2), Val{false}())

    rotor_tw = inv(c.target_rotation)
    ddrotor_tf = rotate(rotor_tw, ddrotor_wf, Val{false}()) # Dont normalize as this is a velocity
    
    bivector(ddrotor_tf)
    α_cache_view(cache, cmc) .= bivector(ddrotor_tf)
end

# function __propagate_opspace_force!(cache::CacheBundle, cmc::CMC{<:QuaternionAttitude}, f)
#     c = cmc.coord_data
#     fID = c.frameID

#     T⁰ᵃ = get_transform(cache, fID)
#     R⁰ᵃ = rotor(T⁰ᵃ)
#     ω⁰ᵃ = get_angular_vel(cache, fID)
#     Ṙ⁰ᵃ = quaternion_derivative(R⁰ᵃ, ω⁰ᵃ)

#     Rᵗ⁰ = inv(c.target_rotation)

#     Ṙᵗᵃ = rotate(Rᵗ⁰, Ṙ⁰ᵃ, Val{false}()) # Dont normalize as this is a velocity

#     ż = bivector(Ṙᵗᵃ)

#     nothing

    
#     c = cmc.coord_data
#     R⁰ᵃ = rotor(T⁰ᵃ)
#     τ = cross(configuration(cache, cmc), f)
#     get_frame_torques(cache)[fID] .+= τ
#     nothing
# end

_configuration(cache::CacheBundle, cmc::CMC{<:QuaternionAttitude}) = SVector{length(cmc), eltype(cache)}(z_cache_view(cache, cmc))
_velocity(cache::CacheBundle, cmc::CMC{<:QuaternionAttitude}) = SVector{length(cmc), eltype(cache)}(ż_cache_view(cache, cmc))
_jacobian(cache::CacheBundle, cmc::CMC{<:QuaternionAttitude}) = J_cache_view(cache, cmc)
_acceleration(cache::CacheBundle, cmc::CMC{<:QuaternionAttitude}) = SVector{length(cmc), eltype(cache)}(α_cache_view(cache, cmc))

####################################################################################################
# FrameAngularVelocity
####################################################################################################
__configuration!(cache::CacheBundle, c::CMC{<:FrameAngularVelocity}) = nothing
__velocity!(cache::CacheBundle, c::CMC{<:FrameAngularVelocity}) = nothing
function __jacobian!(cache::CacheBundle, c::CMC{<:FrameAngularVelocity})
    J = J_cache_view(cache, c)
    fill!(J, zero(eltype(J))) # TODO do this once rather than every time
    J_ret = _vms_jacobian_result_view(cache, J, c.coord_data.frameID)
    Jp = get_angular_jacobian(cache, c.coord_data.frameID)
    J_ret .= Jp
    nothing
end
__acceleration!(cache::CacheBundle, c::CMC{<:FrameAngularVelocity}) = nothing

# _configuration(cache::MechanismCacheBundle, c::CMC{<:FrameAngularVelocity}) = rotor(transform(cache, c.coord_data.frameID))
_velocity(cache::CacheBundle, c::CMC{<:FrameAngularVelocity}) = get_angular_vel(cache, c.coord_data.frameID)
_jacobian(cache::CacheBundle, c::CMC{<:FrameAngularVelocity}) = J_cache_view(cache, c)
_acceleration(cache::CacheBundle, c::CMC{<:FrameAngularVelocity}) = _get_angular_acc(cache, c.coord_data.frameID)

####################################################################################################
# CoordNorm
####################################################################################################
function __configuration!(cache::CacheBundle, c::CMC{<:CoordNorm})
    zₐ   = configuration(cache, c.coord_data.coord)
    zᵢ = sqrt(zₐ' * zₐ)
    z_cache_view(cache, c) .= zᵢ
end
function __velocity!(cache::CacheBundle, c::CMC{<:CoordNorm})
    żₐ = velocity(cache, c.coord_data.coord)
    zₐ = configuration(cache, c.coord_data.coord)
    zᵢ = configuration(cache, c)[1]
    żᵢ = zₐ' * żₐ / zᵢ
    ż_cache_view(cache, c) .= żᵢ
end
function __jacobian!(cache::CacheBundle, c::CMC{<:CoordNorm})
    Jᵢ = J_cache_view(cache, c)
    fill!(Jᵢ, zero(eltype(Jᵢ)))
    Jᵢ_ret = _vms_jacobian_result_view(cache, Jᵢ, c.coord_data.coord)
    Jₐ = jacobian(cache, c.coord_data.coord)
    zₐ = configuration(cache, c.coord_data.coord)
    zᵢ = configuration(cache, c)
    @assert size(Jᵢ_ret, 1) == 1
    # This implements:
    # Jᵢ_ret .= zₐ' * Jₐ / zᵢ
    Nc = length(zₐ) # known at compile time
    for j in axes(Jᵢ_ret, 2)
        for k in 1:Nc
            Jᵢ_ret[1, j] += zₐ[k] * Jₐ[k, j] / zᵢ[1]
        end
    end
    nothing
end
function __acceleration!(cache::CacheBundle, c::CMC{<:CoordNorm})
    # Rewrite with zᵢ and zₐ
    zₐ = configuration(cache, c.coord_data.coord)
    żₐ = velocity(cache, c.coord_data.coord)
    αₐ = acceleration(cache, c.coord_data.coord)
    zᵢ = configuration(cache, c)[1]
    żᵢ = velocity(cache, c)[1]
    # Apply product rule + chain rule to velocity to get
    # αᵢ = (zₐ' * αₐ) / (zᵢ)   +  ((żₐ zᵢ + zₐ żᵢ)' * żₐ) / (zᵢ^2)
    αᵢ = (zₐ' * αₐ / zᵢ) + ((żₐ * zᵢ - zₐ * żᵢ)' * żₐ / zᵢ^2)
    α_cache_view(cache, c) .= αᵢ
end

_configuration(cache::CacheBundle, c::CMC{<:CoordNorm}) = SVector(z_cache_view(cache, c)[1])
_velocity(cache::CacheBundle, c::CMC{<:CoordNorm}) = SVector(ż_cache_view(cache, c)[1])
_jacobian(cache::CacheBundle, c::CMC{<:CoordNorm}) = J_cache_view(cache, c)
_acceleration(cache::CacheBundle, c::CMC{<:CoordNorm}) = SVector(α_cache_view(cache, c)[1])

####################################################################################################
# RotatedCoord
####################################################################################################
function __configuration!(cache::CacheBundle, cmc::CMC{<:RotatedCoord})
    c = cmc.coord_data
    T_b = get_transform(cache, c.frameID)
    R⁻¹ = inv(rotor(T_b))
    z = _configuration(cache, c.world_frame_coord)
    z_cache_view(cache, cmc) .= R⁻¹ * z
    nothing
end
function __velocity!(cache::CacheBundle, cmc::CMC{<:RotatedCoord})
    c = cmc.coord_data
    T_b = get_transform(cache, c.frameID)
    R⁻¹ = inv(rotor(T_b))
    ω_b = get_angular_vel(cache, c.frameID)
    z = _configuration(cache, c.world_frame_coord)
    ż = _velocity(cache, c.world_frame_coord) 
    Ṙ⁻¹z = R⁻¹*cross(z, ω_b)
    ret = Ṙ⁻¹z + R⁻¹*ż
    ż_cache_view(cache, cmc) .= ret
    nothing
end
function __jacobian!(cache::CacheBundle, cmc::CMC{<:RotatedCoord})
    c = cmc.coord_data
    T_b = get_transform(cache, c.frameID)
    R⁻¹ = rotation_matrix(inv(rotor(T_b)))    
    z = _configuration(cache, c.world_frame_coord)
    R⁻¹Sz = R⁻¹*skew(z)

    J_z = _jacobian(cache, c.world_frame_coord)
    J_ωb = get_angular_jacobian(cache, c.frameID)

    J = J_cache_view(cache, cmc)
    fill!(J, zero(eltype(J))) # TODO do this once rather than every time
    J_ret_1 = _vms_jacobian_result_view(cache, J, c.frameID)
    J_ret_2 = _vms_jacobian_result_view(cache, J, c.world_frame_coord)
    # return R⁻¹Sz*J_ωb + R⁻¹*J_z
    # mul!(J_ret, R⁻¹Sz, J_ωb, R⁻¹, J_z)
    @assert length(c) == 3
    for i in axes(J_ret_1, 1), j in axes(J_ret_1, 2)
        J_ij = zero(eltype(J_ret_1))
        for k in 1:3
            J_ij += R⁻¹Sz[i, k] * J_ωb[k, j]
        end
        J_ret_1[i, j] += J_ij
    end
    for i in axes(J_ret_2, 1), j in axes(J_ret_2, 2)
        J_ij = zero(eltype(J_ret_2))
        for k in 1:3
            J_ij += R⁻¹[i, k] * J_z[k, j]
        end
        J_ret_2[i, j] += J_ij
    end
    nothing
end
function __acceleration!(cache::CacheBundle, cmc::CMC{<:RotatedCoord})
    c = cmc.coord_data
    T_b = get_transform(cache, c.frameID)
    R⁻¹ = inv(rotor(T_b))
    ω_b = get_angular_vel(cache, c.frameID)
    ẇ_b = _get_angular_acc(cache, c.frameID)

    z = _configuration(cache, c.world_frame_coord)
    ż = _velocity(cache, c.world_frame_coord)
    z̈ = _acceleration(cache, c.world_frame_coord)

    t1 =       R⁻¹*cross(ω_b, cross(ω_b, z))
    t2 = -2.0*(R⁻¹*cross(ω_b, ż))
    t3 =     -(R⁻¹*cross(ẇ_b, z))
    t4 =       R⁻¹*z̈
    α_cache_view(cache, cmc) .=  t1 + t2 + t3 + t4
    nothing
end

_configuration(cache::CacheBundle, cmc::CMC{<:RotatedCoord}) = SVector{length(cmc)}(z_cache_view(cache, cmc))
_velocity(cache::CacheBundle, cmc::CMC{<:RotatedCoord}) = SVector{length(cmc)}(ż_cache_view(cache, cmc))
_jacobian(cache::CacheBundle, cmc::CMC{<:RotatedCoord}) = J_cache_view(cache, cmc)
_acceleration(cache::CacheBundle, cmc::CMC{<:RotatedCoord}) = SVector{length(cmc)}(α_cache_view(cache, cmc))

####################################################################################################
# UnrotatedCoord
####################################################################################################
function __configuration!(cache::CacheBundle, cmc::CMC{<:UnrotatedCoord})
    c = cmc.coord_data
    T_b = get_transform(cache, c.frameID)
    R = rotor(T_b)
    
    z = _configuration(cache, c.link_frame_coord)
    z_cache_view(cache, cmc) .= R * z
    nothing
end
function __velocity!(cache::CacheBundle, cmc::CMC{<:UnrotatedCoord})
    c = cmc.coord_data
    T_b = get_transform(cache, c.frameID)
    R = rotor(T_b)
    ω_b = get_angular_vel(cache, c.frameID)
    z = _configuration(cache, c.link_frame_coord)
    ż = _velocity(cache, c.link_frame_coord) 
    
    # THIS WORKS
    # Ṙ = skew(ω_b) * rotation_matrix(R)
    # ret = Ṙ*z + R*ż # Extra minus here compared to rotated coordinates

    # ret = skew(ω_b)*(R*z) + R*ż
    # ret = cross(ω_b, R*z) + R*ż
    ret = -cross(R*z, ω_b) + R*ż

    ż_cache_view(cache, cmc) .= ret
    nothing
end
function __jacobian!(cache::CacheBundle, cmc::CMC{<:UnrotatedCoord})
    c = cmc.coord_data
    T_b = get_transform(cache, c.frameID)
    R = rotation_matrix(rotor(T_b)) 
    z = _configuration(cache, c.link_frame_coord)
    SRz = skew(R*z)

    J_z = _jacobian(cache, c.link_frame_coord)
    J_ωb = get_angular_jacobian(cache, c.frameID)

    J = J_cache_view(cache, cmc)
    fill!(J, zero(eltype(J))) # TODO do this once rather than every time
    J_ret = _vms_jacobian_result_view(cache, J, c.frameID)
    # return RSz * -J_ωb + R⁻¹*J_z
    # mul!(J, RSz, -J_ωb, R⁻¹, J_z)
    @assert length(c) == 3
    for i in axes(J_ret, 1), j in axes(J_ret, 2)
        J_ij = zero(eltype(J_ret))
        for k in 1:3
            J_ij += -SRz[i, k] * J_ωb[k, j] # Extra minus here compared to rotated coordinates
            J_ij += R[i, k] * J_z[k, j]
        end
        J_ret[i, j] = J_ij
    end
    nothing
end
function __acceleration!(cache::CacheBundle, cmc::CMC{<:UnrotatedCoord})
    c = cmc.coord_data
    T_b = get_transform(cache, c.frameID)
    R = rotor(T_b)
    ω_b = get_angular_vel(cache, c.frameID)
    ẇ_b = _get_angular_acc(cache, c.frameID)

    z = _configuration(cache, c.link_frame_coord)
    ż = _velocity(cache, c.link_frame_coord)
    z̈ = _acceleration(cache, c.link_frame_coord)

    # Vel is
    # return R * -cross(z, ω_b) + R*ż
    # VPA is 
    # return R * -cross(z, ẇ_b) + R*z̈
    t1 =       cross(ω_b, cross(ω_b, R*z))
    t2 = -2.0*(cross(ω_b, R*ż)) # Extra minus here compared to rotated coordinates
    t3 =      (cross(ẇ_b, R*z)) # Extra minus here compared to rotated coordinates
    t4 =       R*z̈
    α_cache_view(cache, cmc) .=  t1 + t2 + t3 + t4
    nothing
end

_configuration(cache::CacheBundle, cmc::CMC{<:UnrotatedCoord}) = SVector{length(cmc)}(z_cache_view(cache, cmc))
_velocity(cache::CacheBundle, cmc::CMC{<:UnrotatedCoord}) = SVector{length(cmc)}(ż_cache_view(cache, cmc))
_jacobian(cache::MechanismCacheBundle, cmc::CMC{<:UnrotatedCoord}) = J_cache_view(cache, cmc)
_jacobian(cache::VirtualMechanismSystemCacheBundle, cmc::CMC{<:UnrotatedCoord}) = J_cache_view(cache, cmc)
_acceleration(cache::CacheBundle, cmc::CMC{<:UnrotatedCoord}) = SVector{length(cmc)}(α_cache_view(cache, cmc))
