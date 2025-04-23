module VMRobotControlMakieExt

using LinearAlgebra

using VMRobotControl
using VMRobotControl.Transforms: transform, scalar, bivector, Transform, AxisAngle
using VMRobotControl.Splines
using VMRobotControl.StaticArrays
using VMRobotControl: 
    MechanismCacheBundle,
    MechKinematicsBundle, 
    MechDynamicsBundle,
    VMSCacheBundle,
    VMSKinematicsBundle,
    VMSDynamicsBundle,
    CacheBundle
using VMRobotControl:
    AbstractCompiledIndex,
    CompiledFrameID,
    CompiledJointID,
    CompiledCoordID,
    VMSFrameID,
    VMSCoordID
using VMRobotControl: 
    rescale_mesh, 
    transform_mesh
import VMRobotControl:
    transform_plot!

# using VMRobotControl: CompiledCoordID

import VMRobotControl: 
    splineplot,
    splinepointsplot,
    robotsplineplot,
    jointsketch,
    componentsketch,
    robotsketch,
    robotvisualize,
    annotateframes,
    energytimeplot
import VMRobotControl: 
    splineplot!,
    splinepointsplot!,
    robotsplineplot!,
    jointsketch!,
    componentsketch!,
    robotsketch!,
    robotvisualize!,
    annotateframes!,
    energytimeplot!

import VMRobotControl: 
    opspaceplot, 
    opspaceplot!

import VMRobotControl: 
    transform_plot!,
    _update_cache_from_framenumber!

import VMRobotControl: 
    animate_robot, 
    animate_robot_kinematics!,
    link_cache_observable_to_time_observable,
    animate_robot_odesolution


import Makie
using Makie: lines!, mesh!, wireframe!, record, Observable, NoShading
using GeometryBasics

###############################
# Utils
###############################

function Base.convert(::Type{Makie.Quaternion}, r::Rotor{T}) where T
    Makie.Quaternion(
        bivector(r)..., scalar(r)
    )
end

function transform_plot!(mesh, tf::Transform)
    Makie.translation(mesh)[] = Makie.Vec3f(VMRobotControl.origin(tf))
    Makie.rotation(mesh)[] = convert(Makie.Quaternion, rotor(tf))
    nothing
end


###############################
# Spline plotting
###############################


Makie.@recipe(SplinePlot, spline) do scene
    Makie.Theme(
        color=:black,
        linestyle=:solid,
        linewidth=1.5
    )
end

function Makie.plot!(plot::SplinePlot{Tuple{S}}) where {S <: Spline{3}}
    (;spline) = plot

    o_spline = Makie.Observable(Makie.Point3f[])

    function update_plot(spline)
        empty!(o_spline[])
        
        N = n_knots(spline)
        t_max = N-1.0
        D = ndims(spline)
        Ns = 10 # resolution
        ts = LinRange(-1.0, t_max + 1.0, Ns*(N+2))
        for t in ts
            push!(o_spline[], Point3f(spline_position(t, spline)))
        end
        
        notify(o_spline )
    end
    Makie.on(update_plot, spline)
    update_plot(spline[])
    Makie.lines!(plot, o_spline;
        color=plot.color,
        linestyle=plot.linestyle,
        linewidth=plot.linewidth)
    plot
end


Makie.@recipe(SplinePointsPlot, spline) do scene
    Makie.Theme()
end


function Makie.plot!(plot::SplinePointsPlot{Tuple{S}}) where {S <: Spline{3}}
    (;spline) = plot

    o_idxs = Observable(Int[])
    o_knots = Observable(Point3f[])
    o_controls = Observable(Point3f[])

    function update_plot(spline)
        empty!(o_idxs[])
        empty!(o_knots[])
        empty!(o_controls[])
        
        N = length(spline)        
        for i in 1:N
            push!(o_idxs[], i)
            push!(o_knots[], knots(spline)[i, :])
            push!(o_controls[], controls(spline)[i, :])
        end  
        notify.( (o_knots, o_controls) )
    end

    Makie.Observables.onany(update_plot, spline)
    update_plot(spline[])
    scatter!(plot, o_knots; color=:blue)
    scatter!(plot, o_controls; color=:red)

    plot
end

###

Makie.@recipe(RobotSplinePlot, robot, q, t) do scene
    Makie.Theme(
        # Light comes from (0, 0, 15), i.e the sphere
        lightposition = Makie.Vec3f(15, 15, 15),
        # base light of the plot only illuminates red colors
        ambient = Makie.RGBf(0.3, 0, 0),
        camera = :cam3d!
    )
end


function Makie.plot!(plot::RobotSplinePlot{Tuple{R, Q, TIME}}) where {R <: CompiledMechanism, Q<:SVector, TIME<:Real}
    (;robot, q, t) = plot
    mjoints = robot[].rbtree.walk
    joints = [mj.joint for mj in mjoints]
    splinejoints = [j for (j, joint) in enumerate(joints) if isa(joint, VMRobotControl.Joint1DOF) && isa(joint.jointdata, RailData)]
    splinejoint_tfs = [joints[j].jointdata.transform for j in splinejoints]
    splines = [joints[j].jointdata.spline for j in splinejoints]
    splineframes = [mjoints[j].parentFrameID for j in splinejoints]
    splineplots = [VMRobotControl.Splines.splineplot!(plot, spline) for spline in splines]
    function update_plot(robot, q, t)
        transforms = VMRobotControl.kinematics(robot, q, t)
        for (frameidx, tf_joint, plot) in zip(splineframes, splinejoint_tfs, splineplots)
            tf_parent = transforms[frameidx]
            transform_plot!(plot, tf_parent*tf_joint)
        end
    end
    Makie.Observables.onany(update_plot, robot, q, t)
    update_plot(robot[], q[], t[])
    plot
end

###############################
# JointSketch
###############################

Makie.@recipe(JointSketch, cache, jointID) do scene
    Makie.Theme(
        camera = :cam3d!,
        scale=1.0,
        meshcolor=:white,
        linecolor=:black,
        linewidth=1.5,
        shading=NoShading
    )
end

function Makie.plot!(plot::JointSketch{Tuple{C, J}}) where {C<:MechanismCacheBundle, J}
    (; cache, jointID) = plot
    @assert J <: VMRobotControl.CompiledJointID
    joint = cache[].mechanism[jointID[]]
    jointdata = joint.jointData
    parent_frame::CompiledFrameID = joint.parentFrameID
    
    # Sketch custom geometry for each joint type
    plot_objs = _sketchjoint!(plot, jointdata, cache; 
                                scale=plot.scale[],
                                meshcolor=plot.meshcolor,
                                linecolor=plot.linecolor,
                                linewidth=plot.linewidth,
                                shading=plot.shading[])

    # Set so that geometry transforms with the frame of the joint
    update_from_cache(cache) = begin
        tf = get_transform(cache, parent_frame)
        foreach(plot_objs) do po
            transform_plot!(po, tf)
        end
    end
    Makie.Observables.onany(update_from_cache, cache)
    update_from_cache(cache[])

    if !(isa(jointdata, RailData))
        # Also draw a line between parent/child frame
        points = map(cache) do cache # Map Observable to Observable{Vector{SVector{3, T}}}
            [VMRobotControl.origin(get_transform(cache, joint.parentFrameID)), 
            VMRobotControl.origin(get_transform(cache, joint.childFrameID))]
        end
        pl = lines!(plot, points; color=plot.linecolor, plot.linewidth) # Don't return, as we don't want to transform it
    end

    plot
end


function _sketchjoint!(ax, joint::Rigid, cache::Observable; 
                       scale, meshcolor, linecolor, linewidth, shading)
    pl = lines!(ax, [SVector(0., 0., 0.), joint.transform * SVector(0., 0., 0.)]; color=linecolor, linewidth)
    (pl,)
end

function _sketchjoint!(ax, joint::RevoluteData, cache::Observable; 
                       scale, meshcolor, linecolor, linewidth, shading)
    origin = joint.transform * (joint.axis * 0.1 * scale)
    extremity = joint.transform * (joint.axis * -0.1 * scale)
    cyl = Cylinder(Point3f(origin), Point3f(extremity), Float32(0.1 * scale))
    pm = mesh!(ax, normal_mesh(cyl); color=meshcolor, shading)
    pw = wireframe!(ax, cyl; color=linecolor, linewidth, fxaa=true)
    # pl = lines!(ax, [SVector(0., 0., 0.), joint.transform * SVector(0., 0., 0.)]; color=linecolor, linewidth)
    (pm, pw,)
end

function _sketchjoint!(ax, joint::PrismaticData, cache::Observable; 
                       scale, meshcolor, linecolor, linewidth, shading)
    ax_cross_Z = -cross(normalize(joint.axis), normalize(SVector(0., 0., 1.)))
    θ, rotation_axis = asin(norm(ax_cross_Z)), normalize(ax_cross_Z)
    if θ ≈ 0
        r = zero(Rotor{Float64})
    else
        r = AxisAngle(rotation_axis, θ)
    end
    tf = joint.transform * Transform(r)
    rect_mesh = rescale_mesh(normal_mesh(Rect(-0.06, -0.06, -0.1, 0.12, 0.12, 0.2)), scale)
    rect_mesh = transform_mesh(rect_mesh, tf)
    pm = mesh!(ax, rect_mesh; color=meshcolor, shading)
    pw = wireframe!(ax, rect_mesh; color=linecolor, linewidth, fxaa=true)
    (pm, pw,)
end

function _sketchjoint!(ax, joint::RailData, cache::Observable;
                       scale, meshcolor, linecolor, linewidth, shading)
    tf = joint.transform
    # TODO transform spline
    spline = deepcopy(joint.spline)
    for i = 1:size(spline.data.knots, 1)
        spline.data.knots[i, :] = tf * SVector{3}(spline.data.knots[i, :])
    end
    for i = 1:size(spline.data.controls, 1)
        spline.data.controls[i, :] = tf * SVector{3}(spline.data.controls[i, :])
    end
    s = VMRobotControl.splineplot!(ax, spline; color=linecolor, linewidth)
    (s,)
end

function _sketchjoint!(ax, joint::ReferenceJoint, cache::Observable; 
                       scale, meshcolor, linecolor, linewidth, shading)
    # TODO implement    
    ()
end


################################################################################
# Component Sketch
################################################################################

Makie.@recipe(ComponentSketch, cache, componentID) do scene
    Makie.Theme(
        camera = :cam3d!,
        scale=1.0,
        meshcolor=:white,
        linecolor=:black,
        linewidth=1.5,
        shading=NoShading
    )
end

function Makie.plot!(plot::ComponentSketch{Tuple{C, J}}) where {C<:CacheBundle, J}
    (; cache, componentID) = plot
    if cache[] isa MechanismCacheBundle
        component = cache[].mechanism[componentID[]]
    elseif cache[] isa VMSCacheBundle
        component = cache[].vms[componentID[]]
    else
        error("Unsupported cache type: $(typeof(cache))")
    end

    if ~(typeof(component) <: Union{LinearSpring, TanhSpring})
        error("Unsupported component type: $(typeof(component))")
    end
    coord = component.coord
    if ~(VMRobotControl.coord_type(coord) <: CoordDifference)
        error("Unsupported coordinate type: $(typeof(coord))")
    end
    plot_objs = _sketchcomponent!(plot, component; 
                                scale=plot.scale[],
                                meshcolor=plot.meshcolor,
                                linecolor=plot.linecolor,
                                linewidth=plot.linewidth,
                                shading=plot.shading[])
    
    update_from_cache(cache) = begin
        points = []
        _get_points_from_id!(points, cache, coord)
        tip, base = points

        axial = tip - base
        u = normalize(cross(normalize(axial), SVector(0., 0., 1.)))
        v = normalize(cross(normalize(axial), u))
        R = SMatrix{3, 3}(u..., v..., axial...)
        o = base
        tf = @SMatrix([R[1, 1] R[1, 2] R[1, 3] o[1];
                       R[2, 1] R[2, 2] R[2, 3] o[2];
                       R[3, 1] R[3, 2] R[3, 3] o[3];
                       0.      0.      0.      1.])
        foreach(po -> Makie.transformationmatrix(po)[] = tf, plot_objs)
    end
    Makie.Observables.onany(update_from_cache, cache)
    update_from_cache(cache[])

    plot
end


function _sketchcomponent!(ax, component::LinearSpring; scale, meshcolor, linecolor, linewidth, shading)
    rotations = 5
    spiral = [
        begin
            r = scale
            if t < 0.1
                r = r * (t / 0.1)
            elseif t > 0.9
                r = r * ((1 - t) / 0.1)
            end
            x = r * cos(2π * rotations * t)
            y = r * sin(2π * rotations * t)
            z = t
            Point3f(x, y, z)
        end
        for t in LinRange(0, 1, 100)
    ]
    pl = lines!(ax, spiral; color=linecolor, linewidth)
    (pl,)
end


################################################################################
# Robot Sketch
################################################################################

Makie.@recipe(RobotSketch, cache) do scene
    Makie.Theme(
        camera = :cam3d!,
        scale=1.0,
        meshcolor=:white,
        linecolor=:black,
        linewidth=1.5,
        shading=NoShading
    )
end

# Plot robot with no visuals
function Makie.plot!(plot::RobotSketch{Tuple{C}}) where C<:MechanismCacheBundle
    (; cache) = plot

    jointIDs = [jID for (_, jID) in cache[].mechanism.rbtree.joint_id_map]

    foreach(jointIDs) do jointID
        jointsketch!(plot, cache, jointID; 
                        scale=plot.scale[],
                        meshcolor=plot.meshcolor,
                        linecolor=plot.linecolor,
                        linewidth=plot.linewidth,
                        shading=plot.shading[])
    end
    if isempty(jointIDs)
        Makie.scatter!(plot, Point3f[]) # This is to make the plot work when no joints are present
    end
    plot
end

function Makie.plot!(plot::RobotSketch{Tuple{C}}) where C<:VMSCacheBundle
    (; cache) = plot
    cache1 = Makie.@lift VMRobotControl.robot_cache($cache)
    cache2 = Makie.@lift VMRobotControl.virtual_mechanism_cache($cache)
    (; scale, meshcolor, linecolor, linewidth, shading) = plot
    robotsketch!(plot, cache1; scale, meshcolor, linecolor, linewidth, shading)
    robotsketch!(plot, cache2; scale, meshcolor, linecolor, linewidth, shading)
    plot
end


################################
# Annotate robot frame names
################################
Makie.@recipe(AnnotateFrames, cache) do scene
    Makie.Theme(
        camera = :cam3d!,
        color = :black,
        align = (:left, :baseline),
        font = :regular,
        rotation = 0.,
        fontsize = 12,
        strokecolor = :black,
        glowwidth = 0.0,
        glowcolor = (:black, 0),
        word_wrap_width = -1,
        overdraw = false,
        depth_shift = 0f0,
        offset=Vec3f0(0.0, 0.0, 0.0)
    )
end

function Makie.plot!(plot::AnnotateFrames{Tuple{C}}) where C<:MechanismCacheBundle
    (; cache) = plot
    frame_names = frames(cache[].mechanism)
    frame_idxs = get_compiled_frameID.((cache[].mechanism, ), frame_names)
    Makie.text!(plot, cache, frame_idxs; text=frame_names, 
        plot.color,
        plot.align,
        plot.font,
        plot.rotation,
        plot.fontsize,
        plot.strokecolor,
        plot.glowwidth,
        plot.glowcolor,
        plot.word_wrap_width,
        plot.overdraw,
        plot.depth_shift,
        plot.offset)
end

function Makie.plot!(plot::AnnotateFrames{Tuple{C}}) where C<:VMSCacheBundle
    (; cache) = plot
    cache1 = Makie.@lift VMRobotControl.robot_cache($cache)
    cache2 = Makie.@lift VMRobotControl.virtual_mechanism_cache($cache)
    annotateframes!(plot, cache1)
    annotateframes!(plot, cache2)
    plot
end

################################

Makie.@recipe(RobotVisualize, cache) do scene
    Makie.Theme(
        shading = Makie.MultiLightShading,
        camera = :cam3d!,
        transparency = false
    )
end

function Makie.plot!(plot::RobotVisualize{Tuple{C}}) where C<:MechanismCacheBundle
    (; cache) = plot
    vis = visuals(cache[].mechanism); 
    # isempty(vis) && error("No visuals to plot")

    meshes = []
    frames = CompiledFrameID[]

    foreach(vis) do v
        # geom = normal_mesh(v.geometry)
        geom = v.geometry
        if isa(geom, Mesh)
            msh = Makie.mesh!(plot, geom; 
                shading=plot.shading,
                color=v.color, 
                specular=v.specular,
                shininess=v.shininess,
                transparency=plot.transparency
            )
            push!(meshes, msh)
            push!(frames, v.frame)
        else
            error("Unsupported geometry type: $(typeof(geom))")
        end       
    end

    function update_from_cache(cache)
        for (mesh, frame) in zip(meshes, frames)
            tf = get_transform(cache, frame)
            transform_plot!(mesh, tf)
        end
    end

    Makie.Observables.onany(cache) do cache
        update_from_cache(cache)
    end
    
    update_from_cache(cache[])

    if isempty(vis)
        Makie.scatter!(plot, zero(Point3f), color=:black) # This is to make the plot work when no visuals are present
    end
    plot
end

function Makie.plot!(plot::RobotVisualize{Tuple{C}}) where C<:VMSCacheBundle
    (; cache) = plot
    cache1 = Makie.@lift VMRobotControl.robot_cache($cache)
    cache2 = Makie.@lift VMRobotControl.virtual_mechanism_cache($cache)
    robotvisualize!(plot, cache1; transparency=plot.transparency)
    robotvisualize!(plot, cache2; transparency=plot.transparency)
    plot
end


################################
# Plotting transforms
################################

Makie.plottype(::Transform) = Makie.LineSegments
function Makie.convert_arguments(S::Type{<:Makie.LineSegments}, tf::Transform) 
    o = VMRobotControl.origin(tf)
    x = tf * SVector(1, 0, 0)
    y = tf * SVector(0, 1, 0)
    z = tf * SVector(0, 0, 1)
    segments = [o, x, o, y, o, z]
    return Makie.SpecApi.LineSegments(segments)
end

################################
# Plotting frames from compiled id
################################

function _get_point_from_id(cache::CacheBundle, id::Union{CompiledFrameID, VMSFrameID})
    VMRobotControl.origin(get_transform(cache, id))
end

function _get_points_from_id!(points, cache::CacheBundle, id::Union{CompiledFrameID, VMSFrameID})
    push!(points, _get_point_from_id(cache, id))
end

###############################
# Plotting coords from compiled ID
###############################

function _get_point_from_id(cache::CacheBundle, id::Union{CompiledCoordID, VMSCoordID})
    coord = VMRobotControl.coordinate(cache, id).coord_data
    if coord isa Union{FramePoint, FrameOrigin}
        VMRobotControl._configuration(cache, id)
    elseif typeof(coord) isa ConstCoord && length(coord) == 3
        VMRobotControl._configuration(cache, id)
    elseif length(coord) == 3
        VMRobotControl._configuration(cache, id)
    else
        error("Unsupported coordinate type: $(typeof(coord))")
    end

end

function _get_points_from_id!(points, cache::CacheBundle, id::Union{CompiledCoordID, VMSCoordID})
    coord = VMRobotControl.coordinate(cache, id).coord_data
    if coord isa Union{FramePoint, FrameOrigin}
        push!(points, _get_point_from_id(cache, id))
    elseif typeof(coord) <: CoordDifference && length(coord) == 3
        push!(points, _get_point_from_id(cache, coord.parent))
        push!(points, _get_point_from_id(cache, coord.child))
    elseif length(coord) == 3
        push!(points, _get_point_from_id(cache, id))
    else
        error("Unsupported coordinate type: $(typeof(coord))")
    end
end

###############################
# Convert ID to PointBased
###############################

function Makie.convert_arguments(::Makie.PointBased, cache::CacheBundle, id::AbstractCompiledIndex)
    (_get_points_from_id!(Makie.Point{3, Float32}[], cache, id), )
end

###############################
# Convert vector of IDs to PointBased
###############################

function Makie.convert_arguments(::Makie.PointBased, cache::CacheBundle, ids::Vector{<:AbstractCompiledIndex})
    ret = Vector{Makie.Point{3, Float32}}()
    for id in ids
        _get_points_from_id!(ret, cache, id)
    end
    (ret,)
end

###############################
# Convert CoordDifference to lines
###############################

const PlotCoordID{COORDDATA} = Union{CompiledCoordID{COORDDATA}, VMSCoordID{COORDDATA}}

function Makie.convert_arguments(::Type{Makie.Text}, cache::CacheBundle, id::PlotCoordID{<:CoordDifference})
    coord = VMRobotControl.coordinate(cache, id).coord_data
    @assert length(coord) == 3
    p1 = _get_point_from_id(cache, coord.parent)
    p2 = _get_point_from_id(cache, coord.child)
    p = Makie.Point{3, Float32}((p1 + p2)/2)
    ([p],)
end

# function graphplot(m::Mechanism)
#     graphplot(graph(m), nlabels=frames(m), elabels=string.(typeof.(joints(m))))
# end


##########################
# Animation
##########################

function fastforward_heuristic(sol, fastforward)
    Tmax = maximum(sol.t)
    if Tmax > 15.0 
        ff = Tmax/15.0
        @info "Fastforwarding animation at $ff× speed, so that it plays in 15 seconds (was $Tmax seconds)."
        return ff
    end
    return 1.0 # Play at normal speed
end

function link_cache_observable_to_time_observable(
        cache::Observable{CB},
        t::Observable{<:Real},
        sol,
        f_control,
        args,
        unpack_ode_state=default_dissassemble_ode_state
    ) where CB<:CacheBundle
    Makie.on(t) do t
        @assert t >= sol.t[1] "Time $t is before start of simulation $(sol.t[1])"
        @assert t <= sol.t[end] "Time $t is after end of simulation $(sol.t[end])"
        x = sol(t)
        q, q̇ = unpack_ode_state(cache[], x)
        if CB<:Union{MechKinematicsBundle, VMSKinematicsBundle}            
            kinematics!(cache[], t, q)
        elseif CB<:Union{MechDynamicsBundle, VMSDynamicsBundle}
            dynamics!(cache[], t, q, q̇, cache[].cache.gravity[]) # WARNING make sure gravity is initialized
        else    
            error("Unsupported cache type: $(typeof(cache))")
        end
        f_control(cache[], t, args, nothing)
        notify(cache)
    end
end

function default_dissassemble_ode_state(cache::CacheBundle, ode_state::AbstractVector)
    # NOTE this allocates...
    iq, iq̇ = state_idxs(cache)
    ode_state[iq], ode_state[iq̇]
end


function animate_robot_odesolution(
        fig::Makie.Figure, 
        sol, 
        cache::Observable{<:Union{CacheBundle}},
        saveas;
        t=Observable{Float64}(0.),
        fps=60,
        fastforward=nothing,
        unpack_ode_state=default_dissassemble_ode_state,
        f_setup=VMRobotControl.DEFAULT_F_SETUP,
        f_control=VMRobotControl.DEFAULT_F_CONTROL
    )
    # Determine fastforward speed
    ff::Float64 = isnothing(fastforward) ? (fastforward = fastforward_heuristic(sol, fastforward)) : fastforward    
    # Determine vector of frame times
    Tmax = maximum(sol.t)
    N_frames = Int(floor(fps*Tmax/ff))
    ts = LinRange(0.0, Tmax, N_frames)
    # Run f_setup callback, and record (which will call f_control callback)
    args = f_setup(cache[])
    # Set up observable so that cache is updated from t
    link_cache_observable_to_time_observable(cache, t, sol, f_control, args, unpack_ode_state)
    record(fig, saveas, ts, framerate=fps; visible=true) do t_i
        t[] = t_i
    end
end

"""
    animate_robot_kinematics!(ax, m; sketch=true, visualize=true, annotate=true)

Animates the robot joints moving between zero and one.

"""
function animate_robot_kinematics!(ax, m; sketch=true, visualize=true, annotate=true)
    t = Observable(0.0)
    q = Observable(zero_q(m))
    
    cache_observable = map(t, q) do t, q 
        kcache = new_kinematics_cache(m)
        kinematics!(kcache, 0.0, q)
        kcache
    end

    plot_objs = []

    sketch && push!(plot_objs, robotsketch!(ax, cache_observable))
    visualize && push!(plot_objs, robotvisualize!(ax, cache_observable))
    annotate && push!(plot_objs, annotateframes!(ax, cache_observable))

    function update_animation()
        period = 2.0
        t = abs( (time()%period) - (period/2))
        if isa(q[], Tuple)
            # Special handling for virtual mechanism system
            q[] = (ones(ndof(m.robot)) * t, ones(ndof(m.virtual_mechanism)) * t)
        else
            q[] = ones(ndof(m)) * t
        end
    end

    return plot_objs, update_animation
end


##########################
# Energy/time plot
##########################

Makie.@recipe(EnergyTimePlot, robot, precomp) do scene
    Makie.Theme()
end

# Plot robot with no visuals
function Makie.plot!(plot::EnergyTimePlot{Tuple{R, P}}) where {R <: CompiledMechanism, P<:MechanismCacheBundle}
    (;robot, precomp) = plot
    point = Makie.Observable( (0.0, 0.0) )
    function update_plot(robot, precomp)
        point[] = (precomp.t, stored_energy(robot, precomp))
    end
    Makie.Observables.onany(update_plot, robot, precomp)
    update_plot(robot[], precomp[])
    Makie.scatter!(plot, point)
    plot
end

end