using Enzyme
using LinearAlgebra
using VMRobotControl
using StaticArrays

begin # Example using a pendulum
    function compute_squared_error(cache, θ, qs, measurements, jID, cID)
        # Update the joint axis, based on the optimization parameters
        axis = normalize(SVector(θ[1], θ[2], θ[3]))
        cache.mechanism[jID] = VMRobotControl.remake(cache.mechanism[jID]; jointData=Revolute(axis))

        # Sum up the errors
        cost = 0.0
        for (q, p_meas) in zip(qs, measurements)
            kinematics!(cache, 0.0, q)
            p_comp = configuration(cache, cID)
            # @show p_comp, p_meas
            err = p_comp - p_meas
            cost += err' * err
        end
        # Regularization term as θ is normalized, so could shrink to zero or grow to infinity
        cost += 1e-6 * (abs(θ[1]) + abs(θ[2]) + abs(θ[3]) - 1) 

        return cost
    end


    function optimize_axis(m, measurements, qs, cID, jID, axis_init)
        cache = new_kinematics_cache(m)
        
        θ = [axis_init[1], axis_init[2], axis_init[3]]
        @show compute_squared_error(cache, θ, qs, measurements, jID, cID)

        dup_cache = Duplicated(cache, deepcopy(cache))
        dθ = deepcopy(θ)
        dup_theta = Duplicated(θ, dθ)
        const_qs = Const(qs)
        const_measurements = Const(measurements)
        const_jID = Const(jID)
        const_cID = Const(cID)

        for i = 1:50
            fill!(dθ, 0.0)
            t = @elapsed autodiff(Reverse, compute_squared_error, dup_cache, dup_theta, const_qs, const_measurements, const_jID, const_cID)[1][2]
            for i in eachindex(θ)
                θ[i] -= 1e-1 * dθ[i]
            end
            C = compute_squared_error(cache, θ, qs, measurements, jID, cID)
            axis = normalize(SVector(θ[1], θ[2], θ[3])) 
            println("Iteration $i: \tCost: $(round(C;sigdigits=3)), \tAxis: $(round.(axis; sigdigits=3))\t Time=$(round(t*1e6;digits=1))μs")
        end
        axis = normalize(SVector(θ[1], θ[2], θ[3])) 
        axis, θ
    end


    a1 = SVector(0., 0., 1.)
    a2 = normalize(SVector(0., 0.2, 0.8))

    # Build mechanism with `incorrect` axis  to generate measurements
    mechanism = Mechanism{Float64}("Test")
    add_frame!(mechanism, "L1")
    J1 = Revolute(a2)
    add_joint!(mechanism, J1;parent="root_frame", child="L1", jointID="J1")
    add_coordinate!(mechanism, FramePoint("L1", SVector(1., 0., 0.)); coordID="EE")
    m = compile(mechanism)

    # Get coordinate index to measure
    cID = get_compiled_coordID(m, "EE")

    # Generate measurements
    qs = [[i] for i in LinRange(0, 2π, 10)]
    cache = new_kinematics_cache(m)
    measurements = [
        begin
            kinematics!(cache, 0.0, q)
            configuration(cache, cID)
        end
        for q in qs
    ]

    # Get joint index to modify the axis in training
    jID = get_compiled_jointID(m, "J1")
    axis, θ = optimize_axis(m, measurements, qs, cID, jID, a1)
    println("Final axis: $(round.(axis; sigdigits=3)), True axis: $(round.(a2;sigdigits=3))")
end



begin # Example using a franka arm
    function compute_squared_error(cache, θ, qs, measurements, jointIDs, measurement_coord_ID)
        # Update the joint axis, based on the optimization parameters
        i = 1
        for jID in jointIDs
            joint = cache.mechanism[jID]
            axis = normalize(SVector(θ[i], θ[i+1], θ[i+2]))
            origin = SVector(θ[i+3], θ[i+4], θ[i+5])
            transform = Transform(origin, rotor(joint.jointData.transform))
            i+=6
            new_jointdata = Revolute(axis, transform)
            # @show new_jointdata, joint.jointData
            cache.mechanism[jID] = VMRobotControl.remake(joint; jointData=new_jointdata)
        end
        

        # Sum up the errors
        cost = 0.0
        for (q, p_meas) in zip(qs, measurements)
            kinematics!(cache, 0.0, q)
            p_comp = configuration(cache, measurement_coord_ID)
            err = p_comp - p_meas
            cost += err' * err
        end
        # Regularization term as θ is normalized, so could shrink to zero or grow to infinity
        cost += 1e-20 * sqrt(θ'*θ)

        return cost
    end


    function optimize(m, measurements, qs, cID, jointIDs, initial_θ; NSteps=50, α=1e-1, verbose=true)
        cache = new_kinematics_cache(m)
        θ = deepcopy(initial_θ)
        @show compute_squared_error(cache, θ, qs, measurements, jointIDs, cID)

        dup_cache = Duplicated(cache, deepcopy(cache))
        dθ = deepcopy(θ)
        dup_theta = Duplicated(θ, dθ)
        const_qs = Const(qs)
        const_measurements = Const(measurements)
        const_jointIDs = Const(jointIDs)
        const_cID = Const(cID)

        for step = 1:NSteps
            C = compute_squared_error(cache, θ, qs, measurements, jointIDs, cID)
            rms_err = 1e3*sqrt(C/length(measurements))
            fill!(dθ, 0.0) # Reset gradient info 
            t = @elapsed autodiff(Reverse, compute_squared_error, dup_cache, dup_theta, const_qs, const_measurements, const_jointIDs, const_cID)[1][2]
            for i in eachindex(θ) # Update parameters
                θ[i] -= α * dθ[i]
            end
            if verbose && (mod(step, NSteps/50) == 0 || step == 1 || step == NSteps)
                println("Iteration $step: \tCost: $(round(C;sigdigits=3)), \tRMS: $(round(rms_err;sigdigits=3))mm, \t Time=$(round(t*1e6;digits=1))μs")
            end 
        end
        θ
    end

    mechanism = parseRSON("./RSONs/rsons/franka_panda/panda_kinematics.rson")
    add_coordinate!(mechanism, FramePoint("EE_frame", SVector(0., 0., 0.)); coordID="EE_measurement")
    m = compile(mechanism)

    # Get coordinate index to measure
    cID = get_compiled_coordID(m, "EE_measurement")

    # Generate measurements
    qs = [100 .* randn(7) for i in 1:3]
    cache = new_kinematics_cache(m)
    measurements = [
        begin
            kinematics!(cache, 0.0, q)
            configuration(cache, cID)
        end
        for q in qs
    ]

    # Get joint index to modify the axis in training
    jointIDs = [
        get_compiled_jointID(m, "J1"),
        get_compiled_jointID(m, "J2"),
        get_compiled_jointID(m, "J3"),
        get_compiled_jointID(m, "J4"),
        get_compiled_jointID(m, "J5"),
        get_compiled_jointID(m, "J6"),
        get_compiled_jointID(m, "J7")
    ]
    true_θ = Float64[]
    for jID in jointIDs
        joint = cache.mechanism[jID].jointData
        append!(true_θ, joint.axis)
        append!(true_θ, joint.transform.origin)
    end

    initial_θ = true_θ + 0.15 * randn(length(true_θ))
    final_θ = optimize(m, measurements, qs, cID, jointIDs, initial_θ;NSteps=10000, α=1e-3)
    println("Initial, final and true parameters:")
    display(round.(hcat(initial_θ, final_θ, true_θ); digits=4))
end