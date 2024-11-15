using DifferentialEquations
using ForwardDiff
using ForwardDiff: Tag
using LinearAlgebra: dot
using VMRobotControl
using StaticArrays

################
# Compute loss
################

struct SysIDData{N}
    t::Float64
    q::SVector{N, Float64}
    q̇::SVector{N, Float64}
    q̈::SVector{N, Float64}
    τ::SVector{N, Float64}
end

function torque_loss(M, q̈, F, τ)
    e = (M*q̈ - F - τ)
    sum(abs.(e))
end
function acceleration_loss(M, q̈, F, τ)
    e = (q̈ - inv(M)*(F + τ))
    dot(e, e)
end

function predict_torques(m, data::SysIDData, lossfunc::F) where F
    (;t, q, q̇, q̈, τ) = data
    precomp = precompute(m, q, q̇, t)
    M = inertance_matrix(m, precomp)
    GF = generalized_force(m, precomp)
    lossfunc(M, q̈, GF, τ)
end

struct SysIDTag end
SysIDTag(θ::AbstractArray{T}) where {T} = typeof(Tag(SysIDTag(), T))

function get_forwardiff_closure(f, θ)
    diffcfg = ForwardDiff.GradientConfig(f, θ, ForwardDiff.Chunk{length(θ)}(), SysIDTag(θ)())
    diffresult = DiffResults.GradientResult(θ)
    function autodiff_closure(θ)
        ForwardDiff.gradient!(diffresult, f, θ, diffcfg, Val{false}()) # Tag checking disabled
        cost, gradient = DiffResults.value(diffresult), DiffResults.gradient(diffresult)
        cost, gradient
    end
end

#####################
# Data wrangling
#####################

function getSysIDDataFromODESol(sol, τ_func)
    data = Vector{SysIDData}(undef, length(sol.t))
    x = sol.u[:, 1]
    t = sol.t
    q = map(x -> x[q_idxs(m)], x)
    q̇ = map(x -> x[q̇_idxs(m)], x)
    τ = map(t, q, q̇) do t, q, q̇
        precomp = precompute(m, q, q̇, t)
        generalized_force(m, precomp)
    end
    τ = τ_func.(sol.t)
    q̈ = similar(q̇)
    q̈[1] = (q̇[2] - q̇[1])*sampling_freq
    q̈[end] = (q̇[end] - q̇[end-1])*sampling_freq
    for i = 2:(length(q̇)-1)
        q̈[i] = (q̇[i+1] - q̇[i-1])/2 * sampling_freq
    end
    data = map(t, q, q̇, q̈, τ) do t, q, q̇, q̈, τ
        SysIDData(t, q, q̇, q̈, τ)
    end
    data
end


################
# Optimizers
################

mutable struct ADAM{T}
    α::T
    β1::T
    β2::T
    ϵ::T
    m::Vector{T} # 1st moment vector
    v::Vector{T} # 2nd moment vector
    m̂::Vector{T}
    v̂::Vector{T}
    t::T
end

ADAM(α, β1, β2, ϵ) = ADAM(α, β1, β2, ϵ, zeros(0), zeros(0), zeros(0), zeros(0), 0.)
ADAM(α) = ADAM(α, 0.9, 0.999, 1e-8)
ADAM() = ADAM(1e-3)

function init!(o::ADAM{T}, N) where T
    o.m = zeros(N)
    o.v = zeros(N)
    o.m̂ = zeros(N)
    o.v̂ = zeros(N)
end

function update!(p::AbstractVector{T}, o::ADAM{T}, ∇::AbstractVector{T}) where T
    o.t = o.t + 1
    @. o.m = o.β1 * o.m + (1 - o.β1) * ∇
    @. o.v = o.β2 * o.v + (1 - o.β2) * ∇.^2
    @. o.m̂ = o.m/(1 - o.β1^o.t)
    @. o.v̂ = o.v/(1 - o.β2^o.t)
    @. p   = p - (o.α * o.m̂ ./ (sqrt.(o.v̂) .+ o.ϵ ))
end

struct GradientDescent{T}
    α::T
end

init!(o::GradientDescent{T}, N) where T = nothing
update!(p::AbstractVector{T}, o::GradientDescent{T}, ∇::AbstractVector{T}) where T = @.(p   = p - (o.α * ∇))

################
# Optimizing
################

function optimization_loop!(gradient_closure, θ::Union{Vector,MVector}, optimizer, callback, NSteps)
    init!(optimizer, length(θ))
    for _ = 1:NSteps
      C, ∇ = gradient_closure(θ)
      callback(θ, C) && break # Break if callback returns true
      update!(θ, optimizer, ∇)
    end
    θ
end


function optimize(build_mechanism, θ0, data::Vector{<:SysIDData}, lossfunc::F, optimizer, callback, NSteps) where F
    function f(θ)
        m = build_mechanism(θ)
        cost = 0.0
        for d in data
            cost += predict_torques(m, d, lossfunc)
        end
        cost/length(data) # Normalize
    end
    f_df = get_forwardiff_closure(f, θ0)
    optimization_loop!(f_df, θ0, optimizer, callback, NSteps)
end


###############
# Example 1
###############

build_pendulum(θ) = begin
    T = eltype(θ)
    mass = θ[1]^2
    damping = θ[2]^2
    stiffness = θ[3]^2
    length = 0.1

    m = Mechanism{eltype(θ)}("Pendulum")
    joint = Revolute(SVector{3, T}(0.0, 1.0, 0.0))
    add_frame!(m, "L1_frame")
    add_joint!(m, joint;parent=root_frame(m), child="L1_frame", jointID="J1")
    push!(m, PointMass(T(mass), CartesianOperationSpace("L1_frame",SVector(0.0, 0.0, -length)))) 
    push!(m, LinearDamper(damping, JointSubspace("J1"))) 
    push!(m, LinearSpring(stiffness, JointSubspace("J1") - π/2)) 
    compile(m)
end

m = build_pendulum([1.0, 0.1, 0.1])

q0, q̇0 = randn(SVector{1, Float64}), randn(SVector{1, Float64})
q0, q̇0 = SVector(π/2), SVector(0.0)
x0 = vcat(q0, q̇0)
T = 2.0
sampling_freq = 100.0
saveat = 1 / sampling_freq

τ_func(t) = SVector(sin(10.0*t))

i1, i2 = q_idxs(m), q̇_idxs(m)
N_states = length(i1) + length(i2)
function ode_dynamics(x::SVector, _, t)
    x::SVector{N_states}
    q, q̇ = x[i1], x[i2]
    u = τ_func(t)
    q̈ = dynamics(m, q, q̇, t, u)
    vcat(q̇, q̈)
end

# f = get_ode_dynamics(m)
prob = ODEProblem(ode_dynamics, x0, T)
tols = (:abstol => 1e-3, :reltol => 1e-3)
sol = solve(prob, Tsit5(); maxiters=1e4, saveat=saveat, tols...)
data = getSysIDDataFromODESol(sol, τ_func)

optimize(build_pendulum, [1.0, 1.0, 1.0], data, torque_loss, ADAM(0.01), (p, C) -> begin println("$C, $p"); false end, 1000)

###############
# Example 2
###############

franks = Dict()
function franka(T)
    if T in keys(franks)
        return deepcopy(franks[T])
    else 
        frank = VMRobotControl.RSON.parseRSON("../RSONs/rsons/franka_panda/pandaSurgical.rson", T)
        empty!(frank.visuals)
        franks[T] = frank
        return deepcopy(frank)
    end
end

build_franka(θ) = begin
    T = eltype(θ)
    stiffness = θ[1]^2
    damping = θ[2]^2
    
    mechanism = franka(eltype(θ))

    coord = FrameOrigin("instrument_EE_frame")
    push!(mechanism, LinearSpring(stiffness, coord))
    push!(mechanism, LinearDamper(damping, coord))
    compile(mechanism)
end

m = build_franka([30.0, 3.0])

q = 1e-3*ones(SVector{7, Float64})
q̇ = 1e-3*ones(SVector{7, Float64})
x0 = vcat(q, q̇)

T = 2.0
sampling_freq = 100.0
saveat = 1 / sampling_freq

τ_func(t) = SVector(sin(10.0*t), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

i1, i2 = q_idxs(m), q̇_idxs(m)
N_states = length(i1) + length(i2)
function ode_dynamics(x::SVector, _, t)
    x::SVector{N_states}
    q, q̇ = x[i1], x[i2]
    u = τ_func(t)
    q̈ = dynamics(m, q, q̇, t, u)
    vcat(q̇, q̈)
end

# f = get_ode_dynamics(m)
prob = ODEProblem(ode_dynamics, x0, T)
tols = (:abstol => 1e-3, :reltol => 1e-3)
sol = solve(prob, Tsit5(); maxiters=1e4, saveat=saveat, tols...)
data = getSysIDDataFromODESol(sol, τ_func)

optimize(build_franka, [1.0, 1.0], data, torque_loss, ADAM(0.1), (p, C) -> begin println("$C, $p"); false end, 1000)


using GLMakie
f = Figure()
ax = Axis(f[1, 1])
lines!(ax, t, getindex.(q, 1), label="Pos")
lines!(ax, t, getindex.(q̇, 1), label="Vel")
lines!(ax, t, getindex.(q̈, 1), label="Acc")
axislegend(ax)
f

coord = JointSubspace("J1")
opspaceConfiguration(precomp, m(coord))