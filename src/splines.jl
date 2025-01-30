module Splines

export Spline, CubicSpline, LoopingCubicSpline
export spline_position, spline_derivative, spline_second_derivative
export n_knots

using LinearAlgebra
using Random
using SparseArrays
using StaticArrays

import Base: length, ndims, display

abstract type Spline{D, T} end

struct CubicBezierCurve{D, T}
    p₁::SVector{D, T}
    p₂::SVector{D, T}
    p₃::SVector{D, T}
    p₄::SVector{D, T}
end

CubicBezierCurve(p₁::T, p₂::T, p₃::T, p₄::T) where T<:Real = CubicBezierCurve(SVector.((p₁, p₂, p₃, p₄))...)

""" 
    curve_position(t, ::CubicBezierCurve)

r(t) is a cubic bezier curve passing through p₁ and p₄ at t=0.0 and 1.0 respectively, with control points 
p₂ and p₃. It is twice differentiable.

# Examples
```julia-repl
julia> c = CubicBezierCurve(2.1, 2.3, 2.7, 2.9)
julia> curve_position(0.0, c)
1-element SVector{1, Float64} with indices SOneTo(1):
 2.1
```
"""
function curve_position(t, c::CubicBezierCurve)
    p₁, p₂, p₃, p₄ = c.p₁, c.p₂, c.p₃, c.p₄
    (1-t)^3*p₁ + 3(1-t)^2 *t*p₂ + 3(1-t)t^2 * p₃ + t^3 *p₄
end

"""
    curve_derivative(t, ::CubicBezierCurve)

First derivative of a cubic bezier curve at t. 
"""
function curve_derivative(t, c::CubicBezierCurve)
    p₁, p₂, p₃, p₄ = c.p₁, c.p₂, c.p₃, c.p₄
    3*(p₂ - p₁)*(1-t)^2 + 6*(p₃-p₂)*(1-t)*t + 3*(p₄-p₃)*t^2 
end

"""
    curve_second_derivative(t, ::CubicBezierCurve)

Second derivative of a cubic bezier curve at t.
"""
function curve_second_derivative(t, c::CubicBezierCurve)
    p₁, p₂, p₃, p₄ = c.p₁, c.p₂, c.p₃, c.p₄
    6*(p₁ - 2*p₂ + p₃)*(1-t) + 6(p₄ - 2*p₃ + p₂)*t
end

function cubic_bezier_curve_length(c::CubicBezierCurve; N=10)
    integrate_curve_between(c, 0.0, 1.0; N=N)
end

"""
    integrate_curve_between(c::CubicBezierCurve, from, until, N=10)

Numerically integrate the length of a bezier curve r(t) between
`t=from` and `t=until`. The curve position is evaluated `N+2` times.
"""

function integrate_curve_between(c::CubicBezierCurve, from, until; N=10)
    # If N=0, just does diference between start and end
    dt = (until-from)/(N+1)
    ts = LinRange(from+dt, until-dt, N)

    L = 0
    local p1, p2
    p1 = curve_position(from, c)
    for t in ts
        p2 = curve_position(t, c)
        L += norm(p2-p1)
        p1 = p2
    end 
    p2 = curve_position(until, c)
    L += norm(p2 - p1)
    L
end

function cubic_spline_interpolate_basic(knots)
    controls = _cubic_spline_interpolate_basic(knots)
    CubicSpline(CubicSplineData(knots, controls))
end

function _cubic_spline_interpolate_basic(knots)
    """
    Given a set of points ('knots'), find the nescesary control points to 
    interpolate between the knots with a cubic bezier spline

    Zero acceleration at either end, but non-zero velocity
    """
    N = size(knots, 1)
    D = size(knots, 2)

    controls = 0*knots
    controls[1, :] = knots[1, :]
    controls[end, :] = knots[end, :]

    if N > 2
        # Solve for the remaining
        A = Tridiagonal(ones(N-3), 4*ones(N-2), ones(N-3))
        b = 6.0 .*knots[2:N-1, :]
        b[1, :] -= knots[1, :]
        b[end, :] -= knots[end, :]
        for i = 1:D
            controls[2:end-1, i] = A\b[:, i]
        end
    end
    controls
end

function cubic_spline_interpolate_zero_terminated(knots)
    """
    Given a set of points ('knots'), return a cubic bezier spline that passes 
    through each points and terminates with zero acceleration and zero velocity
    at either end.
    """
    N = size(knots, 1)
    D = size(knots, 2)

    controls = 0*knots

    # Solve controls to be continous at all t except 0 or N
    A = Tridiagonal(ones(N-1), 4*ones(N), ones(N-1))
    A[1, 1] = 5
    A[end, end] = 5
    b = 6.0 .*knots
    for i = 1:D
        controls[:, i] = A\b[:, i]            
    end
    
    # Add first/last knots/controls to terminate continously and at 0
    S₀ = controls[1, :]
    Sₙ = controls[N, :]
    CubicSpline(CubicSplineData(
        vcat(S₀', knots, Sₙ'),
        vcat(S₀', controls, Sₙ')
    ))
end

function cubic_spline_interpolate_looping(knots)
    """
    """
    N = size(knots, 1)
    D = size(knots, 2)

    controls = 0*knots

    # Solve controls to be continous at all t except 0 or N
    A = sparse(Tridiagonal(ones(N-1), 4*ones(N), ones(N-1)))
    A[1, end] = 1
    A[end, 1] = 1
    b = 6.0 .*knots
    for i = 1:D
        controls[:, i] = A\b[:, i]            
    end
    
    # Add first/last knots/controls to terminate continuously and at 0
    LoopingCubicSpline(CubicSplineData(knots, controls))
end

# function compute_lengths(knots, controls)
#     lengths = Vector{eltype(knots)}(undef, size(knots, 1))
#     L = 0
#     lengths[1] = L
#     for i = 2:length(lengths)
#         knots[i, :]
#         controls[i, :]
#         L += bezier_curve_length(
#             knots[i-1, :], 
#             (2*controls[i-1, :] +   controls[i, :])/3, 
#             (  controls[i-1, :] + 2*controls[i, :])/3, 
#             knots[i, :]
#         )
#         lengths[i] = L
#     end
#     return lengths
# end

function integrate_length(::Val{D}, knots::AbstractMatrix{T}, controls::AbstractMatrix{T}) where {D,T}
    L = 0
    for i = 1:size(knots, 1)-1
        allD = SVector{D}(NTuple{D}(j for j in 1:D))
        curve = CubicBezierCurve{D, T}(
            knots[i, allD],
            (2*controls[i, allD] +   controls[i+1, allD])/3,
            (  controls[i, allD] + 2*controls[i+1, allD])/3,
            knots[i+1, allD]
        )
        L += cubic_bezier_curve_length(curve)
    end
    L
end

function integrate_spline_between(s::Spline{D, T}, from, until; N=10) where {D,T}
    from ≈ 0.0 && (from += eps(from)) # To avoid error in get curve
    until ≈ 0.0 && (until += eps(until)) # To avoid error in get curve
    c_from, i_from, tc_from = get_curve_and_t(from, s)
    c_until, i_until, tc_until = get_curve_and_t(until, s)
    if i_from == i_until
        return integrate_curve_between(c_from, tc_from, tc_until)
    else
        L = 0.0
        L += integrate_curve_between(c_from, tc_from, 1.0)
        i = i_from + 1
        while i < i_until
            curve = get_curve(s, i)
            L += cubic_bezier_curve_length(curve)
            i += 1
        end
        L += integrate_curve_between(c_until, 0.0, tc_until)
    end
    L
end



# function position(spline, l)
#     # find where l is in the lengths Vector
#     i = binary_search(spline.lengths, l)
#     t = (l - lengths[i])/(lengths[i+1]
# end

struct CubicSplineData{D, T}
    knots::Matrix{T}
    controls::Matrix{T}
    function CubicSplineData(knots::Matrix, controls::Matrix{T}) where T
        @assert size(knots, 2) == size(controls, 2)
        D = size(knots, 2)
        new{D, T}(knots, controls)
    end
end

CubicSplineData(knots::AbstractVector, controls::AbstractVector) = 
    CubicSplineData(reshape(knots, :, 1), reshape(controls, :, 1))

struct CubicSpline{D, T} <: Spline{D, T}
    data::CubicSplineData{D, T}
end

struct LoopingCubicSpline{D, T} <: Spline{D, T}
    data::CubicSplineData{D, T}
end

knots(s::Spline) = s.data.knots
controls(s::Spline) = s.data.controls

n_knots(s::Spline) = size(knots(s), 1)
# length(s::Spline) = size(knots(s), 1)
ndims(s::Spline{D}) where D = D

function CubicSpline(knots)
    cubic_spline_interpolate_basic(knots)
    # cubic_spline_interpolate_zero_terminated(knots)
    # cubic_spline_interpolate_looping(knots)
end

function LoopingCubicSpline(knots)
    cubic_spline_interpolate_looping(knots)
end

function Random.rand(rng::AbstractRNG, ::Random.SamplerType{CubicSpline{D, T}}) where {D, T}
    CubicSpline(rand(rng, T, 10, D))
end
function Random.rand(rng::AbstractRNG, ::Random.SamplerType{LoopingCubicSpline{D, T}}) where {D, T}
    LoopingCubicSpline(rand(rng, T, 10, D))
end

function get_curve(s::Spline{D, T}, i::Int) where {D, T}
    N = n_knots(s)
    @assert ((i ≥ 1) && (i ≤ N)) "i = $i is outside of range [1, $N]"
    i₊₀ = mod(i-1, N) + 1 # can safely index i
    i₊₁ = mod(i, N) + 1
    sknots, scontrols = knots(s), controls(s)
    allD = SVector{D}(NTuple{D}(i for i in 1:D))
    CubicBezierCurve{D, T}(
        sknots[i₊₀, allD],
        (2*scontrols[i₊₀, allD] +   scontrols[i₊₁, allD])/3,
        (  scontrols[i₊₀, allD] + 2*scontrols[i₊₁, allD])/3,
        sknots[i₊₁, allD]
    )
end

function get_curve_and_t(t::T1, s::CubicSpline{D, T2}) where {D, T1, T2}
    # t is always > 0, so when we ceil we always get a positive integer
    i = Int(ceil(t))
    t_curve = t-i+1
    curve = get_curve(s, i)
    return curve, i, t_curve 
end

function get_curve_and_t(t::T1, s::LoopingCubicSpline{D, T2}) where {D, T1, T2}
    N = n_knots(s)
    i = Int(floor(t))
    t_curve = t - i
    i_mod = mod(i, N) + 1
    curve = get_curve(s, i_mod)
    return curve, i_mod, t_curve 
end

function spline_position(t::T1, s::CubicSpline{D, T2}) where {D, T1, T2}
    """
    Gives position r(t) for cubic spline s. If s is formed of N knots, then 
    r(0) = p₀
    r(N) = pₙ
    """
    T_out = SVector{D, promote_type(T1, T2)}
    N_knots = n_knots(s)
    max_t = N_knots - 1.0

    if (t > 0.0) & (t < max_t)
        curve, _, t_curve = get_curve_and_t(t, s)
        return curve_position(t_curve, curve)::T_out
    else
        # Linear extrapolation
        sknots, scontrols = knots(s), controls(s)
        allD = SVector{D}(NTuple{D}(i for i in 1:D))
        if t ≤ 0.0
            p₁ = sknots[1, allD]
            p₂ = (2*scontrols[1, allD] + scontrols[2, allD])/3
            return (p₁ + t*3*(p₂ - p₁))::T_out
        else
            p₃ = (scontrols[end-1, allD] + 2 * scontrols[end, allD])/3
            p₄ = sknots[end, allD]
            dt = (t-max_t)
            return (p₄ + dt*3*(p₄ - p₃))::T_out
        end
    end
end

function spline_derivative(t::T1, s::CubicSpline{D, T2}) where {D, T1, T2}
    """
    Gives position r(t) for cubic spline s. If s is formed of N knots, then 
    r(0) = p₀
    r(N) = pₙ
    """
    T_out = SVector{D, promote_type(T1, T2)}
    N_knots = n_knots(s)
    max_t = N_knots - 1.0

    if (t > 0.0) & (t < max_t)
        curve, _, t_curve = get_curve_and_t(t, s)
        return curve_derivative(t_curve, curve)::T_out
    else
        # Linear extrapolation
        sknots, scontrols = knots(s), controls(s)
        allD = SVector{D}(NTuple{D}(i for i in 1:D))
        if t ≤ 0.0
            p₁ = sknots[1, allD]
            p₂ = (2*scontrols[1, allD] + scontrols[2, allD])/3
            return T_out(3*(p₂ - p₁))
        else
            p₃ = (scontrols[end-1, allD] + 2 * scontrols[end, allD])/3
            p₄ = sknots[end, allD]
            return T_out(3*(p₄ - p₃))
        end
    end
end

function spline_second_derivative(t::T1, s::CubicSpline{D, T2}) where {D, T1, T2}
    """
    Gives position r(t) for cubic spline s. If s is formed of N knots, then 
    r(0) = p₀
    r(N) = pₙ
    """
    T_out = SVector{D, promote_type(T1, T2)}
    N_knots = n_knots(s)
    max_t = N_knots - 1.0

    if (t > 0.0) & (t < max_t)
        curve, _, t_curve = get_curve_and_t(t, s)
        return curve_second_derivative(t_curve, curve)::T_out
    else
        # Linear extrapolation
        if t ≤ 0.0
            return zero(T_out)
        else
            return zero(T_out)
        end
    end
end

function spline_position(t::T1, s::LoopingCubicSpline{D, T2}) where {D, T1, T2}
    T_out = SVector{D, promote_type(T1, T2)}
    curve, _, t_curve = get_curve_and_t(t, s)
    return curve_position(t_curve, curve)::T_out
end

function spline_derivative(t::T1, s::LoopingCubicSpline{D, T2}) where {D, T1, T2}
    T_out = SVector{D, promote_type(T1, T2)}
    curve, _, t_curve = get_curve_and_t(t, s)
    return curve_derivative(t_curve, curve)::T_out
end

function spline_second_derivative(t::T1, s::LoopingCubicSpline{D, T2}) where {D, T1, T2}
    T_out = SVector{D, promote_type(T1, T2)}
    curve, _, t_curve = get_curve_and_t(t, s)
    return curve_second_derivative(t_curve, curve)::T_out
end

integrate_length(spline::CubicSpline{D, T}) where {D, T} = integrate_length(Val(D), spline.data.knots, spline.data.controls)
integrate_length(spline::LoopingCubicSpline{D, T}) where {D, T} = integrate_length(Val(D), spline.data.knots, spline.data.controls)

function binary_search(func::F, val, tol, low, high; maxiters = 50) where F
    iters = 0
    while low + tol <= high
        mid = (low + high)/2
        guess = func(mid)
        if guess > val
            high = mid
        else
            low = mid
        end
        ((iters += 1) > maxiters) && throw(ErrorException("Maxiters"))
    end
    return (low + high)/2
end

function resample(spline::Spline{D, T}; tol=1e-4) where {D, T}
    N_knots = n_knots(spline)
    L = integrate_length(spline)
    δ_desired = L/(N_knots-1.0)
    # new_knots = Vector{SVector{D, T}}()
    # push!(new_knots, SVector{D, T}(spline_position(0.0, spline)))
    new_knots = copy(spline.data.knots)
    t = 0.0    
    # Each new knot is spaced `spacing` apart
    # while t < N_knots
    for i = 2:(N_knots-1)
        dist_func = (δ) -> integrate_spline_between(spline, t, t+δ)
        δ = binary_search(dist_func, δ_desired, tol, 0.0, N_knots-t)
        t = t + δ
        # push!(new_knots, spline_position(t, spline))
        new_knots[i, :] .= spline_position(t, spline)
    end
    # new_knots_mat = reduce(vcat, new_knots)
    # @show new_knots_mat
    # return CubicSpline(new_knots_mat)
    return CubicSpline(new_knots)
end

end