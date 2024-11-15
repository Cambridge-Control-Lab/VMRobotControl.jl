include("./hessians.jl")

module Sandbox

# using Main.RigidBody
# using Main.RigidBody: dynamics
# using Main.RSON: parse

# using DifferentialEquations
using ForwardDiff
using StaticArrays
using DiffResults
# import Plots
# using GLMakie

using Main.Hessians
using InteractiveUtils

f = x -> SVector(x[1]^3, x[2]^2)
x0 = SVector(1.0, 1.0)

@code_warntype hessian(f, x0)
H = hessian(f, x0)

DiffResults.value(H)
DiffResults.derivative(H)
DiffResults.hessian(H)

end