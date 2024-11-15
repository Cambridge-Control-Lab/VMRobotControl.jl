module URDF

export parseURDF, parseURDFString
export URDFParserConfig

using ColorTypes: RGBA
using DigitalAssetExchangeFormatIO # To support .dae files
using EzXML
using FileIO: load
using GeometryBasics
using LinearAlgebra: Diagonal, Symmetric, issymmetric, isposdef
using StaticArrays
using ..VMRobotControl

include("./parse_urdf.jl")
end