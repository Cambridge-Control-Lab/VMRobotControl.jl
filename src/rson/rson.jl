module RSON

export 
    find_frame_index,
    parseRSON,
    parseRSONString,
    RSONParserConfig,
    serializeRSON

using ColorTypes: 
    RGBA
using DigitalAssetExchangeFormatIO # To support .dae files
using FileIO: 
    load
using GeometryBasics
using LinearAlgebra: 
    Diagonal,
    Symmetric,
    issymmetric,
    isposdef,
    normalize,
    norm
using 
    OrderedCollections,
    StaticArrays

using ..VMRobotControl
using ..VMRobotControl:
    AbstractJointData,
    RevoluteData,
    PrismaticData,
    Rigid,
    MechanismJoint,
    MeshFromFile
   
import JSON

################################################
# Consts
################################################
# Versions
const RSON_PARSER_VERSION = (0, 1, 0) # v0.1.0
const RSON_PARSER_VERSION_STRING = let
    major, minor, patch = RSON_PARSER_VERSION
    "$major.$minor.$patch"
end

# Json parser types
const JSON_STR = String
const JSON_FLOAT = Float64
const JSON_INT = Int64
const JSON_ARRAY = Vector{Any}
const JSON_BOOL = Bool
const JSON_NULL = Nothing
const JSON_OBJECT = OrderedDict{String, Any}


# Strings
# general 
const S_NAME = "name"
const S_TYPE = "type"
# rson types
const S_MECHANISM = "mechanism"
const S_VIRTUAL_MECHANISM_SYSTEM = "virtual_mechanism_system"
# mechanism fields
const S_FRAMES = "frames"
const S_JOINTS = "joints"
const S_COORDINATES = "coordinates"
const S_COMPONENTS = "components"
const S_MATERIALS = "materials"
const S_VISUALS = "visuals"

# allowed extra fields
const S_EXTRA = "extra"
const S_DESCRIPTION = "description"

include("./parsing.jl")
include("./serializing.jl")

end
