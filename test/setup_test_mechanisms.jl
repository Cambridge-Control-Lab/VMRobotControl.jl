# TEST_ENZYME = "Enzyme" ∈ keys(Pkg.project().dependencies)
TEST_ENZYME = false
# TEST_ENZYME = true

using DiffResults
using ForwardDiff
using FileIO
using LinearAlgebra
using ProgressMeter
using Random
using StaticArrays
using Test
using UUIDs
using VMRobotControl
using VMRobotControl: joint_transform, joint_twist, joint_vpa, jacobian_column, AbstractJointData
using VMRobotControl: Twist, SpatialAcceleration
using VMRobotControl.Hessians: my_hessian, hessian_vector_product
using VMRobotControl.Transforms: angular_velocity, AxisAngle, AxisAngleDerivative, quatmul_matrix, quatmul_geodual_bivector_matrix
using VMRobotControl: get_inertance_components
using VMRobotControl: Storage, Inertance, GenericComponent
import VMRobotControl: jacobian # Explicitly import to avoid name clash with Enzyme.jacobian

try
    FileIO.add_format(format"DAE", (), ".dae", [:DigitalAssetExchangeFormatIO => UUID("43182933-f65b-495a-9e05-4d939cea427d")])
catch e
    if e == ErrorException("format DAE is already registered")
    else
        rethrow(e)
    end
end

if TEST_ENZYME
    @info "Enabling Enzyme tests"
    import Enzyme
    using Enzyme:
        autodiff,
        Active,
        Const,
        Duplicated,
        Reverse
end

################################################################################

module_path = joinpath(splitpath(dirname(pathof(VMRobotControl)))[1:end-1])
# Walk dir to find all rsons
rsons = String[]
for (root, dirs, files) in walkdir(joinpath(module_path, "RSONs"))
    for file in files
        if endswith(file, ".rson")
            path = joinpath(root, file)
            push!(rsons, path)
        end
    end
end

urdfs = joinpath.((module_path,), ("URDFs",), [
    "sciurus17_description/urdf/sciurus17.urdf",
    "franka_description/urdfs/fr3.urdf"
])


systems = let 
    p = Progress(length(rsons) + length(urdfs); desc="Parsing robot files...", dt=0.1)
    systems = Any[]
    for rson in rsons
        rson_parser_cfg = RSONParserConfig(; parse_visuals=false, suppress_warnings=false, error_on_not_recognized=false)
        push!(systems, parseRSON(rson, rson_parser_cfg))
        next!(p)
    end
    urdf_parser_cfg = URDFParserConfig(;parse_visuals=false, suppress_warnings=true)
    for urdf in urdfs
        push!(systems, parseURDF(urdf, urdf_parser_cfg))
        next!(p)
    end
    systems
end

compiled_systems = @showprogress desc="Compiling Robots/Virtual Mechanism Systems" dt=0.2 map(compile, systems)
compiled_mechanisms = [s for s in compiled_systems if s isa CompiledMechanism]

# For dynamics tests, only consider systems with any inertances defined
systems_with_inertances = [s
    for s in compiled_systems
        if (
            (isa(s, CompiledMechanism) && !isempty(get_inertance_components(s)))
            ||
            (isa(s, CompiledVirtualMechanismSystem) && !isempty(get_inertance_components(s.robot)) && !isempty(get_inertance_components(s.virtual_mechanism)))
        )
]
