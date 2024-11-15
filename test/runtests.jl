if (dir = splitpath(pwd())[end]) != "test" # If we are not in the test directory, cd to it
    if dir == "VMRobotControl.jl"
        @info "Changing directory to ./test, and activating test environment"
        cd("test")
        using TestEnv
        TestEnv.activate()
    else
        error("Not in correct directory for running tests. The current working directory is: '$(pwd())'. In this dir is: '$(readdir())'")
    end
end

using Pkg
# TEST_ENZYME = "Enzyme" ∈ keys(Pkg.project().dependencies)
TEST_ENZYME = false
# TEST_ENZYME = true


using DiffResults
using FileIO
using ForwardDiff
using LinearAlgebra
using ProgressMeter
using Random

using VMRobotControl

using VMRobotControl: joint_transform, joint_twist, joint_vpa, jacobian_column, AbstractJointData
using VMRobotControl: Twist, SpatialAcceleration
using VMRobotControl.Hessians: my_hessian, hessian_vector_product
using VMRobotControl.Transforms: angular_velocity, AxisAngle, AxisAngleDerivative, quatmul_matrix, quatmul_geodual_bivector_matrix
using VMRobotControl: get_inertance_components
using VMRobotControl: Storage, Inertance, GenericComponent

using StaticArrays
# using StatsBase: sample
using Test
using UUIDs

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

function test_on_mechanisms(test, mechanisms::Vector)
    @showprogress desc=string(test) for m in mechanisms
        @testset "Robot: '$(name(m))'" begin
            test(m)
        end
    end
end

rsons = [
    "inertia.rson",
    "inerter.rson",
    "pendulum.rson",
    "robot.rson",
    "simple3link.rson",
    "planar_drill_guide.rson",
    "franka_panda/pandaDremelMount.rson",
    "franka_panda/pandaSurgical.rson",
    "franka_panda/pandaSurgicalSystem.rson"
]

# urdfs = readdir("../URDFs/source")
urdfs = [
    "../URDFs/sciurus17_description/urdf/sciurus17.urdf",
    "../URDFs/franka_description/urdfs/fr3.urdf"
]

# Walk dir
rsons = String[]
for (root, dirs, files) in walkdir("../RSONs")
    for file in files
        if endswith(file, ".rson")
            path = joinpath(root, file)
            push!(rsons, path)
        end
    end
end

systems = let 
    p = Progress(length(rsons) + length(urdfs); desc="Parsing robot files...", dt=0.1)
    systems = Any[]
    for rson in rsons
        rson_parser_cfg = RSONParserConfig(; parse_visuals=false, suppress_warnings=false, error_on_not_recognized=false)
        push!(systems, parseRSON(rson, rson_parser_cfg))
        next!(p)
    end
    urdf_parser_cfg = URDFParserConfig(;parse_visuals=false, suppress_warnings=true)
    for urfd in urdfs
        push!(systems, parseURDF(urfd, urdf_parser_cfg))
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

if isdefined(Main, :Revise)
    Revise.includet("pendulum_test.jl")
    Revise.includet("scara_test.jl")
    Revise.includet("joint_test.jl")
    Revise.includet("velocity_kinematics_test.jl")
    Revise.includet("coordinate_test.jl")
    Revise.includet("dynamics_test.jl")
    Revise.includet("inverse_dynamics_test.jl")
    Revise.includet("energy_test.jl")
    Revise.includet("rson_test.jl")
    Revise.includet("Enzyme_compat_test.jl")
else
    include("pendulum_test.jl")
    include("scara_test.jl")
    include("joint_test.jl")
    include("velocity_kinematics_test.jl")
    include("coordinate_test.jl")
    include("dynamics_test.jl")
    include("inverse_dynamics_test.jl")
    include("energy_test.jl")
    include("rson_test.jl")
    include("Enzyme_compat_test.jl")
end

results = @testset "All tests" begin
    results = pendulum_tests();
    results = scara_tests();
    results = single_joint_tests(mobile_jointtypes);
    results = single_joint_mechanism_tests(all_jointtypes);
    results = double_joint_tests(mobile_jointtypes);
    results = double_joint_mechanism_tests(all_jointtypes);
    small_immut_mechanisms = [m for m in compiled_mechanisms if ndof(m) < 5];
    results = @testset "Velocity Kinematics FD" test_on_mechanisms(test_velocity_kinematics_vs_finitediff, compiled_mechanisms);
    results = @testset "Velocity Kinematics AD" test_on_mechanisms(test_velocity_kinematics_vs_autodiff, small_immut_mechanisms);
    results = @testset "Coordinate tests" test_on_mechanisms(test_coordinates, compiled_systems);
    results = @testset "Dynamics tests" test_on_mechanisms(test_dynamics, systems_with_inertances);
    # results = @testset "Inverse Dynamics tests" test_on_mechanisms(test_inverse_dynamics, compiled_mechanisms); # TODO 
    results = @testset "Energy tests" test_on_mechanisms(test_energy, systems_with_inertances);
    results = @testset "RSON tests" test_on_mechanisms(test_rson, systems);
    # TODO ForwardDiff compat tests
    TEST_ENZYME && (@testset "Enzyme compat tests" test_on_mechanisms(test_enzyme_compat, compiled_systems));
end;
nothing
