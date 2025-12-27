include("setup_test_mechanisms.jl")

if isdefined(Main, :Revise)
    Revise.includet("pendulum_test_defs.jl")
    Revise.includet("scara_test_defs.jl")
    Revise.includet("joint_test_defs.jl")
    Revise.includet("velocity_kinematics_test_defs.jl")
    Revise.includet("coordinate_test_defs.jl")
    Revise.includet("dynamics_test_defs.jl")
    Revise.includet("inverse_dynamics_test_defs.jl")
    Revise.includet("energy_test_defs.jl")
    Revise.includet("rson_test_defs.jl")
    Revise.includet("Enzyme_compat_test_defs.jl")
else
    include("pendulum_test_defs.jl")
    include("scara_test_defs.jl")
    include("joint_test_defs.jl")
    include("velocity_kinematics_test_defs.jl")
    include("coordinate_test_defs.jl")
    include("dynamics_test_defs.jl")
    include("inverse_dynamics_test_defs.jl")
    include("energy_test_defs.jl")
    include("rson_test_defs.jl")
    include("Enzyme_compat_test_defs.jl")
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
    results = @testset "Inverse Dynamics tests" test_on_mechanisms(test_inverse_dynamics, compiled_mechanisms); # TODO 
    results = @testset "Energy tests" test_on_mechanisms(test_energy, systems_with_inertances);
    results = @testset "RSON tests" test_on_mechanisms(test_rson, systems);
    # TODO ForwardDiff compat tests
    TEST_ENZYME && (@testset "Enzyme compat tests" test_on_mechanisms(test_enzyme_compat, compiled_systems));
end;
nothing
