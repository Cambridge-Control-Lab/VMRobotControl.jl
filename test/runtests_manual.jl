begin
    using TestEnv
    TestEnv.activate("VMRobotControl")

    include("setup_test_mechanisms.jl")

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
end

# Run one at a time by highlighting a line
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
