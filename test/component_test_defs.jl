using Random

using VMRobotControl

using VMRobotControl: Storage, Inertance, GenericComponent

using StaticArrays
using Test

function build_empty_mechanism()
    mechanism = Mechanism{Float64}("Empty mechanism")
    compiled_mechanism = compile(mechanism)
    t = 0.0
    q = q̇ = zero(SVector{0, Float64})
    gravity = VMRobotControl.DEFAULT_GRAVITY
    cache = new_dynamics_cache(compiled_mechanism)
    precompute!(cache, compiled_mechanism, t, q, q̇, gravity)
    return compiled_mechanism, cache
end

function test_component(_c, test_opspace_force, test_inertance, test_stored_energy)
    m, precomp = build_empty_mechanism()
    c = m(_c)
    # @code_warntype _test_component(c, precomp, test_opspace_force, test_inertance, test_stored_energy)
    # @inferred _test_component(c, precomp, test_opspace_force, test_inertance, test_stored_energy)
    _test_component(c, precomp, test_opspace_force, test_inertance, test_stored_energy)
end

function _test_component(component, cache, test_opspace_force, test_inertance, test_stored_energy)
    test_opspace_force && opspace_force(cache, component)
    generalized_force(cache, component)
    # VMRobotControl._wrench(c, precomp)        
    test_inertance && inertance_matrix(cache, component)
    test_stored_energy && stored_energy(cache, component)
    
    local a

    test_opspace_force &&   (opspace_force_allocs = @allocations(opspace_force(cache, component)))
                            generalized_force_allocs = @allocations(generalized_force(cache, component))
    test_inertance &&       (inertance_matrix_allocs = @allocations(inertance_matrix(cache, component)))
    test_stored_energy &&   (stored_energy_allocs = @allocations(stored_energy(cache, component)))


    test_opspace_force &&   @test opspace_force_allocs == 0
                            @test a = generalized_force_allocs == 0
    # @test @allocations(VMRobotControl._wrench(c, precomp)) == 0
    test_inertance &&       @test inertance_matrix_allocs == 0
    test_stored_energy &&   @test stored_energy_allocs  == 0
    nothing
end

function test_components()
    # Strategy
    #
    #   There are several properties that we can test for each component. Components expose the 
    # following interfaces: 
    #   - opspace_force
    #       # From which is derived:
    #       - generalized_force
    #       - wrench
    #   - inertance_matrix
    #   - stored_energy
    #
    # In the future it would be nice to test enery properties, such as that storages/inertances
    # conserve energy, and dissipations dissipate energy.
    #
    # For now we will test implementation details:
    #   - That calls do not allocate
    #   - That calls to ComponentCollections do not allocate
    
    rng = MersenneTwister(1234)
    coord1 = FrameOrigin("root_frame")
    coord2 = LinkFrameCoordinate(coord1, "root_frame")
    cs = [
        PointMass(1.0, coord1),
        PointMass(1.0, coord2),
        Inertia(zero(SMatrix{3,3,Float64, 9}), "root_frame"),
        LinearSpring(1.0, coord1),
        LinearSpring(1.0, coord2),
        GravityCompensator(1.0, coord1),
        GravityCompensator(1.0, coord2),
        LinearDamper(1.0, coord1),
        LinearDamper(1.0, coord2),
    ]
    @testset "Individual components" begin
        for c in cs
            C = typeof(c)
            test_inertance = C<:Inertance || C<:GenericComponent
            test_stored_energy = C<:Storage || C<:Inertance || C<:GenericComponent
            test_component(c, true, test_inertance, test_stored_energy)
        end
    end

    energy_storing_components = [c for c in cs if (isa(c, GenericComponent) || isa(c, Storage) || isa(c, Inertance))]
    inerting_components = [c for c in cs if (isa(c, GenericComponent) || isa(c, Inertance))]
    all_components = [c for c in cs]

    @testset "Component collections" begin
        f = test_component(ComponentCollection(energy_storing_components), false, false, true)
        f = test_component(ComponentCollection(inerting_components), false, true, false)
        f = test_component(ComponentCollection(all_components), false, false, false)
    end
end

test_components();
