using Revise

using VMRobotControl
using VMRobotControl.StaticArrays
using VMRobotControl.Hessians.ForwardDiff
using VMRobotControl.Hessians.ForwardDiff: Dual, Tag
import VMRobotControl.Hessians.ForwardDiff: ≺
using VMRobotControl.DifferentialEquations

using Random
using InteractiveUtils: @code_warntype
using LinearAlgebra
using BenchmarkTools

# struct HessiansTagType end
# HessiansTag(::AbstractArray{T}) where T  = typeof(Tag(HessiansTagType(), T))

struct TestTag end
≺(a::TestTag, b) = true
≺(a, b::TestTag) = false

function create_franka()
    rson = "../RSONs/rsons/franka_panda/pandaDummyInstrument.rson"
    parseRSON(rson)
end

function do_f_for_T(f::F, T) where F<:Function
    t_start = time()
    function profloop()
        while time() < t_start + T
            f()
        end
    end
    profloop()
end

begin
    mechanism = create_franka()    
    m = compile(mechanism)
    kcache = new_kinematics_cache(m)
    vcache = new_rbstates_cache(m)
    jcache = new_jacobians_cache(m)
    dcache = new_dynamics_cache(m)

    function test(kcache, dcache)
        
        gravity = VMRobotControl.DEFAULT_GRAVITY

        q = zero_q(dcache)
        q̇ = zero_q̇(dcache)
        q̈ = deepcopy(q̇)
        t = 0.0
        u = zero_u(dcache)

        tf = rand(Transform{Float64})

        @btime *($tf, $tf)
        @btime kinematics!($kcache, $t, $q)
        @btime velocity_kinematics!($dcache, $t, $q, $q̇)
        @btime compute_jacobians_fast!($m.rbtree, $transforms, $t, $q)
        @btime precompute!($m, $t, $q, $q̇, $gravity)
        @btime inertance_matrix($m, $precomp)
        @btime generalized_force($m, $precomp)
        @btime dynamics($m, $precomp, $u)

        # @code_warntype extended_kinematics(m.rbtree, q, q̇, t)
        # @code_warntype compute_jacobians_fast(m.rbtree, transforms, q)
        # @code_warntype precompute(m.rbtree, q, q̇, t)
        # @code_warntype inertance_matrix(m, precomp, q, q̇)
        # @code_warntype inertance_matrix(m.generic_components, precomp, q, q̇)
        # @code_warntype generalized_force(m, precomp, q, q̇)
        # @code_warntype generalized_force(m.inertances, precomp, q, q̇)
        # @code_warntype dynamics(m, q, q̇, t, u)

        # @profview do_f_for_T(() -> *(tf, tf), 1.0)
        # @profview do_f_for_T(() -> kinematics(m, t, q), 1.0)
        # @profview do_f_for_T(() -> velocity_kinematics(m.rbtree, t, q, q̇, gravity), 1.0)
        # @profview do_f_for_T(() -> compute_jacobians_fast(m.rbtree, transforms, t, q), 1.0)
        # @profview do_f_for_T(() -> precompute!(m, t, q, q̇, gravity), 10.0)
        # @profview do_f_for_T(() -> inertance_matrix(m, precomp), 1.0)
        # @profview do_f_for_T(() -> generalized_force(m, precomp), 10.0)
        # @profview_allocs do_f_for_T(() -> dynamics(m, q, q̇, t, u), 1.0)
        # @profview solve(prob, Tsit5(); save_everystep=false)
        nothing
    end

    test(m)
end
