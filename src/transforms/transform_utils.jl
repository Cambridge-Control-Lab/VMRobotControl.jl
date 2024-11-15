"""
    quatmul_geodual_bivector_matrix(r::Rotor)

Returns a matrix `S` that corrects from a rotor jacobian ̇r = J⁽ʳ⁾q to an angular
velocity jacobian ω = J⁽ʷ⁾̇q, by J⁽ʷ⁾ = S J⁽ʳ⁾.

Equivalent to performing quaternion multiplication with, flipping the direction 
of the vector and extracting the vector part of the result....
"""
function quatmul_geodual_bivector_matrix(r::Rotor)
    # ω = -2*bivector(r * geo_dual(ṙ))
    #                                 [1  0  0  0
    # ω = -2*[0 1 0 0                  0 -1  0  0 
    #         0 0 1 0                  0  0 -1  0
    #         0 0 0 1] * quatmul(u *   0  0  0 -1] * Jʳq̇)

    #                  [u[1], -u[2], -u[3], -u[4]    [1  0  0  0
    # -2 * [0 1 0 0     u[2],  u[1], -u[4],  u[3]     0 -1  0  0  
    #       0 0 1 0     u[3],  u[4],  u[1], -u[2]     0  0 -1  0 
    #       0 0 0 1] *  u[4], -u[3],  u[2],  u[1]] *  0  0  0 -1] Jʳq̇

    #                  [u[1], -u[2], -u[3], -u[4]    [1  0  0  0
    # -2 * [0 1 0 0     u[2],  u[1], -u[4],  u[3]     0 -1  0  0  
    #       0 0 1 0     u[3],  u[4],  u[1], -u[2]     0  0 -1  0 
    #       0 0 0 1] *  u[4], -u[3],  u[2],  u[1]] *  0  0  0 -1] Jʳq̇

    #                                    [1  0  0  0
    # -2 * [u[2],  u[1], -u[4],  u[3]     0 -1  0  0  
    #       u[3],  u[4],  u[1], -u[2]     0  0 -1  0 
    #       u[4], -u[3],  u[2],  u[1]] *  0  0  0 -1] Jʳq̇

    #                                 
    # -2 * [u[2], -u[1],  u[4], -u[3]  
    #       u[3], -u[4], -u[1],  u[2] 
    #       u[4],  u[3], -u[2], -u[1]] Jʳq̇

    u = rotor_to_svector(r)
    -2.0 * @SMatrix [u[2]  -u[1]   u[4]  -u[3];
                     u[3]  -u[4]  -u[1]   u[2];
                     u[4]   u[3]  -u[2]  -u[1]]
end

"""
    skew(v::SVector{3})

Return the skew symettric matrix operator performing vector product.
"""
function skew(v::SVector{3, T}) where T
    # TODO replace with SkewHermitian matrix or similar type.
    O = zero(T)
    @SMatrix [ O   -v[3]  v[2];
               v[3] O    -v[1];
              -v[2] v[1]  O]
end

function compute_stationary_point_A_b(transforms::AbstractVector{<:Transform})
    N = length(transforms)
    A = Matrix{Float64}(undef, (3*N, 6))
    b = Vector{Float64}(undef, 3*N)

    for i = 1:N
        A[(i-1)*3+1:i*3, 1:3] .= rotation_matrix(rotor(transforms[i]))
        A[(i-1)*3+1:i*3, 4:6] .= -I(3)
        b[(i-1)*3+1:i*3] .= -origin(transforms[i])
    end
    A, b
end

function find_stationary_point(transforms::AbstractVector{<:Transform})
    A, b = compute_stationary_point_A_b(transforms)
    θ = A\b
    p_fixed, offset = θ[SVector(4, 5, 6)], θ[SVector(1, 2, 3)]
    p_fixed, offset, θ, A, b
end
