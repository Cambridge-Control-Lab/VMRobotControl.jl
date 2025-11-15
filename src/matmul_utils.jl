"""
    custom_transpose_mul_add!(Y, X, a)

Computes a * X' * X and adds the result to Y.
"""
function custom_transpose_mul_add!(Y, X, a)
    m, n = size(X)
    @assert size(Y) == (n, n)
    for i = 1:n
        for j = 1:n
            t = zero(eltype(Y))
            for k = 1:m
                t += X[k, i] * X[k, j]
            end
            Y[i, j] = Y[i, j] + a * t
        end
    end
end
