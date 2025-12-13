p = train(f, df, p, N, η)

sol1 = solve(prob; p=p0) 
sol2 = solve(prob; p=p)

@show C1 = f(p0)
@show C2 = f(p)

fig = Figure()
ax = LScene(fig[1,1])


cache = Observable(new_kinematics_cache(m))
robotvisualize!(ax, cache)
println("Animating")
animate_robot_odesolution(fig, sol1, cache, "/tmp/pre.mp4")
animate_robot_odesolution(fig, sol2, cache, "/tmp/post.mp4")
