using Revise

using 
    DifferentialEquations,
    Documenter,
    ForwardDiff,
    GLMakie,
    Literate,
    VMRobotControl,
    StaticArrays
    
# https://documenter.juliadocs.org/stable/man/guide/#Package-Guide
TUTORIAL_PAGES = [
    "Introduction" => "tutorials/introduction.md",
    "Building a mechanism" => "tutorials/building.md",
    "Using a mechanism" => "tutorials/using.md",
    "Virtual Model Control of a Simulated Robot" => "tutorials/vms.md",
    "Plotting with Makie.jl" => "tutorials/plotting.md",
    "Simulating with DifferentialEquations.jl" => "tutorials/simulating.md",
    "Virtual Model Control of a Real Robot using ROS2" => "tutorials/control.md",
    "Optimization" => "tutorials/optimization.md"
]
EXAMPLES_PAGES = [
    "Franka Impedance Control" => "examples/franka_impedance_control.md",
    "Sciurus reaching with obstacle avoidance" => "examples/sciurus_reaching.md",
    "Franka compliant path following" => "examples/compliant_path_following.md",
    "Pendulum on a Bezier-Spline Rail" => "examples/rail_robot.md"
]

API_PAGES = "api/api.md"

# Make examples pages
Literate.markdown("./examples/compliant_path_following.jl", "./docs/src/examples/"; flavor=Literate.CommonMarkFlavor(), execute=true)
Literate.markdown("./examples/franka_impedance_control.jl", "./docs/src/examples/"; flavor=Literate.CommonMarkFlavor(), execute=true)
Literate.markdown("./examples/sciurus_reaching.jl", "./docs/src/examples/"; flavor=Literate.CommonMarkFlavor(), execute=true)
Literate.markdown("./examples/rail_robot.jl", "./docs/src/examples/"; flavor=Literate.CommonMarkFlavor(), execute=true)


makedocs(
    sitename="VMRobotControl.jl",
    source="./src",
    pages = [
        "Home" => "index.md",
        "Tutorials" => TUTORIAL_PAGES,
        "Examples" => EXAMPLES_PAGES,
        "API" => API_PAGES,
        "Developer Notes" => "developer/developer.md"
    ],
    modules=[VMRobotControl],
    pagesonly = true,
    warnonly = true,
    linkcheck = true,
    draft=false,
)
