# VMRobotControl.jl

VMRobotControl allows you to build dynamic controllers for your robot as intuitive
virtual mechanisms.
You design your controller by building a virtual mechanism.
You connect the virtual mechanism to the robot through compliant elements (like springs and dampers).
The control action is automatically translated to the robot actuators (desired joint forces/torques).

This package is designed for building, simulating and controlling robots
using passivity-based controllers represented by virtual mechanisms.
This is in the tradition of *virtual model control* and *energy shaping/damping injection*.
For a defined robot geometry, you can select specific *coordinates* and connect
 *components* to shape the robot dynamic properties.
properties.
This provides a simple way to program complex robot behaviour incrementally, from mechanical 
primitives.

```@raw html
<video controls width="49%" height="auto" autoplay loop>
    <source src="assets/franka_impedance_control.mp4" type="video/mp4">
</video>
<video controls width="49%" height="auto" autoplay loop>
    <source src="assets/compliant_path_following.mp4" type="video/mp4">
</video>
```

# Design features

- Easy creation of rigid body mechanisms with dynamic components
- Library of components like springs and dampers, both linear and nonlinear
- Library of geometry 'coordinates' with Jacobians to help build components
- `VirtualMechanismSystem` type for building virtual-mechanism controllers
- Allocation free computation methods for fast simulation/control
- Loading URDFs
- Plotting and simulation with Makie.jl and DifferentialEquations.jl

```@raw html
<video controls width="66%" height="auto" autoplay loop>
    <source src="assets/rail_robot.mp4" type="video/mp4">
</video>
```
# Installation

Install [julia](https://julialang.org/).

The current recommended method to install this package is to clone the github repository and mark
it as a package under development. In your terminal:
```sh
git clone git@github.com:Cambridge-Control-Lab/VMRobotControl.jl.git
julia
```
Then in julia, hit `]` to enter Pkg mode, and run the commands
```julia
registry update
dev ./VMRobotControl.jl
```

```@raw html
<video controls width="98%" height="auto" autoplay loop>
    <source src="assets/sciurus_reaching.mp4" type="video/mp4">
</video>
```
# Citing this library
Please cite this library as
```bibtex
@misc{VMRobotControl,
    author = {Daniel Larby},
    title = {{VMRobotControl.jl}},
    howpublished = {\url{https://github.com/Cambridge-Control-Lab/VMRobotControl}},
    year = {2024}
}
```
and also consider citing the authors works on [optimal virtual model control](https://doi.org/10.48550/arXiv.2411.06627)
and [robot control for robotic surgery](https://doi.org/10.1016/j.ifacol.2023.10.015).
```bibtex
@misc{larby2024optimalvirtualmodelcontrol,
      title={Optimal Virtual Model Control for Robotics: Design and Tuning of Passivity-Based Controllers}, 
      author={Daniel Larby and Fulvio Forni},
      year={2024},
      eprint={2411.06627},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      doi = {https://doi.org/10.48550/arXiv.2411.06627},
      url={https://arxiv.org/abs/2411.06627}, 
}
@article{LARBY20238548,
    title = {A Generalized Approach to Impedance Control Design for Robotic Minimally Invasive Surgery},
    journal = {IFAC-PapersOnLine},
    year = {2023},
    note = {22nd IFAC World Congress},
    doi = {https://doi.org/10.1016/j.ifacol.2023.10.015},
    author = {Daniel Larby and Fulvio Forni},
}
```

# Authors and Acknowledgements

I'm Daniel Larby. This package was initially developed during my PhD ([here is my website!](https://daniellarby.net/))
at the Control Lab, in the Department of Engineering, University of Cambridge, CB2 1PZ, UK.

I was supported by my supervisor, [Fulvio Forni](https://sites.google.com/site/fulvioforni/), and
the other students in the group.

My PhD was graciously supported by the Engineering and Physical Sciences Research Council; 
and by [CMR Surgical](https://us.cmrsurgical.com/).

