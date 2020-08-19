# climbing simulator

**Matlab climbing and walking simulation platform for limbed robots**

![ex3_uneven_dynamic_Uno-gait-planning_stability_polyhedron.gif](./docs/media/ex3_uneven_dynamic_Uno-gait-planning_stability_polyhedron.gif)

Author(s) and maintainer(s): [Space Robotics Lab.](http://www.astro.mech.tohoku.ac.jp/e/index.html) Climbing Robotics Team

[![srl-logo-original.jpg](./docs/media/srl-logo-original.jpg)](http://www.astro.mech.tohoku.ac.jp/e/index.html)
![crt_color_logo_a_hi-reso.png](./docs/media/crt_color_logo_a_hi-reso.png)

The climbing simulator is being developed by the Climbing Robotics Team in [Space Robotics Laboratory](http://www.astro.mech.tohoku.ac.jp/e/index.html) at Tohoku University, Japan. 

The source code is released under a [BSD 3-Clause license](./LICENSE).

## Overview
This simulator wraps up functions for:
* [x] Dynamic analysis and simulation of articulated multi-body systems with a floating base (*This is performed by [SpaceDyn](http://www.astro.mech.tohoku.ac.jp/spacedyn/)* [1])
* [x] Design of legged robotic system
* [x] Environment demonstration (any terrain shape, inclination, and gravity) 
* [x] Visualization (Robot, map, support polygons, stability criterion, and time-history of any state variables) 

#### Simulation examples
| ![ex1_uneven_dynamic_fixed_stride.gif](./docs/media/ex1_uneven_dynamic_fixed_stride.gif) | ![ex2_flat_kinematic_Uno-gait-planning.gif](./docs/media/ex2_flat_kinematic_Uno-gait-planning.gif) |![GIA-poly-hedoron.png](./docs/media/GIA-poly-hedoron.png)|
|--------|--------|--------|
| 1) Standard gait on uneven terrain | 2) Optimal foothold selection on descrete grippable points [2] | 3) Stability region visualization [3] |

## Usage

#### Requirements
We confirmed the code is working with:
* Matlab 2018b or higher
* [SpaceDyn](http://www.astro.mech.tohoku.ac.jp/spacedyn/) [1] - a MATLAB Toolbox for Space and Mobile Robots (version 2, release 0)
  - SpaceDyn toolbox v2r0 is also installed by cloning this repository (under `lib/spacedyn_v2r0`)

#### Build and Run
* Download the files from this repository
* Open MATLAB
* Select `climbing_simulator/` as the current folder for MATLAB
  - You should see `dat/`, `lib/` and `src/` directories under it
* Setup the parameters as you want
* Run `climb_main.m` file (**Note: MATLAB might ask you to change folder or add path. Choose to "add path"**)

## Publications
[1] Kazuya Yoshida et. al., "[The SpaceDyn: a MATLAB toolbox for space and mobile robots](https://ieeexplore.ieee.org/document/811712)", Proceedings of the 1999 IEEE/RSJ International Conference on Intelligent Robots and Systems. Human and Environment Friendly Robots with High Intelligence and Emotional Quotients (Cat. No.99CH36289), Kyongju, South Korea, 1999, pp. 1633-1638 vol.3, [doi: 10.1109/IROS.1999.811712](https://doi.org/10.1109/IROS.1999.811712).

[2] Kentaro Uno et al., "[Gait Planning for a Free-Climbing Robot Based on Tumble Stability](https://ieeexplore.ieee.org/document/8700455)", Proceedings of the 2019 IEEE/SICE International Symposium on System Integration (SII), Paris, France, 2019, pp. 289-294, [doi: 10.1109/SII.2019.8700455](https://doi.org/10.1109/SII.2019.8700455).

[3] Warley F. R. Ribeiro et al., "[Dynamic Equilibrium of Climbing Robots based on Stability Polyhedron for Gravito-Inertial Acceleration]()", Proceedings of the 23rd International Conference on Climbing and Walking Robots and the Support Technologies for Mobile Machines, (remote conference), 2020.

## FAQ
Please contact us (limb@astro.mech.tohoku.ac.jp) if there is any question or suggestion
