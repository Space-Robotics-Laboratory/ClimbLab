# ClimbLab - Climbing robot Matlab simulator

![climblab.png](./docs/media/climblab.png)

Author(s) and maintainer(s): [Space Robotics Lab.](http://www.astro.mech.tohoku.ac.jp/e/index.html) Climbing Robotics Team

[![srl-logo-original.jpg](./docs/media/srl-logo-original.jpg)](http://www.astro.mech.tohoku.ac.jp/e/index.html)
![crt_color_logo_a_hi-reso.png](./docs/media/crt_color_logo_a_hi-reso.png)

The climbing simulator is being developed by the Climbing Robotics Team in [Space Robotics Laboratory](http://www.astro.mech.tohoku.ac.jp/e/index.html) at Tohoku University, Japan. 

The source code is released under a [BSD 3-Clause license](./LICENSE).

## Overview
This simulator wraps up functions for:
* [x] Dynamic analysis and simulation of articulated multi-body systems with a floating base (*This is performed by [SpaceDyn](https://github.com/Space-Robotics-Laboratory/SpaceDyn)* [1])
* [x] Design of legged robotic system
* [x] Environment demonstration (different terrain shape, inclination, and gravity)
* [x] Visualization (Robot, map, support polygons, stability criterion, and time-history of any state variables) 

#### Simulation examples
| ![ex1_uneven_dynamic_fixed_stride.gif](./docs/media/ex1_uneven_dynamic_fixed_stride.gif) | ![ex2_flat_kinematic_Uno-gait-planning.gif](./docs/media/ex2_flat_kinematic_Uno-gait-planning.gif) |
|--------|--------|
| 1) Standard gait on uneven terrain | 2) Optimal foothold selection on descrete grippable points [1] |

|![GIA-poly-hedoron.png](./docs/media/GIA-poly-hedoron.png) | ![ex4_uneven_terrain_dynamic_Non-perioidc-gait-planning.gif](./docs/media/ex4_uneven_terrain_dynamic_Non-perioidc-gait-planning.gif)|
|--------|--------|
| 3) Stability region visualization [2] | 4) Topographically salient region detection and non-periodic foothold selection [3] |

## Publication

If you use this simulator in an academic context, please cite the following publication.
> K. Uno, W. F. R. Ribeiro, Y. Koizumi, K. Haji, K. Kurihara, W. Jones, and K. Yoshida
> **"ClimbLab: MATLAB Simulation Platform for Legged Climbing Robotics"**,
> Proceedings of the 24th International Conference on Climbing and Walking Robots and the Support Technologies for Mobile Machines (CLAWAR), 2021.

    @inproceedings{uno2021climblab,
      title={ClimbLab: MATLAB Simulation Platform for Legged Climbing Robotics},
      author={Kentaro Uno and Warley F. R. Ribeiro and Yusuke Koizumi and Keigo Haji and Koki Kurihara and William Jones and Kazuya Yoshida},
      booktitle={Proceedings of the 24th International Conference on Climbing and Walking Robots and the Support Technologies for Mobile Machines (CLAWAR)},
      pages={TBD},
      year={2021}
    }

#### Requirements
We confirmed the code is working with:
* Matlab 2018b or higher
* [SpaceDyn](http://www.astro.mech.tohoku.ac.jp/spacedyn/) - a MATLAB Toolbox for Space and Mobile Robots (version 2, release 0)
  - SpaceDyn toolbox v2r0 is also installed by cloning this repository (under `lib/spacedyn_v2r0`)

#### Build and Run
* Download the files from this repository
* Open MATLAB
* Select `ClimbLab/` as the current folder for MATLAB
  - You should see `config/`, `dat/`, `docs/`, `lib/` and `src/` directories under it
* Setup the parameters as you want by changing the variable `config`.
  - Note: If you select USER configuration type, follow instructions described in the `config/USER/config_USER_param_template.m`.
* Run `src/climb_main.m` file (**Note: MATLAB might ask you to change folder or add path. Choose to "add path"**)

## References

Our recent works using ClimbLab simulator are as follows:

[1] Kentaro Uno *et al*., "[Gait Planning for a Free-Climbing Robot Based on Tumble Stability](https://ieeexplore.ieee.org/document/8700455)", Proceedings of the 2019 IEEE/SICE International Symposium on System Integration (SII), Paris, France, 2019, pp. 289-294, [doi: 10.1109/SII.2019.8700455](https://doi.org/10.1109/SII.2019.8700455).

[2] Warley F. R. Ribeiro *et al*., "[Dynamic Equilibrium of Climbing Robots based on Stability Polyhedron for Gravito-Inertial Acceleration](https://clawar.org/conference-proceedings/clawar-conference/clawar-2020-proceedings/)", Proceedings of the 23rd International Conference on Climbing and Walking Robots and the Support Technologies for Mobile Machines, Moscow, Russian Federation, 2020, pp. 297-304, [doi: 10.13180/clawar.2020.24-26.08.18](https://doi.org/10.13180/clawar.2020.24-26.08.18).

[3] Kentaro Uno *et al*., "[Non-Periodic Gait Planning Based on Salient Region Detection for a Planetary Cave Exploration Robot](https://www.hou.usra.edu/meetings/isairas2020fullpapers/pdf/5027.pdf)", Proceedings of the International Symposium on Artificial Intelligence, Robotics and Automation in Space, remote conference, 2020, No. 5027.

## FAQ

Please contact us by sending an email to the following address if there is any question or suggestion.

    srl-limb[at]grp.tohoku.ac.jp

## Release note

* Version 1.0: released on 18th Sept. 2020.
* Version 2.0: released on 26th Oct. 2020. This version includes:
    - topographically salient region detection algorithm [3]
    - non-periodic gait planner [3]
    - gripper detachment evaluation
* Version 3.0: released on 27th Aug.2021. This version includes
    - Camera sensing 
    - New robot models
    - More analysis scripts
