# climbing simulator
Matlab climbing and walking simulation platform for limbed robotics

Author(s) and maintainer(s): [Space Robotics Lab.](http://www.astro.mech.tohoku.ac.jp/e/index.html) Climbing Robotics Team

![logo.png](.\docs\media\logo.png)![ex1_uneven_dynamic_fixed_stride.gif](https://github.com/Space-Robotics-Laboratory/climbing_simulator/docs\media\ex1_uneven_dynamic_fixed_stride.gif)
![ex2_flat_kinematic_Uno-gait-planning.gif](.\docs\media\ex2_flat_kinematic_Uno-gait-planning.gif) ![ex3_uneven_dynamic_Uno-gait-planning_stability_polyhedron.gif](.\docs\media\ex3_uneven_dynamic_Uno-gait-planning_stability_polyhedron.gif)

climbing simulator is ...

The source code is released under a BSD 3-Clause license.

## Usage

#### Requirements
We confirmed the code is working with the following function.
- Matlab 2020
- [SpaceDyn](http://www.astro.mech.tohoku.ac.jp/spacedyn/) [1] - a MATLAB Toolbox for Space and Mobile Robots (>= version 2, release 0)
- SpaceDyn toolbox v2r0 is also installed by cloning this repository (under `lib/spacedyn_v2r0`)

#### Build and Run
- Download the files from this repository.
- Open MATLAB
- Select `climbing_simulator/` as the current folder for MATLAB
  * You should see `dat/`, `lib/` and `src/` directories under it
- Setup the parameters as you want
- Run `climb_main.m` file (**Note: MATLAB might ask you to change folder or add path. Choose to "add path"**)

## References
[1] K. Nagaoka et. al., "[Passive Spine Gripper for Free-Climbing Robot in Extreme Terrain](https://ieeexplore.ieee.org/document/8260908)", IEEE Robotics and Automation Letters, Vol. 3, No. 3, pp. 1765--1770, 2018.

[2] 蓑手勇人　他， "[岩石把持グリッパを用いた４足歩行ロボットの開発](https://www.jstage.jst.go.jp/article/jsmermd/2017/0/2017_2P2-B12/_article/-char/ja/)", ロボティクス・メカトロニクス講演会講演概要集2017, pp. 2P2-B12, 2017.

## FAQ
