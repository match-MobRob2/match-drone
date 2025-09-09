# Installation
Follow these steps to set up the project locally:


## Install ros-gz (ROS Jazzy)

  > [!TIP]
  > The `ros_gz` library makes heavy use of templates which causes compilers to consume a lot of memory. If your build fails with `c++: fatal error: Killed signal terminated program cc1plus`
  > try building with `colcon build --parallel-workers=1 --executor sequential`. You might also have to set `export MAKEFLAGS="-j 1"` before running `colcon build` to limit
  > the number of processors used to build a single package.


## Documentation
The fast-lio2 SLAM algorithm originates from the following paper:
```
@article{DBLP:journals/corr/abs-2107-06829,
  author       = {Wei Xu and
                  Yixi Cai and
                  Dongjiao He and
                  Jiarong Lin and
                  Fu Zhang},
  title        = {{FAST-LIO2:} Fast Direct LiDAR-inertial Odometry},
  journal      = {CoRR},
  volume       = {abs/2107.06829},
  year         = {2021},
  url          = {https://arxiv.org/abs/2107.06829},
  eprinttype    = {arXiv},
  eprint       = {2107.06829},
  timestamp    = {Fri, 21 Jun 2024 12:54:52 +0200},
  biburl       = {https://dblp.org/rec/journals/corr/abs-2107-06829.bib},
  bibsource    = {dblp computer science bibliography, https://dblp.org}
}
```