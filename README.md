<p align="center">
  <img src="resources/header.gif" alt="animated" />
</p>

# Overview

*strelka* is an open source modular library for quadruped locomotion, designed to provide researchers and developers with a flexible and extensible framework for implementing and testing a range of quadrupedal locomotion algorithms and controllers. *strelka* provides a seamless integration of planning, control, estimation, and communication modules, enabling users to develop and test complex locomotion strategies in simulation and on real hardware platforms.

The key feature of *strelka* is its modular design, which allows users to easily swap out and customize individual components of the locomotion pipeline. This means that users can quickly experiment with different control algorithms, sensors, and other hardware components, without having to rewrite large portions of the codebase. *strelka* also provides a decent set of pre-built controllers and locomotion primitives that users can leverage to quickly prototype and test new ideas.

It is **not** based on Robot Operating System (ROS). Instead it uses [Lightweight Communications and Marshalling (LCM) library](https://github.com/lcm-proj/lcm) and [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page). The reason for that is the fact that majority of the control stack does not actually require ROS dependencies. So deployment speed on the real robot becomes much faster.

To aid in development and testing, [*strelka_ros*](https://github.com/RumblingTurtle/strelka_ros) includes a range of visualization and data-processing tools. These tools make it easy to visualize the robot's motion and sensor data, as well as to monitor the performance of the various control and planning algorithms. [*strelka_ros*](https://github.com/RumblingTurtle/strelka_ros) also includes support for the Gazebo simulation environment, allowing users to easily test and validate their locomotion algorithms in a realistic simulated environment.

Overall, *strelka* is an ideal framework for researchers and developers who are interested in experimenting with quadrupedal locomotion, and who are looking for a flexible and extensible software platform that can be easily customized to meet their specific needs.

## YouTube Demo
<a href="http://www.youtube.com/watch?feature=player_embedded&v=NjJO0jSLSFc" target="_blank" >
 <img src="http://img.youtube.com/vi/NjJO0jSLSFc/mqdefault.jpg" alt="Watch the demo" width="240" height="180" border="10"/>
</a>

## [Architechture overview](architecture.md)

## [Implementing your robot](defining_new_robots.md)

# Requirements
- Ubuntu (=18.04,20.04)
- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) (tested and built on 3.3.7)
- [LCM](https://github.com/lcm-proj/lcm) 
# Installation
## Build using CMake
```
# Assuming you cloned the repo to ~/
mkdir ~/strelka/build
cd ~/strelka/build
cmake -DCMAKE_BUILD_TYPE=Release ..
sudo make install
```
*Building the library in release mode is important to keep MPC's update rate high enough
## Uninstall
```
cd ~/strelka/build
sudo make uninstall
```

## Run tests
```
cd build && ctest --vv
```

# Examples: Unitree A1
Real robot would require both [robot_low_command.lcm](strelka_messages/lcm/robot_low_command.lcm) and [robot_raw_state.lcm](strelka_messages/lcm/robot_raw_state.lcm) publishers. [*strelka_ros*](https://github.com/RumblingTurtle/strelka_ros) provides full simulation support.
```
~/strelka/build/examples/lcm_high_command_publisher
~/strelka/build/examples/a1_state_estimator
~/strelka/build/examples/a1_wbic
~/strelka/build/examples/a1_local_planner
```
Also make sure to check out [*strelka_ros*](https://github.com/RumblingTurtle/strelka_ros) for use of ROS and perception information with strelka library. 

#  LCM message profiling
[lcm-mon](https://github.com/trehansiddharth/lcm-mon) repo can be used to debug LCM messages
```
lcm-mon -t ~/strelka/strelka_messages/lcm
```