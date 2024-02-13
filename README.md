# (WIP) ROS 2 port of franka_ros for Panda (FER) robots

This project is for porting over the various functionalities from franka_ros into ROS2 for Panda robots.
Franka Emika has dropped software support for robots older than FR3, which leaves a lot of older hardware outdated and unable to migrate to ROS2.

This repository attempts to remedy that somewhat, by bringing existing features from franka_ros over to ROS2 specifically for the Panda robots.

As of 16.11.23, almost all single-robot `franka_ros` features have been migrated, including different controller interfaces, error recovery, and runtime parameter setters. Multi-arm support is also available via `franka_multi_hardware_interface`, launched via `dual_franka_launch.py`.

The repo is still in active development, and I will try to address any missing features or bugs as soon as possible.

## Credits
The original version is forked from mcbed's port of franka_ros2 for [humble][mcbed-humble].

## Working (not thoroughly tested) features
* Single arm:
    * FrankaState broadcaster
    * All control interfaces (torque, position, velocity, Cartesian).
    * Example controllers for all interfaces
    * Controllers are swappable using rqt_controller_manager
    * Runtime franka::ControlException error recovery via `~/service_server/error_recovery`
        * Upon recovery, the previously executed control loop will be executed again, so no reloading necessary.
    * Runtime internal parameter setter services much like what is offered in the updated `franka_ros2`
* Multi arm:
    * initialization and joint state broadcaster
    * Read/write interfaces
    * FrankaState broadcaster
    * Swappable controllers
    * Error recovery and parameter setters
    * Dual joint impedance & velocity example controllers

## Known issues
* Joint position controller might cause some bad motor behaviors. Suggest using torque or velocity for now.


## Priority list
* <s>Publishing FrankaState</s>
* Completely replicate the functionalities of `franka_hw`
    * <s>Implement joint limits interface (position, velocity, effort)</s> Joint limits are not really working in ros2_control; they need to be implemented at controller level.
    * <s>Adding different joint interfaces (joint {velocity, position}</s> and cartesian {<s>velocity</s>, pose})
    * <s>Adding logic for switching to different joint interfaces</s>
* <s>Adding error recovery services</s>
    * <s>franka_ros2 crashes right away if the E-stop is pressed or a controller exception occurs.</s>
    * <s>franka_ros2 does not start if the robot is already in error mode before the node is started.</s>
* <s>Adding additional example controllers (Cartesian, joint velocity/position, etc. )</s>
* <s>Add reconfiguration service for:</s>
    * <s>Cartesian, Joint impedance</s>
    * <s>Force torque, full collision behaviors</s>
    * <s>EE, K frames</s>
    * <s>Load settings</s>
* Clean up base acceleration-dependent values in Franka State
* <s>Clean up dependency tree for packages</s>
* <s>Test it out with moveit! 2</s>
    * Implement tutorials for multi-arm moveit
    * <s>bug fixing with SRDF/URDF not finding the `base_link` defined in the dual panda URDF</s>
    * Implement quality-of-life functions for moveit
* Investigating multiple arm control
    * <s>Initialization</s>
    * <s>Reading joint states</s>
    * <s>Broadcasting franka states</s>
    * Controllers for:
        * Joint-level stuff
        * Cartesian-level stuff
    * Splitting all broadcasters into its own nodes
    * Cleaning up parametrization
* Test out gripper
* Make reusable impedance controllers with proper subscribers for general use
* Add extensive tutorials!

## Installation Guide

(Tested on Ubuntu 22.04, ROS2 Humble, Panda 4.2.2 & 4.2.1, and Libfranka 0.9.2)

1. Build libfranka 0.9.2 from source by following the [instructions][libfranka-instructions].
2. Clone this repository into your workspace's `src` folder.
3. Source the workspace, then in your workspace root, call: `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build`
4. Add the build path to your `LD_LIBRARY_PATH`: `LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/path/to/libfranka/build"`
5. To test, source the workspace, and run `ros2 launch franka_bringup franka.launch.py robot_ip:=<fci-ip>`.

## License

All packages of `panda_ros2` are licensed under the [Apache 2.0 license][apache-2.0], following `franka_ros2`.

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html

[fci-docs]: https://frankaemika.github.io/docs

[mcbed-humble]: https://github.com/mcbed/franka_ros2/tree/humble

[libfranka-instructions]: https://frankaemika.github.io/docs/installation_linux.html