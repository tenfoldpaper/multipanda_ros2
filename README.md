# (WIP) ROS 2 port of franka_ros for Panda (FER) robots

This project is for porting over the various functionalities from franka_ros into ROS2 for Panda robots.
Franka Emika has dropped software support for robots older than FR3, which leaves a lot of older hardware outdated and unable to migrate to ROS2.

This repository attempts to remedy that somewhat, by bringing existing features from franka_ros over to ROS2 specifically for the Panda robots.

Still work in progress, but changelogs will be updated to indicate the current status.

The current version is forked from mcbed's port of franka_ros2 for [humble][mcbed-humble], which has a much smaller feature set than franka_ros.


## Installation Guide

(Tested on Ubuntu 22.04, ROS2 Humble, Panda 4.2.2, and Libfranka 0.9.2)

1. Build libfranka 0.9.2 from source by following the [instructions][libfranka-instructions].
2. Clone this repository into your workspace's `src` folder.
3. Source the workspace, then in your workspace root, call: `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build`
4. Add the build path to your `LD_LIBRARY_PATH`: `LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/path/to/libfranka/build"`
5. To test, source the workspace, and run `ros2 launch franka_moveit_config moveit.launch.py robot_ip:=<fci-ip>`.

## License

All packages of `panda_ros2` are licensed under the [Apache 2.0 license][apache-2.0], following `franka_ros2`.

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html

[fci-docs]: https://frankaemika.github.io/docs

[mcbed-humble]: https://github.com/mcbed/franka_ros2/tree/humble

[libfranka-instructions]: https://frankaemika.github.io/docs/installation_linux.html