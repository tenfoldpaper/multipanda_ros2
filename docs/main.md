# Documentation for multipanda_ros2
`multipanda_ros2` is a `ros2_control`-based framework for the popular, now unsupported, Franka Emika Robot (Panda).
It is built on ROS2 Humble, and as a consequence, Ubuntu 22.04. Support for newer versions will be released as they come.
It is capable of controlling an arbitrary number of locally connected FCI-enabled robots accessible via their IP addresses.
The primary features of the framework include:
- ROS2 compatibility (naturally)
- 1khz access to robot data, i.e. the state and model
- All control modes offered by `libfranka`, i.e. torque, joint position/velocity, Cartesian position/velocity (only for real robots), regardless of how many robots are being controlled
- Franka Hand gripper integration
- Integrated MuJoCo simulation

In each of the following sections, in-depth details about the various parts of the framework are introduced. The aim of the documentation is to make the package more understandable and easily customizable for your own uses.

## Installation guide
(Tested on Ubuntu 22.04, ROS2 Humble, Panda 4.2.2 & 4.2.1, `libfranka` 0.9.2 and MuJoCo 3.1.3)
On a computer running Ubuntu 22.04 and real-time kernel (if you wish to use it with a real robot), do the following:
1. Build `libfranka` 0.9.2 from source by following the [instructions][libfranka-instructions].
2. Build MuJoCo from source by following the [instructions][mujoco-instructions] (tested with 3.1.3).
3. Install library dependencies:
    - Install GLFW with `sudo apt-get install libglfw3;sudo apt-get install libglfw3-dev`
    - Install [Eigen **3.3.9**](https://gitlab.com/libeigen/eigen/-/releases/3.3.9). Remove Eigen 3.4.0 if that was installed from following the `libfranka` steps.
        - Some functions will break if you use Eigen 3.4.0, and will fail to compile.
    - Install [`dq-robotics`](https://dqrobotics.github.io/) C++ version
4. Install ROS2 Humble by following their [instructions][humble-instructions] and create a workspace.
5. Clone this repository (i.e. the multipanda) into your workspace's `src` folder.
6. Install the dependencies by running this rosdep command from the workspace root: `rosdep install --from-paths src -y --ignore-src`
7. Adjust the path to mujoco in `franka_hardware` package's `CMakeLists.txt`.
8. Source the workspace, then in your workspace root, call: `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build`
9. Add the build path to your `LD_LIBRARY_PATH` by adding the following line to your `~/.bashrc`: `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:{path to libfranka}/build:{path to mujoco's install}/lib`
    - The MuJoCo path is the INSTALLED folder's directory. The `lib` folder should only have the two `.so` files, and a folder called `cmake`.
    - Likewise, the `libfranka` path should contain the `.so` files.
10. To run:
    - Single arm:
        1. with real robot, source the workspace, and run:
            - Default: `ros2 launch franka_bringup franka.launch.py robot_ip:=<fci-ip>`.
            - Multi-mode: `ros2 launch franka_bringup multimode_franka.launch.py robot_ip:=<fci-ip>`.
            - `arm_id` is fixed to `panda`.
        2. with sim robot, source the workspace, and run:
            - Default: `ros2 launch franka_bringup franka_sim.launch.py`.
            - Multi-mode: `ros2 launch franka_bringup multimode_sim.launch.py`.
            - `arm_id` is fixed to `panda`.
            - Currently, only the robot with gripper attachment is available.
    - Dual arm:
        1. with real robot, source the workspace, and run:
            - Default: `ros2 launch franka_bringup dual_franka.launch.py robot_ip_1:=<robot-1-fci-ip> robot_ip_2:=<robot-2-fci-ip> arm_id_1=<robot-1-name> arm_id_2=<robot-2-name>`.
            - Multi-mode: `ros2 launch franka_bringup dual_multimode_franka.launch.py robot_ip_1:=<robot-1-fci-ip> robot_ip_2:=<robot-2-fci-ip> arm_id_1=<robot-1-name> arm_id_2=<robot-2-name>`.
            - There is no default `arm_id`.
            - Grippers are set to true by default; add `hand_n=false` to disable this.
        2. with sim robot, source the workspace, and run:
            - Default: `ros2 launch franka_bringup dual_franka_sim.launch.py`.
            - Multi-mode: `ros2 launch franka_bringup dual_multimode_sim.launch.py`
            - `arm_id_1=mj_left` and `arm_id_2=mj_right` by default.

## In-depth documentation

### [franka_bringup][bringup]
The `franka_bringup` package contains all the launch files that are used to start the framework.

### [franka_description][description]
The  `franka_description` package is where the robot URDF, `ros2-control` configuration, and MuJoCo `xml` files are stored.

### [franka_hardware][hardware]
The `franka_hardware` package is the core of `multipanda_ros2`. This is where the bulk of the code for interfacing with both the real and simulated robots is. Information on how to use the default Franka Hand grippers is also described here.

### [multimode_controller][mmc]
The `multimode_controller` is a standard `ros2-control` controller, but uses its own controller implementation format, called _controllets_. It is designed to reduce the amount of programming overhead, and to allow extremely fast switching between different controllers within the same control mode.

## Other packages

### franka_control2
A `ros2-control` node copied from the original repo. All the launch files execute this particular version, instead of the one installed using `apt install`, for better transparency.

### franka_example_controllers
A series of example controllers for both one-arm and two-arm cases, some written by Franka Emika, some by me. It is intended to be used as a starting point for writing a new controller.
Please see the example controllers (I recommend `cartesian_impedance_example_controller.cpp/hpp`) on how to structure your own one.
Generally, you should keep in mind the following checklist:
- Make sure to export your controller as a plugin at the end of the `.cpp` file, like:
``` cpp
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceExampleController,
                       controller_interface::ControllerInterface)
```
- Add the controller to `CMakeLists.txt` under `add_library` section,
- Update the `franka_example_controllers.xml` file with the new controller so that your controller can be found,
- Add your new controller (make sure the names all match) to the `.yaml` config file in `franka_bringup`.

You can of course create your own controller package by following this structure.

### franka_gripper
Franka Hand action server written by Franka Emika, copied from their repository.

### franka_moveit_config
A package that contains the `moveit` configuration files, along with example launch files.

### franka_msgs
All custom messages/services/actions are included here, except for the ones used by the multi-mode controller.

### franka_robot_state_broadcaster
This implements two broadcasters that publishes the robot model and state data at a lower frequency as a ROS2 topic. It interfaces with `franka_semantic_components`.

### franka_semantic_components
A `SemanticComponentInterface` wrapper for processing the robot model and state data that is updated at 1khz from the main `franka_hardware` package. This provides the interfaces for those data that you can use in your `ros2-control` controllers.



[bringup]: ./chapters/franka_bringup.md
[description]: ./chapters/franka_description.md
[hardware]: ./chapters/franka_hardware.md
[mmc]: ./chapters/franka_multi_mode_controller.md

[libfranka-instructions]: https://frankaemika.github.io/docs/installation_linux.html
[mujoco-instructions]: https://mujoco.readthedocs.io/en/latest/programming/#building-mujoco-from-source
[humble-instructions]: https://docs.ros.org/en/humble/Installation.html