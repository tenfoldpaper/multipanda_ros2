# multipanda_ros2
## A sim- and real Panda robot integration based on the `ros2_control` framework

This project implements most (and eventually all) features from the original `franka_ros` repository in ROS2 Humble, specifically for the Franka Emika Robot (Panda).
This project significantly expands upon the original `franka_ros2` from the company, who dropped the support for the Pandas.

Additionally, a one-arm mujoco simulation has been integrated, meaning you can now run the same controller on both simulated and real robots. 

As of 24.04.05, the real robot interfaces are more or less complete, though more extensive testing is still needed.

It is still under active development, with the current target feature being the multi-arm mujoco simulation integration.

## Documentation

Documentation for this project is available [here](./docs/main.md).

## Working (not thoroughly tested) features
* Real robot:
    * FrankaState broadcaster
    * All control interfaces (torque, position, velocity, Cartesian).
    * Example controllers for all interfaces
    * Controllers are swappable using rqt_controller_manager
    * Runtime franka::ControlException error recovery via `~/service_server/error_recovery`
        * Upon recovery, the previously executed control loop will be executed again, so no reloading necessary.
    * Runtime internal parameter setter services much like what is offered in the updated `franka_ros2`
* Sim robot:
    * Same as the real robot, except Cartesian command interface is not available, and there is no plan to implement this for now.
    * Gripper server with identical interface to the real gripper (i.e. action servers)
    * Example controllers for the real single-arm listed above, that correspond to those interfaces, work out of the box.
    * FrankaState implements the basics: torque, joint position/velocity, `O_T_EE` and `O_F_ext_hat`.
    * Model provides all the existing functions: `pose`, `zeroJacobian`, `bodyJacobian`, `mass`, `gravity`, `coriolis`.
        * Gravity for now just returns the corresponding `qfrc_gravcomp` force from mujoco.
        * Coriolis = `qfrc_bias - qfrc_gravcomp`

## Known issues
* Joint position controller might cause some bad motor behaviors. Suggest using torque or velocity for now.
* The default `franka_moveit_config` package depends on `warehouse_ros_mongo`, which has been deprecated. It has been changed to `warehouse_ros_sqlite` for now to ensure that `rosdep install` works properly. For now, please refer to this discussion [here](https://discourse.ros.org/t/fixing-moveit2-humble-moveit-ros-benchmarks-package/32048) for a potential solution; once a proper solution has been identified, it will be added.

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
            
## Credits
The original version is forked from mcbed's port of franka_ros2 for [humble][mcbed-humble].

## License

All packages of `panda_ros2` are licensed under the [Apache 2.0 license][apache-2.0], following `franka_ros2`.

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html
[fci-docs]: https://frankaemika.github.io/docs
[mcbed-humble]: https://github.com/mcbed/franka_ros2/tree/humble
[libfranka-instructions]: https://frankaemika.github.io/docs/installation_linux.html
[mujoco-instructions]: https://mujoco.readthedocs.io/en/latest/programming/#building-mujoco-from-source
[humble-instructions]: https://docs.ros.org/en/humble/Installation.html