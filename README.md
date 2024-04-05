# multipanda_ros2
## A sim- and real Panda robot integration based on the `ros2_control` framework

This project implements most (and eventually all) features from the original `franka_ros` repository in ROS2 Humble, specifically for the Franka Emika Robot (Panda).
This project significantly expands upon the original `franka_ros2` from the company, who dropped the support for the Pandas.

Additionally, a one-arm mujoco simulation has been integrated, meaning you can now run the same controller on both simulated and real robots. 

As of 24.04.05, the real robot interfaces are more or less complete, though more extensive testing is still needed.

It is still under active development, with the current target feature being the multi-arm mujoco simulation integration.

Extensive tutorials and documentations will come soon.


## Working (not thoroughly tested) features
* Single arm:
    * FrankaState broadcaster
    * All control interfaces (torque, position, velocity, Cartesian).
    * Example controllers for all interfaces
    * Controllers are swappable using rqt_controller_manager
    * Runtime franka::ControlException error recovery via `~/service_server/error_recovery`
        * Upon recovery, the previously executed control loop will be executed again, so no reloading necessary.
    * Runtime internal parameter setter services much like what is offered in the updated `franka_ros2`
* MuJoCo single arm:
    * Torque and joint velocity interfaces
    * Gripper server with identical interface to the real gripper (i.e. action servers)
    * Example controllers for the real single-arm listed above, that correspond to those interfaces, work out of the box.
    * FrankaState implements the basics: torque, joint position/velocity, `O_T_EE` and `O_F_ext_hat`.
    * Model provides all the existing functions: `pose`, `zeroJacobian`, `bodyJacobian`, `mass`, `gravity`, `coriolis`.
        * Gravity for now just returns the corresponding `qfrc_gravcomp` force from mujoco.
        * Coriolis = `qfrc_bias - qfrc_gravcomp`
* Multi arm:
    * All the features of the single arm, minus the Cartesian interfaces.
    * multimode controller; documentation coming soon.

## Known issues
* Joint position controller might cause some bad motor behaviors. Suggest using torque or velocity for now.

## Installation Guide

(Tested on Ubuntu 22.04, ROS2 Humble, Panda 4.2.2 & 4.2.1, and Libfranka 0.9.2)

1. Build libfranka 0.9.2 from source by following the [instructions][libfranka-instructions].
2. Build mujoco from source by following the [instructions][mujoco-instructions] (tested with 3.1.3).
3. Clone this repository (i.e. the multipanda) into your workspace's `src` folder.
4. Adjust the path to GLFW and mujoco in `franka_hardware` package's `CMakeLists.txt`.
5. Source the workspace, then in your workspace root, call: `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build`
6. Add the build path to your `LD_LIBRARY_PATH`: `LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/path/to/libfranka/build:/path/to/mujoco/build/lib"`
7. To run:
    1. with real robot, source the workspace, and run `ros2 launch franka_bringup franka.launch.py robot_ip:=<fci-ip>`.
    2. with sim robot, source the workspace, and run `ros2 launch franka_bringup franka_sim.launch.py`.

## Credits
The original version is forked from mcbed's port of franka_ros2 for [humble][mcbed-humble].

## License

All packages of `panda_ros2` are licensed under the [Apache 2.0 license][apache-2.0], following `franka_ros2`.

[apache-2.0]: https://www.apache.org/licenses/LICENSE-2.0.html

[fci-docs]: https://frankaemika.github.io/docs

[mcbed-humble]: https://github.com/mcbed/franka_ros2/tree/humble

[libfranka-instructions]: https://frankaemika.github.io/docs/installation_linux.html

[mujoco-instructions]: https://mujoco.readthedocs.io/en/latest/programming/#building-mujoco-from-source
