# Changelog

## Added 28.11.23
* Added dual panda arm moveit configurations and launch files. Still some bugs remaining, but plan & execute works.

## Added 23.11.23
* Implemented control loop re-running upon error recovery

## Added 16.11.23
* Implemented the error recovery fix from 15.11.23 to multi-arm

## Added 15.11.23
* Fixed the issue where error recovery was not working at all. This is a monkey patch; it does not use `ros2_control`'s error return type handling at all.

## Added 18.10.23
* Added working `dual_joint_velocity_example_controller`

## Added 17.10.23
* Added `write` interface
* Fixed multi-arm hardware interface to work with Franka state broadcaster
* Parametrized `franka_robot_state`'s `robot_name_`
* Added working `dual_joint_impedance_example_controller`

## Added 16.10.23
* Added basic multi-arm hardware interface. Currently, initialization and `read()` function work. 

## Added 12.10.23
* Added param setter services to allow run-time changing of stiffness, collision behavior, load, frames, etc.

## Added 11.10.23
* Added franka::ControlException handling in the control loop under `robot.cpp`
* Added error recovery server for triggering automatic recovery, under `franka_error_recovery_service_server`.
* Made corresponding changes to `franka_hardware_interface` and `robot`
* Added `ErrorRecovery.srv`
* Overhauled `prepare_command_mode_switch` to accommodate Cartesian interfaces with error checks
* Added Cartesian velocity interface and a corresponding example controller

## Added 10.10.23
* Added additional controller interfaces: joint position, joint velocity
* Added example controllers for the new interfaces
* Confirmed that switching between controllers at runtime with rqt_controller_manager woks.
## Added 06.10.23
* Added `franka_semantic_components` adapted for panda
* Updated `FrankaState.msg` to fit the ROS2 requirements
* Added `Errors.msg`
* Added working FrankaState broadcaster, which is included in `franka.launch.py`

## Added 05.10.23
* Added `model_base.hpp` and `model.hpp`; copy-pasted mostly from `franka_ros`
* Extended `robot.xpp` to have a pointer to the robot's model
* Added `FrankaState` to `franka_hardware_interface`; not yet exposed as a published topic.

## [Old from Franka]
### Added
* CI tests in Jenkins
* joint\_effort\_trajectory\_controller package that contains a version of the
 joint\_trajectory\_controller that can use the torque interface.
 [See this PR](https://github.com/ros-controls/ros2_controllers/pull/225)
* franka\_bringup package that contains various launch files to start controller examples or Moveit2
* franka\_moveit\_config package that contains a minimal moveit config to control the robot
* franka\_example\_controllers package that contains some example controllers to use
* franka\_hardware package that contains a plugin to access the robot
* franka\_msgs package that contains common message, service and action type definitions
* franka\_description package that contains all meshes and xacro files
* franka\_gripper package that offers action and service interfaces to use the Franka Hand gripper
