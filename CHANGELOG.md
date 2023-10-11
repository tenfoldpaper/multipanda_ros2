# Changelog

## Added 11.10.23
* Added franka::ControlException handling in the control loop under `robot.cpp`
* Added error recovery server for triggering automatic recovery, under `franka_error_recovery_service_server`.
* Made corresponding changes to `franka_hardware_interface` and `robot`
* Added `ErrorRecovery.srv`

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
