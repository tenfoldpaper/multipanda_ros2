cmake_minimum_required(VERSION 3.5)
project(franka_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Grasp.action"
  "action/Homing.action"
  "action/Move.action"
  "srv/ErrorRecovery.srv"
  "srv/SetCartesianStiffness.srv"
  "srv/SetTCPFrame.srv"
  "srv/SetForceTorqueCollisionBehavior.srv"
  "srv/SetFullCollisionBehavior.srv"
  "srv/SetJointStiffness.srv"
  "srv/SetStiffnessFrame.srv"
  "srv/SetLoad.srv"
  "srv/SetMujocoPoses.srv"
  "msg/GraspEpsilon.msg"
  "msg/FrankaState.msg"
  "msg/FrankaModel.msg"
  "msg/Errors.msg"
  "msg/PoseStampedArray.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()