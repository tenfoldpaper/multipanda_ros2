cmake_minimum_required(VERSION 3.8)
project(multi_mode_control_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CartesianImpedanceGoal.msg"
  "msg/DualCartesianImpedanceGoal.msg"
  "msg/ContactThresholds.msg"
  "msg/Controller.msg"
  "msg/JointArray.msg"
  "msg/JointGoal.msg"
  "msg/VelocityGoal.msg"
  "msg/Wrench.msg"
  "srv/GetControllers.srv"
  "srv/GetRobotStates.srv"
  "srv/SetCartesianImpedance.srv"
  "srv/SetControllers.srv"
  "srv/SetJointImpedance.srv"
  "srv/SetRecoveryMode.srv"
  "srv/SetReflex.srv"
  "srv/SetThresholds.srv"
  "srv/SetWorldState.srv"
  DEPENDENCIES geometry_msgs franka_msgs
)



# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
