cmake_minimum_required(VERSION 3.8)
project(panda_motion_generators)

set(default_build_type "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  rclcpp_action
  sensor_msgs
  std_srvs
  std_msgs
  panda_motion_generator_msgs
  multi_mode_control_msgs
  multi_mode_controller
  Eigen3
  tf2
  tf2_ros
  geometry_msgs
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Base library
add_library(${PROJECT_NAME} SHARED
  src/panda_poly_c2_joint_motion_generator.cpp
  src/panda_dq_c1_cartesian_motion_generator.cpp
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(panda_motion_generators PUBLIC
  dqrobotics
)

target_include_directories(
        ${PROJECT_NAME}
        PUBLIC
        include
        ${EIGEN3_INCLUDE_DIRS}
)
# Cartesian motion generator
add_executable(panda_dq_c1_cartesian_motion_generator_node
  src/panda_dq_c1_cartesian_motion_generator_node.cpp
)

target_include_directories(panda_dq_c1_cartesian_motion_generator_node
  PUBLIC
  include
)

target_link_libraries(panda_dq_c1_cartesian_motion_generator_node
  panda_motion_generators
)

ament_target_dependencies(panda_dq_c1_cartesian_motion_generator_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Joint motion generator
add_executable(panda_poly_c2_joint_motion_generator_node
  src/panda_poly_c2_joint_motion_generator_node.cpp
)

target_include_directories(panda_poly_c2_joint_motion_generator_node
  PUBLIC
  include
)

target_link_libraries(panda_poly_c2_joint_motion_generator_node
  panda_motion_generators
)

ament_target_dependencies(panda_poly_c2_joint_motion_generator_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Joint motion generator testing client
add_executable(panda_poly_c2_joint_motion_generator_client
  src/panda_poly_c2_joint_motion_generator_client.cpp)
ament_target_dependencies(panda_poly_c2_joint_motion_generator_client
  "rclcpp"
  "rclcpp_action"
  "panda_motion_generator_msgs"
)

# Dirty client to set the robot to the initial pose as defined by communication_test from libfranka
add_executable(panda_to_init_pose_client
  src/panda_to_init_pose_client.cpp)
ament_target_dependencies(panda_to_init_pose_client
  "rclcpp"
  "rclcpp_action"
  "panda_motion_generator_msgs"
  "multi_mode_control_msgs"
)

# Cartesian motion generator testing client
add_executable(panda_dq_c1_cartesian_motion_generator_client
  src/panda_dq_c1_cartesian_motion_generator_client.cpp)
ament_target_dependencies(panda_dq_c1_cartesian_motion_generator_client
  "rclcpp"
  "rclcpp_action"
  "panda_motion_generator_msgs"
  "tf2"
  "tf2_ros"
  "geometry_msgs"
)

# Installation
install(TARGETS 
  ${PROJECT_NAME}
  DESTINATION lib
)

# The nodes need to be installed separately in the ${PROJECT_NAME} folder
# for ros2 run to find the executable
install(TARGETS 
  panda_poly_c2_joint_motion_generator_node
  panda_dq_c1_cartesian_motion_generator_node
  panda_poly_c2_joint_motion_generator_client
  panda_dq_c1_cartesian_motion_generator_client
  panda_to_init_pose_client
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)


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
