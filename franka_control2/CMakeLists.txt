cmake_minimum_required(VERSION 3.8)
project(franka_control2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Franka REQUIRED) # Franka_DIR will provide the path to the libfranka installation

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_index_cpp
  controller_interface
  controller_manager_msgs
  hardware_interface
  pluginlib
  rclcpp
  realtime_tools
  Franka
  franka_msgs
  std_msgs
  controller_manager
)

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_cmake_core REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(franka_control2_node src/franka_control2_node.cpp)
ament_target_dependencies(franka_control2_node 
  controller_manager
  Franka
  franka_msgs
  realtime_tools)
  
target_include_directories(franka_control2_node
  PRIVATE
  include
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

install(
  TARGETS franka_control2_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
