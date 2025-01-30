cmake_minimum_required(VERSION 3.8)
project(franka_mj_hardware)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Default to release
set(default_build_type "Release")
# Respect the build type from user and ide
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(CHECK_TIDY "Adds clang-tidy tests" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(mujoco_ros2_control REQUIRED)
find_package(std_srvs REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Franka REQUIRED) # Franka_DIR will provide the path to the libfranka installation
find_package(Boost COMPONENTS thread chrono REQUIRED)

pluginlib_export_plugin_description_file(mujoco_ros2_control plugins.xml)

add_library(franka_mj_hardware 
  SHARED
  src/franka_mj_hardware_system.cpp
  src/robot_sim.cpp
  src/gripper_sim_action_server.cpp
  src/franka_executor.cpp
  src/control_mode.cpp
  src/helper_functions.cpp
  src/robot_sim.cpp)

target_compile_features(franka_mj_hardware PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(franka_mj_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  )

ament_target_dependencies(
  franka_mj_hardware
  "mujoco_ros2_control"
  "hardware_interface"
  "rclcpp"
  "rclcpp_components"
  "rclcpp_lifecycle"
  "rclcpp_action"
  "pluginlib"
  Franka
  franka_msgs
  std_srvs
)

target_link_libraries(
  franka_mj_hardware
  Boost::thread
  Boost::chrono
  ${YAML_CPP_LIBRARIES}
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS franka_mj_hardware
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

ament_export_include_directories(
  include
)
ament_export_libraries(
  franka_mj_hardware
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
