cmake_minimum_required(VERSION 3.8)
project(multi_mode_controller)

set(default_build_type "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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

set(THIS_PACKAGE_INCLUDE_DEPENDS
builtin_interfaces
control_msgs
multi_mode_control_msgs
controller_interface
controller_manager
franka_semantic_components
generate_parameter_library
pluginlib
rclcpp
rclcpp_lifecycle
rcutils
realtime_tools
sensor_msgs
Franka
)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


add_library(openGJK STATIC
    src/openGJK.c
)
target_include_directories(
  openGJK
  PUBLIC
  include
)

install(TARGETS
  openGJK
  DESTINATION lib
)

add_library(multi_mode_controller SHARED 
  src/robot_data.cpp
  src/controller_factory.cpp
  src/redundancy_resolution.cpp
  src/comless_panda_cartesian_impedance_controller.cpp
  src/comless_dual_cartesian_impedance_controller.cpp
  src/comless_coupled_dual_cartesian_impedance_controller.cpp
  src/comless_panda_joint_impedance_controller.cpp
  src/comless_dual_joint_impedance_controller.cpp
  src/panda_cartesian_impedance_controller.cpp
  src/dual_cartesian_impedance_controller.cpp
  src/twin_cartesian_impedance_controller.cpp
  src/des_coupled_dual_cartesian_impedance_controller.cpp
  src/panda_joint_impedance_controller.cpp
  src/panda_joint_velocity_controller.cpp
  src/twin_joint_impedance_controller.cpp)

target_include_directories(
  multi_mode_controller
    PRIVATE
    include
    ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(multi_mode_controller PUBLIC
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(multi_mode_controller PUBLIC 
                      openGJK)

install(
        TARGETS multi_mode_controller
        DESTINATION lib
)

install(
        DIRECTORY include/
        DESTINATION include
)

add_library(synchronous_client SHARED src/synchronous_client.cpp)
ament_target_dependencies(synchronous_client PUBLIC rclcpp ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(
  synchronous_client
   PRIVATE
   include
   ${EIGEN3_INCLUDE_DIRS}
)
install(TARGETS
  synchronous_client
  DESTINATION lib
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
        multi_mode_controller
        synchronous_client
        openGJK
)
ament_export_dependencies(
        franka_semantic_components
        rclcpp
)
ament_package()
