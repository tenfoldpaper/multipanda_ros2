cmake_minimum_required(VERSION 3.5)
project(franka_robot_state_broadcaster)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(CHECK_TIDY "Adds clang-tidy tests" OFF)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    builtin_interfaces
    control_msgs
    controller_interface
    controller_manager
    franka_msgs
    franka_semantic_components
    generate_parameter_library
    pluginlib
    rclcpp_lifecycle
    rcutils
    realtime_tools
    sensor_msgs)

find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(franka_robot_state_broadcaster
        SHARED
        src/franka_robot_state_broadcaster.cpp
        src/franka_robot_model_broadcaster.cpp)

target_include_directories(
  franka_robot_state_broadcaster
        PRIVATE
        include
)


generate_parameter_library(franka_robot_state_broadcaster_parameters src/franka_robot_state_broadcaster_parameters.yaml)

target_link_libraries(franka_robot_state_broadcaster
                      PUBLIC franka_robot_state_broadcaster_parameters)

ament_target_dependencies(franka_robot_state_broadcaster PUBLIC
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
  
pluginlib_export_plugin_description_file(controller_interface franka_robot_state_broadcaster.xml)

install(
        TARGETS franka_robot_state_broadcaster
        DESTINATION lib
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

ament_export_include_directories(
        include
)
ament_export_libraries(
        franka_robot_state_broadcaster
)
ament_export_dependencies(
        hardware_interface
        pluginlib
        rclcpp
)
ament_package()
