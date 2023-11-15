cmake_minimum_required(VERSION 3.5)
project(franka_semantic_components)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(CHECK_TIDY "Adds clang-tidy tests" OFF)

set(THIS_PACKAGE_INCLUDE_DEPENDS Franka franka_hardware franka_msgs hardware_interface controller_interface rclcpp_lifecycle)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(Franka REQUIRED)
# add src/franka_robot_state.cpp to add_library after
add_library(franka_semantic_components SHARED src/franka_robot_state.cpp src/franka_robot_model.cpp)
target_include_directories(franka_semantic_components PRIVATE include)
target_link_libraries(franka_semantic_components Franka::Franka)
ament_target_dependencies(franka_semantic_components
                          Franka
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS franka_semantic_components DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)













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

ament_export_include_directories(include)
ament_export_libraries(franka_semantic_components)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
