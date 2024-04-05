cmake_minimum_required(VERSION 3.5)
project(franka_hardware)

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
find_package(std_srvs REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Franka REQUIRED) # Franka_DIR will provide the path to the libfranka installation
find_package(Boost COMPONENTS thread chrono REQUIRED)

# Mujoco dependencies
set(OpenGL_GL_PREFERENCE GLVND)
set(Mujoco_DIR ~/Libraries/mujoco_build/lib/cmake)
find_package(glfw3 3.3 REQUIRED)     # GLFW3 here,
find_package(mujoco REQUIRED PATHS ${Mujoco_DIR} NO_DEFAULT_PATH)
find_package(Eigen3 REQUIRED)
add_library(franka_hardware
        SHARED
        src/franka_hardware_interface.cpp
        src/franka_multi_hardware_interface.cpp
        src/franka_mujoco_hardware_interface.cpp
        src/robot.cpp
        src/robot_sim.cpp
        src/franka_param_service_server.cpp
        src/franka_error_recovery_service_server.cpp
        src/gripper_sim_action_server.cpp
        src/franka_executor.cpp
        src/control_mode.cpp
        src/helper_functions.cpp
)

target_include_directories(
        franka_hardware
        PRIVATE
        include
        ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(
        franka_hardware
        hardware_interface
        Franka
        pluginlib
        rclcpp
        rclcpp_action
        rclcpp_components
        franka_msgs
        std_srvs
)

target_link_libraries(
        franka_hardware
        mujoco::mujoco
        glfw
        Boost::thread
        Boost::chrono

)

pluginlib_export_plugin_description_file(hardware_interface franka_hardware.xml)

install(
        TARGETS franka_hardware
        DESTINATION lib
)
install(
        DIRECTORY include/
        DESTINATION include
)

if(BUILD_TESTING)
    find_package(ament_cmake_clang_format REQUIRED)
    find_package(ament_cmake_copyright REQUIRED)
    find_package(ament_cmake_cppcheck REQUIRED)
    find_package(ament_cmake_flake8 REQUIRED)
    find_package(ament_cmake_lint_cmake REQUIRED)
    find_package(ament_cmake_pep257 REQUIRED)
    find_package(ament_cmake_xmllint REQUIRED)

    set(CPP_DIRECTORIES src include)
    ament_clang_format(CONFIG_FILE ../.clang-format ${CPP_DIRECTORIES})
    ament_copyright(${CPP_DIRECTORIES} package.xml)
    ament_cppcheck(${CPP_DIRECTORIES})
    ament_lint_cmake(CMakeLists.txt)
    ament_flake8()
    ament_pep257()
    ament_xmllint()
    if(CHECK_TIDY)
        find_package(ament_cmake_clang_tidy REQUIRED)
        set(ament_cmake_clang_tidy_CONFIG_FILE ../.clang-tidy)
        ament_clang_tidy(${CMAKE_BINARY_DIR})
    endif()
endif()

ament_export_include_directories(
        include
)
ament_export_libraries(
        franka_hardware
)
ament_export_dependencies(
        hardware_interface
        pluginlib
        rclcpp
)
ament_package()
