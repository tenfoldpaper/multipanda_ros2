cmake_minimum_required(VERSION 3.8)
project(franka_gripper_custom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(Franka REQUIRED)  

# --- Executable node ---
add_executable(franka_gripper_control src/franka_gripper_control.cpp)

target_include_directories(franka_gripper_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(franka_gripper_control
  rclcpp
  sensor_msgs
  std_msgs
  control_msgs
  franka_msgs
  Franka)

target_link_libraries(franka_gripper_control Franka::Franka)

install(TARGETS franka_gripper_control
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
# )

ament_package()

