#pragma once

#include <ostream>
#include <type_traits>
#include <string>
#include <vector>
#include <algorithm>
#include <iterator>
#include <mujoco/mujoco.h>

// A dump of functions that are used by both hardware and multi_hardware interfaces.
namespace franka_hardware {
  bool all_of_element_has_string(std::vector<std::string> vec, std::string content);
  
  /**
   * Checks the command mode type of the provided interface.
   * Returns 0 if size is 0,
   * 1 if it is joint,
   * 2 if it is cartesian,
   * -1 otherwise.
  */
  int check_command_mode_type(std::vector<std::string> interfaces);
  // Function for extracting ns in joint names
  std::string get_ns(std::string const& s);

  // Function for extracting joint number
  int get_joint_no(std::string const& s);
  
  void set_torque_control(const mjModel* m,int actuator_no,int flag);
  
  void set_position_servo(const mjModel* m,int actuator_no,double kp);
  
  void set_velocity_servo(const mjModel* m,int actuator_no,double kv);
}