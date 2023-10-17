#include <franka_hardware/helper_functions.hpp>
// A dump of functions that are used by both hardware and multi_hardware interfaces.
namespace franka_hardware {
bool all_of_element_has_string(std::vector<std::string> vec, std::string content){
  if(vec.size() == 0){
    return false;
  }
  return std::all_of(
        vec.begin(), vec.end(),
        [&](std::string elem) { return elem.find(content) != std::string::npos; });
  
}
int check_command_mode_type(std::vector<std::string> interfaces){
  if(interfaces.size() != 0){
    bool is_cartesian = all_of_element_has_string(interfaces, "ee_cartesian");
    bool is_joint = all_of_element_has_string(interfaces, "joint");
    if(!(is_cartesian || is_joint)){
      return -1;
    }
    if(is_joint){
      return 1;
    }
    if(is_cartesian){
      return 2;
    }
  }
  else{
    return 0;
  }
  return -1;
}
}