#include <franka_mj_hardware/helper_functions.hpp>
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
std::string get_ns(std::string const& s)
{
    std::string::size_type pos = s.find_last_of('_');
    if (pos != std::string::npos)
    {
        return s.substr(0, pos);
    }
    else
    {
        return s;
    }
}

// Function for extracting joint number
int get_joint_no(std::string const& s){
  int no = s.back() - '0' - 1;
  return no;
}
void set_torque_control(const mjModel* m,int actuator_no,int flag)
{
  if (flag==0)
    m->actuator_gainprm[10*actuator_no+0]=0;
  else
    m->actuator_gainprm[10*actuator_no+0]=1;
}
void set_position_servo(const mjModel* m,int actuator_no,double kp)
  {
    m->actuator_gainprm[10*actuator_no+0]=kp;
    m->actuator_biasprm[10*actuator_no+1]=-kp;
  }
  /******************************/

  /******************************/
  void set_velocity_servo(const mjModel* m,int actuator_no,double kv)
  {
    m->actuator_gainprm[10*actuator_no+0]=kv;
    m->actuator_biasprm[10*actuator_no+2]=-kv;
  }
}