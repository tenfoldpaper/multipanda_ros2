#include <rclcpp/rclcpp.hpp>

#include <panda_motion_generators/motion_generators/panda_poly_c2_joint_motion_generator.h>

using namespace panda_motion_generators;

int main(int argc, char** argv) {
  //, "panda_motion_generator_node"
  // gotta parametrize these to a yaml file or something
  rclcpp::init(argc, argv);
  if(argc != 7){
    std::cerr << "The node requires 5 arguments!\n"
                 "Usage: node --ros-args action_name "
                                        "robot_state_srv_name "
                                        "controller_name "
                                        "controllet_name "
                                        "desired_pose_name" << std::endl;
  }
  std::string action_name = argv[1];//"joint_via_motion";
  std::string robot_state_srv_name = argv[2];//"/left_and_right/get_robot_states";
  std::string controller_name = argv[3];//"single_multi_mode_controller";
  std::string controllet_name = argv[4];//"panda_joint_impedance_controller";
  std::string desired_pose_name = argv[5];//"/left_and_right/des  _coupled_dual_cartesian_impedance_controller/desired_pose";
  printf("Starting joint via motion generator with the following parameters:\n%s\n%s\n%s\n%s\n%s\n############\n",
          action_name.c_str(),
          robot_state_srv_name.c_str(),
          controller_name.c_str(),
          controllet_name.c_str(),
          desired_pose_name.c_str());
  auto mg = PandaPolyC2JointMotionGenerator(action_name, 
                                            robot_state_srv_name, 
                                            controller_name, 
                                            controllet_name,
                                            desired_pose_name);
  mg.spinROSNode();
  rclcpp::shutdown();
  return 0;
}
