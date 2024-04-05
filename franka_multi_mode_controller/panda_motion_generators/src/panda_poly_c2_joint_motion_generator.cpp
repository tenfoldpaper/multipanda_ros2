#include <panda_motion_generators/motion_generators/panda_poly_c2_joint_motion_generator.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <multi_mode_controller/utils/panda_limits.h>
#include <multi_mode_control_msgs/srv/get_robot_states.hpp>
#include <multi_mode_control_msgs/srv/set_controllers.hpp>

using namespace panda_motion_generators;
using namespace std::chrono_literals;
namespace PL = panda_limits;
using multi_mode_control_msgs::msg::Controller;
using JointGoalMsg = multi_mode_control_msgs::msg::JointGoal;
using multi_mode_control_msgs::srv::GetRobotStates;
using RobotPosePtr = std::shared_ptr<const sensor_msgs::msg::JointState>;
using multi_mode_control_msgs::srv::SetControllers;
using Vector7d = Eigen::Matrix<double, 7, 1>;

PandaPolyC2JointMotionGenerator::PandaPolyC2JointMotionGenerator(
    const std::string& action_name,
    const std::string& robot_state_srv_name,
    const std::string& controller_name,
    const std::string& controllet_name,
    const std::string& desired_pose_name) :
    PolyC2JointMotionGenerator<JointGoalMsg, 7>(action_name, 
                                                robot_state_srv_name, 
                                                controller_name, 
                                                desired_pose_name, 
                                                PL::q_min,
                                                PL::q_max, 
                                                PL::qD_max, 
                                                PL::qDD_max) {  
  controllet_name_ = controllet_name;
};
JointGoalMsg PandaPolyC2JointMotionGenerator::convertToMsg(
    const JointGoal<7>& goal) {
  JointGoalMsg g;
  Eigen::Map<Vector7d> q(g.q.data());
  Eigen::Map<Vector7d> qD(g.qd.data());
  q = goal.q;
  qD = goal.qD;
  return g;
}

bool PandaPolyC2JointMotionGenerator::getCurrentPose(Vector7d& out) {
  // probably need a way to ensure that this call is short
  static auto request = std::make_shared<multi_mode_control_msgs::srv::GetRobotStates::Request>();
  auto response = std::make_shared<multi_mode_control_msgs::srv::GetRobotStates::Response>();
  if(this->get_robot_state_node_.callWithTimeout(request, response, 2))
  {
    for (int i=0;i<7;++i) {
      out[i] = response.get()->states[0].q[i];
    }
    return true;
  }
  return false;
}

void PandaPolyC2JointMotionGenerator::prepareMotion() {
  auto srv = std::make_shared<SetControllers::Request>();
  Controller c;
  // Need to parametrize c.name to a member variable set at init
  c.name = controllet_name_;
  //TODO: parametrize hardcoded namespace later
  
  rclcpp::Parameter param_;
  auto res_name_ = this->parameters_client->get_parameter("resources." + c.name, param_).as_string_array()[0];
  RCLCPP_INFO(this->node_->get_logger(), "Res name: %s", res_name_.c_str());
  c.resources.push_back(res_name_);
  auto request = std::make_shared<multi_mode_control_msgs::srv::SetControllers::Request>();
  auto response = std::make_shared<multi_mode_control_msgs::srv::SetControllers::Response>();
  request->controllers.push_back(c);
  this->set_controllers_node_.callWithTimeout(request, response, 2);
}
