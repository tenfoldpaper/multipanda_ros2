#include <panda_motion_generators/motion_generators/panda_dq_c1_cartesian_motion_generator.h>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
// #include <franka_msgs/msg/franka_state.hpp>

#include <multi_mode_controller/utils/panda_limits.h>
#include <multi_mode_control_msgs/srv/get_robot_states.hpp>
#include <multi_mode_control_msgs/srv/set_controllers.hpp>

using namespace panda_motion_generators;
namespace PL = panda_limits;
using CartesianGoalMsg = multi_mode_control_msgs::msg::CartesianImpedanceGoal;
using multi_mode_control_msgs::msg::Controller;
using multi_mode_control_msgs::srv::GetRobotStates;
using DQ_robotics::DQ;
using Eigen::Matrix4d;
using Eigen::Quaterniond;
// using RobotPosePtr = std::shared_ptr<const franka_msgs::msg::FrankaState>;
using multi_mode_control_msgs::srv::SetControllers;

PandaDqC1CartesianMotionGenerator::PandaDqC1CartesianMotionGenerator(
    const std::string& action_name,
    const std::string& robot_state_srv_name,
    const std::string& controller_name,
    const std::string& controllet_name,
    const std::string& desired_pose_name) :
    DqC1CartesianMotionGenerator<CartesianGoalMsg>(
                                action_name,
                                robot_state_srv_name,
                                controller_name,
                                desired_pose_name,
                                PL::tD_max,
                                PL::rD_max) {
  controllet_name_ = controllet_name;
};

CartesianGoalMsg PandaDqC1CartesianMotionGenerator::convertToMsg(
    const CartesianGoal& goal) {
  CartesianGoalMsg g;
  g.pose.position.x = goal.pos.translation().q(1);
  g.pose.position.y = goal.pos.translation().q(2);
  g.pose.position.z = goal.pos.translation().q(3);
  g.pose.orientation.w = goal.pos.q(0);
  g.pose.orientation.x = goal.pos.q(1);
  g.pose.orientation.y = goal.pos.q(2);
  g.pose.orientation.z = goal.pos.q(3);
  Eigen::Matrix<double, 7, 1> q_n = 0.5*(PL::q_min + PL::q_max);
  g.q_n = {q_n[0], q_n[1], q_n[2], q_n[3], q_n[4], q_n[5], q_n[6]};
  return g;
}

bool PandaDqC1CartesianMotionGenerator::getCurrentPose(::DQ& out) {
  static auto request = std::make_shared<multi_mode_control_msgs::srv::GetRobotStates::Request>();
  auto response = std::make_shared<multi_mode_control_msgs::srv::GetRobotStates::Response>();
  if(this->get_robot_state_node_.callWithTimeout(request, response, 2)){
    Eigen::Map<Matrix4d> O_T_EE(response.get()->states[0].o_t_ee.data(), 4, 4); // stride is needed for some reason
    Quaterniond r(O_T_EE.block<3,3>(0,0));
    // i,j,p,q -> block of size 3(p) rows 1(q) col, starting at row 0(i), col 3(j)
    Eigen::Vector3d t(O_T_EE.block(0,3,3,1)); // Did the syntax change? (O_T_EE.block(0,3,1,3));
    ::DQ rot(r.w(), r.x(), r.y(), r.z());
    ::DQ trans(0, t[0], t[1], t[2]);
    out = (rot + 0.5*::DQ::E*trans*rot).normalize();
    return true;
  }
  return false;
}

void PandaDqC1CartesianMotionGenerator::prepareMotion() {
  auto srv = std::make_shared<SetControllers::Request>();
  Controller c;
  c.name = controllet_name_;
  
  rclcpp::Parameter param_;
  auto res_name_ = this->parameters_client->get_parameter("resources." + c.name, param_).as_string_array()[0];
  RCLCPP_INFO(this->node_->get_logger(), "Res name: %s", res_name_.c_str());
  c.resources.push_back(res_name_);
  auto request = std::make_shared<multi_mode_control_msgs::srv::SetControllers::Request>();
  auto response = std::make_shared<multi_mode_control_msgs::srv::SetControllers::Response>();
  request->controllers.push_back(c);
  this->set_controllers_node_.callWithTimeout(request, response, 2);
}
