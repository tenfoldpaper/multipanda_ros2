#include <multi_mode_controller/controllers/des_coupled_dual_cartesian_impedance_controller.h>

#include <multi_mode_controller/utils/controller_factory.h>

using namespace panda_controllers;
using Eigen::Vector3d;
using Eigen::Quaterniond;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Pose = DualCartesianImpedanceControllerPose;
using Parameters = DualCartesianImpedanceControllerParams;
using GoalMsg = multi_mode_control_msgs::msg::CartesianImpedanceGoal;
using ConfigRequest = multi_mode_control_msgs::srv::SetCartesianImpedance::Request;
using ConfigResponse = multi_mode_control_msgs::srv::SetCartesianImpedance::Response;
using Controller = DesCoupledDualCartesianImpedanceController;


static auto registration = ControllerFactory::registerClass<Controller>(
    "des_coupled_dual_cartesian_impedance_controller");


bool Controller::desiredPoseCallbackImpl(Pose& p_d, 
                                         const Pose& p,
                                         const GoalMsg& msg) {
  
  auto& clk = *PandaControllerBase<Parameters, Pose>::node_->get_clock();
  const auto& logger = PandaControllerBase<Parameters, Pose>::node_->get_logger();
  this->bCollisionAvoidance=false;
  this->bManipulability=true;
  
  p_d.poses[0].position = Vector3d(msg.pose.position.x, msg.pose.position.y - (arm_distance/2) + y_offset,
                          msg.pose.position.z);
  p_d.poses[1].position = Vector3d(msg.pose.position.x, msg.pose.position.y + (arm_distance/2) - y_offset,
                          msg.pose.position.z);
  if ((p_d.poses[0].position+PandaControllerBase<Parameters, Pose>::getOffset().poses[0].position-p.poses[0].position).norm() > 0.4 ||
      (p_d.poses[1].position+PandaControllerBase<Parameters, Pose>::getOffset().poses[1].position-p.poses[1].position).norm() > 0.4) {
    RCLCPP_WARN_THROTTLE(logger, clk, 1000, "des_coupled_DCIC: Discarding "
        "target pose that is too far away from current pose (left: %f m, right: %f m, allowed "
        "maximum is 0.1 m).",
        (p_d.poses[0].position+PandaControllerBase<Parameters, Pose>::getOffset().poses[0].position-p.poses[0].position).norm(),
        (p_d.poses[1].position+PandaControllerBase<Parameters, Pose>::getOffset().poses[1].position-p.poses[1].position).norm());
    return false;
  }
  // ideally, orientation needs to be fixed in place.
  p_d.poses[0].orientation = Quaterniond(msg.pose.orientation.w, msg.pose.orientation.x,
                                msg.pose.orientation.y, msg.pose.orientation.z);
  // should be neg -w and -y? we want mirroring along the xz plane
  p_d.poses[1].orientation = Quaterniond(-msg.pose.orientation.w, msg.pose.orientation.x,
                                msg.pose.orientation.y, msg.pose.orientation.z);
  if (p.poses[0].orientation.angularDistance(p_d.poses[0].orientation) > 0.4 ||
      p.poses[1].orientation.angularDistance(p_d.poses[1].orientation) > 0.4) {
    RCLCPP_WARN_THROTTLE(logger, clk, 1000, "des_coupled_DCIC: Discarding "
        "target pose that rotates too far away from current pose (left: %f rad, right: %f rad, "
        "allowed maximum is 0.15 rad).",
        p.poses[0].orientation.angularDistance(p_d.poses[0].orientation),
        p.poses[1].orientation.angularDistance(p_d.poses[1].orientation));
    return false;
  }
  p_d.poses[0].q_n = Eigen::Map<const Vector7d>(msg.q_n.data());
  p_d.poses[1].q_n = Eigen::Map<const Vector7d>(msg.q_n.data());
  return true;
}

bool Controller::setParametersCallbackImpl(Parameters& p_d, const Parameters& p,
            const std::shared_ptr<ConfigRequest>& request, const std::shared_ptr<ConfigResponse>& response) {
  for(int i=0; i<2; i++){
    p_d.params[i].stiffness = Matrix6d(request->stiffness.data());
    p_d.params[i].damping_ratio = Vector6d(request->damping_ratio.data());
    p_d.params[i].nullspace_stiffness = request->nullspace_stiffness;
  }
  return true;
}
