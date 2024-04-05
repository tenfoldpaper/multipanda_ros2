#include <multi_mode_controller/controllers/panda_cartesian_impedance_controller.h>

#include <multi_mode_controller/utils/controller_factory.h>

using namespace panda_controllers;
using Eigen::Vector3d;
using Eigen::Quaterniond;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Pose = PandaCartesianImpedanceControllerPose;
using Parameters = PandaCartesianImpedanceControllerParams;
using GoalMsg = multi_mode_control_msgs::msg::CartesianImpedanceGoal;
using ConfigRequest = multi_mode_control_msgs::srv::SetCartesianImpedance::Request;
using ConfigResponse = multi_mode_control_msgs::srv::SetCartesianImpedance::Response;
using Controller = PandaCartesianImpedanceController;


static auto registration = ControllerFactory::registerClass<Controller>(
    "panda_cartesian_impedance_controller");


bool Controller::desiredPoseCallbackImpl(Pose& p_d, 
                                         const Pose& p,
                                         const GoalMsg& msg) {
  p_d.position = Vector3d(msg.pose.position.x, msg.pose.position.y,
                          msg.pose.position.z);
  if ((p_d.position+PandaControllerBase<Parameters, Pose>::getOffset().position-p.position).norm() > 0.1) {
    auto& clk = *PandaControllerBase<Parameters, Pose>::node_->get_clock();
    const auto& logger = PandaControllerBase<Parameters, Pose>::node_->get_logger();
    RCLCPP_WARN_THROTTLE(logger, clk, 1000, "panda_cartesian_impedance_controller: Discarding "
        "target pose that is too far away from current pose (%f m, allowed "
        "maximum is 0.1 m).",
        (p_d.position+PandaControllerBase<Parameters, Pose>::getOffset().position-p.position).norm());
    return false;
  }
  p_d.orientation = Quaterniond(msg.pose.orientation.w, msg.pose.orientation.x,
                                msg.pose.orientation.y, msg.pose.orientation.z);
  if (p.orientation.angularDistance(p_d.orientation) > 0.15) {
    auto& clk = *PandaControllerBase<Parameters, Pose>::node_->get_clock();
    const auto& logger = PandaControllerBase<Parameters, Pose>::node_->get_logger();
    RCLCPP_WARN_THROTTLE(logger, clk, 1000, "panda_cartesian_impedance_controller: Discarding "
        "target pose that rotates too far away from current pose (%f rad, "
        "allowed maximum is 0.15 rad).",
        p.orientation.angularDistance(p_d.orientation));
    return false;
  }
  p_d.q_n = Eigen::Map<const Vector7d>(msg.q_n.data());
  return true;
}

bool Controller::setParametersCallbackImpl(Parameters& p_d, const Parameters& p,
            const std::shared_ptr<ConfigRequest>& request, const std::shared_ptr<ConfigResponse>& response) {
  p_d.stiffness = Matrix6d(request->stiffness.data());
  p_d.damping_ratio = Vector6d(request->damping_ratio.data());
  p_d.nullspace_stiffness = request->nullspace_stiffness;
  return true;
}
