#include <multi_mode_controller/controllers/comless_panda_joint_impedance_controller.h>

#include <multi_mode_controller/utils/controller_factory.h>
#include <multi_mode_controller/utils/damping_design.h>

using namespace panda_controllers;
using Matrix7d = Eigen::Matrix<double, 7, 7>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Pose = PandaJointImpedanceControllerPose;
using Controller = ComlessPandaJointImpedanceController;

static auto registration = ControllerFactory::registerClass<Controller>(
    "comless_panda_joint_impedance_controller");

void Controller::computeTauImpl(
    const std::vector<std::array<double, 7>*>& tau,
    const Pose& desired, const double& p) {
  Pose current = getCurrentPose();
  current.q -= getOffset().q;
  Eigen::Map<const Vector7d> coriolis(robot_data_[0]->coriolis().data());
  Eigen::Map<const Matrix7d> inertia(robot_data_[0]->mass().data());
  static Matrix7d damping_ratio = Matrix7d::Identity()*0.8;
  static Vector7d stiffness(std::vector<double>(
      {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}).data());
  Matrix7d designed_damping = sqrtDesign<7>(inertia,
      Matrix7d(p * stiffness.asDiagonal()), damping_ratio.diagonal());
  auto tau_d = coriolis + p*stiffness.asDiagonal()*(desired.q - current.q) +
               designed_damping*(desired.qD - current.qD);
  for (size_t i = 0; i < 7; ++i) {
    (*tau[0])[i] = tau_d[i];
  }
}

double Controller::defaultParameters() {
  return 1.0;
}

Pose Controller::getCurrentPoseImpl() {
  Pose current;
  current.q = Vector7d(robot_data_[0]->state().q.data());
  current.qD = Vector7d(robot_data_[0]->state().dq.data());
  // RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "getCurrentPoseImpl: " << current.q << " " << current.qD);
  return current;
}

bool Controller::hasOffsetImpl() {
  return getOffset().q.norm() != 0;
}

void Controller::resetOffset() {
  Pose p;
  p.q = Vector7d::Zero();
  p.qD = Vector7d::Zero();
  setOffset(p);
}
