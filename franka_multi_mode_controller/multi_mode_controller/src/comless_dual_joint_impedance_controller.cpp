#include <multi_mode_controller/controllers/comless_dual_joint_impedance_controller.h>

#include <multi_mode_controller/utils/controller_factory.h>
#include <multi_mode_controller/utils/damping_design.h>

using namespace panda_controllers;
using Matrix7d = Eigen::Matrix<double, 7, 7>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Pose = DualJointImpedanceControllerPose;
using Controller = ComlessDualJointImpedanceController;

static auto registration = ControllerFactory::registerClass<Controller>(
    "comless_dual_joint_impedance_controller");

void Controller::computeTauImpl(
    const std::vector<std::array<double, 7>*>& tau,
    const Pose& desired, const double& p) {
  Pose current = getCurrentPose();
  for(int i = 0; i < 2; i++){
    current.poses[i].q -= getOffset().poses[i].q;
    Eigen::Map<const Vector7d> coriolis(robot_data_[i]->coriolis().data());
    Eigen::Map<const Matrix7d> inertia(robot_data_[i]->mass().data());
    static Matrix7d damping_ratio = Matrix7d::Identity()*0.8;
    static Vector7d stiffness(std::vector<double>(
        {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0}).data());
    Matrix7d designed_damping = sqrtDesign<7>(inertia,
        Matrix7d(p * stiffness.asDiagonal()), damping_ratio.diagonal());
    auto tau_d = coriolis + p*stiffness.asDiagonal()*(desired.poses[i].q - current.poses[i].q) +
                designed_damping*(desired.poses[i].qD - current.poses[i].qD);
    for (size_t j = 0; j < 7; ++j) {
      (*tau[i])[j] = tau_d[j];
    }
  }
}

double Controller::defaultParameters() {
  return 1.0;
}

Pose Controller::getCurrentPoseImpl() {
  Pose current;
  for(int i = 0; i < 2; i++){
    current.poses[i].q = Vector7d(robot_data_[i]->state().q.data());
    current.poses[i].qD = Vector7d(robot_data_[i]->state().dq.data());
  }
  return current;
}

bool Controller::hasOffsetImpl() {
  return (getOffset().poses[0].q.norm() != 0 || getOffset().poses[1].q.norm() != 0);
}

void Controller::resetOffset() {
  Pose p;
  for(int i = 0; i < 2; i++){
    p.poses[i].q = Vector7d::Zero();
    p.poses[i].qD = Vector7d::Zero();
  }
  setOffset(p);
}
