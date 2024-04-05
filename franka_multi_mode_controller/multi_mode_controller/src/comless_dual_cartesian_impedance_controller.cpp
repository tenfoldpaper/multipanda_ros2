#include <multi_mode_controller/controllers/comless_dual_cartesian_impedance_controller.h>
#include <multi_mode_controller/utils/controller_factory.h>
#include <multi_mode_controller/utils/damping_design.h>
#include <multi_mode_controller/utils/nullspace_projection.h>
// SEEMS LIKE A POINTER ISSUE.
// WHEN TWO CLASSES DERIVE FROM THE SAME PARENT, IT DOESN'T SEEM TO WORK
using namespace panda_controllers;
using Eigen::Vector3d;
using Matrix7d = Eigen::Matrix<double, 7, 7>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Eigen::Quaterniond;
using Pose = DualCartesianImpedanceControllerPose;
using Params = DualCartesianImpedanceControllerParams;
using Controller = ComlessDualCartesianImpedanceController;

static auto registration = ControllerFactory::registerClass<Controller>(
    "comless_dual_cartesian_impedance_controller");

void Controller::computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
    const Pose& desired_poses, const Params& p) {
  // measurement setup
  // double start_time = get_wall_time();
  // std::vector<std::array<double,7>> taus_(2);
  // measurement setup

  Pose current_poses = getCurrentPose();
  Eigen::VectorXd right_left_qs(17);
  right_left_qs << 0, 0, 0, current_poses.poses[1].q_n, current_poses.poses[0].q_n;
  // Vector14d Q = ... 
  bool isRight = false;
  for(int i = 0; i < 2; i++){
    auto& desired = desired_poses.poses[i];
    auto& current = current_poses.poses[i];
    current.position -= getOffset().poses[i].position;
    Eigen::Map<const Matrix7d> inertia(robot_data_[i]->mass().data());
    Eigen::Map<const Vector7d> coriolis(robot_data_[i]->coriolis().data());
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(
        robot_data_[i]->eeZeroJacobian().data());
    Eigen::Map<const Vector7d> qD(robot_data_[i]->state().dq.data());
    Vector6d error;
    error.head(3) << current.position - desired.position;
    if (desired.orientation.coeffs().dot(current.orientation.coeffs()) < 0.0) {
      current.orientation.coeffs() << -current.orientation.coeffs();
    }
    Vector7d tau_collision(7);
    tau_collision.setZero();
    if(bCollisionAvoidance){
      tau_collision << -redundancy_resolution::CollisionAvoidanceGradient(right_left_qs, isRight);
      // if(isRight){
      //   RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "CA tau: " <<  tau_collision.transpose());
      // }
    }
    Vector7d tau_manipulability(7);
    tau_manipulability.setZero();
    if(bManipulability){
      tau_manipulability << -redundancy_resolution::ManipulabilityGradient(right_left_qs, isRight);
    }
    isRight = !isRight;
    Eigen::Quaterniond rot_error(
        current.orientation * desired.orientation.inverse());
    Eigen::AngleAxisd rot_error_aa(rot_error);
    error.tail(3) << rot_error_aa.axis() * rot_error_aa.angle();
    Vector7d tau_task, tau_nullspace, tau_d;
    Matrix6d D = sqrtDesign<6>(pandaCartesianInertia(jacobian, inertia),
        p.params[i].stiffness, p.params[i].damping_ratio);
    tau_task << jacobian.transpose() * (-p.params[i].stiffness*error - D*(jacobian*qD));
    tau_nullspace <<
        getDynamicallyConsistentNullspaceProjection<7>(inertia, jacobian) *
        (p.params[i].nullspace_stiffness * (desired.q_n - current.q_n) -
        (2.0 * std::sqrt(p.params[i].nullspace_stiffness)) * qD);
    tau_d << tau_task + tau_nullspace + coriolis + tau_collision + tau_manipulability;

    for (size_t j = 0; j < 7; ++j) {
      (*tau[i])[j] = std::min(10.0, tau_d[j]); // saturate the torque mag
      // taus_[i][j] = std::min(10.0, tau_d[j]); // measurement 
    }
  }
  /*Uncomment for logging */
  // if(getCounter() < max_count_){
  //   double end_time = get_wall_time();
  //   double diff = end_time - start_time;
  //   measureTime(diff);
  //   measureTau(taus_, end_time);
  //   increaseCounter();
  // }
  // else{
  //   if(!logged_){
  //     RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Max count reached: " << getCounter() << "/" << max_count_);
  //     std::string file_name{"dcic"};
  //     write_tau_to_file(file_name);
  //     write_time_to_file(file_name);
  //     RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Files written");
  //     logged_ = true;
  //   }
  // }
  /*Uncomment for logging */
}

Params Controller::defaultParameters() {
  Params p;
  for(int i = 0; i < 2; i++){
    p.params[i].stiffness.setIdentity();
    p.params[i].stiffness.topLeftCorner(3, 3) << 400 * Eigen::Matrix3d::Identity();
    p.params[i].stiffness.bottomRightCorner(3, 3) << 20 * Eigen::Matrix3d::Identity();
    p.params[i].damping_ratio = Vector6d::Constant(0.8);
    p.params[i].nullspace_stiffness = 10;
  }
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Params set");
  
  return p;
}

Pose Controller::getCurrentPoseImpl() {
  Pose p;
  for(int i = 0; i < 2; i++){
    Eigen::Map<const Vector7d> q(robot_data_[i]->state().q.data());
    Eigen::Affine3d transform(
        Eigen::Matrix4d::Map(robot_data_[i]->state().O_T_EE.data()));
    p.poses[i].position = transform.translation();
    p.poses[i].orientation = Eigen::Quaterniond(transform.linear());
    p.poses[i].q_n = q;
  }
  // RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "getCurrentPoseImpl: " << p.position.transpose() << " " << p.orientation);
  return p;
}

bool Controller::hasOffsetImpl() {
  return getOffset().poses[0].orientation.norm() != 0 || getOffset().poses[1].position.norm() != 0;
}

void Controller::resetOffset() {
  Pose p;
  for(int i = 0; i<2; i++){
    p.poses[i].position = Vector3d::Zero();
    p.poses[i].orientation = Quaterniond(1, 0, 0, 0);
    p.poses[i].q_n = Vector7d::Zero();
  }
  setOffset(p);
}

