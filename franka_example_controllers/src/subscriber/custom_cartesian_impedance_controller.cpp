#include <franka_example_controllers/subscriber/custom_cartesian_impedance_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <franka/model.h>

inline void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true) {
    double lambda_ = damped ? 0.2 : 0.0;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
    Eigen::MatrixXd S_ = M_;  // copying the dimensions of M_, its content is not needed.
    S_.setZero();

    for (int i = 0; i < sing_vals_.size(); i++)
        S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

    M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}


namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
CustomCartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
CustomCartesianImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // should be model interface
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(franka_robot_model_name);
  }
  return config;
}

controller_interface::return_type CustomCartesianImpedanceController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {

  // get state variables
  Eigen::Map<const Matrix4d> current(franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector).data());
  Eigen::Vector3d current_position(current.block<3,1>(0,3));
  Eigen::Quaterniond current_orientation(current.block<3,3>(0,0));
  Eigen::Map<const Matrix7d> inertia(franka_robot_model_->getMassMatrix().data());
  Eigen::Map<const Vector7d> coriolis(franka_robot_model_->getCoriolisForceVector().data());
  Eigen::Matrix<double, 6, 7> jacobian(
      franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector).data());
  Eigen::Map<const Vector7d> dq(franka_robot_model_->getRobotState()->dq.data());
  Eigen::Map<const Vector7d> q(franka_robot_model_->getRobotState()->q.data());
  Eigen::Map<const Vector7d> tau_J_d(franka_robot_model_->getRobotState()->tau_J_d.data());
  
  // position error
  error_.head(3) << current_position - position_d_;
  // clip translational error
  for (int i = 0; i < 3; i++) {
    error_(i) = std::min(std::max(error_(i), translational_clip_min_(i)), translational_clip_max_(i));
  }
  
  // rotation error
  if (orientation_d_.coeffs().dot(current_orientation.coeffs()) < 0.0) {
    current_orientation.coeffs() << -current_orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(current_orientation.inverse() * orientation_d_);
  error_.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error_.tail(3) << -current.block<3,3>(0,0) * error_.tail(3);

  // clip rotation error
    for (int i = 0; i < 3; i++) {
    error_(i+3) = std::min(std::max(error_(i+3), rotational_clip_min_(i)), rotational_clip_max_(i));
  }

  // integrate error
  error_i_.head(3) << (error_i_.head(3) + error_.head(3)).cwiseMax(-2 * translational_clip_).cwiseMin(2 * translational_clip_);
  error_i_.tail(3) << (error_i_.tail(3) + error_.tail(3)).cwiseMax(-2 * rotational_clip_).cwiseMin(2 * rotational_clip_);

  // compute control
  // allocate variables
  Vector7d tau_task, tau_nullspace, tau_d;
  tau_task.setZero();
  tau_nullspace.setZero();
  tau_d.setZero();
  
  // task control torques
  tau_task << jacobian.transpose() * (-stiffness_*error_ - damping_*(jacobian*dq) - Ki_ * error_i_); 
  
  // nullspace control torques
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) - jacobian.transpose() * jacobian_transpose_pinv) *
                    (
                      ns_stiff_q1_to_4_ * (q_d_nullspace_.head(4) - q.head(4)) - (2.0 * sqrt(ns_stiff_q1_to_4_)) * dq.head(4) +
                      ns_stiff_q5_to_7_ * (q_d_nullspace_.tail(3) - q.tail(3)) - (2.0 * sqrt(ns_stiff_q5_to_7_)) * dq.tail(3) 
                    );

  tau_d <<  tau_task + coriolis + tau_nullspace;

  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);

  for (int i = 0; i < num_joints_; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
  }

  // filter desired pose targets
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);

  return controller_interface::return_type::OK;
}

CallbackReturn CustomCartesianImpedanceController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<double>("pos_stiff", 1000);
    auto_declare<double>("rot_stiff", 100);
    auto_declare<double>("ns_stiff_q1_to_4", 10);
    auto_declare<double>("ns_stiff_q5_to_7", 0.001);
    auto_declare<double>("translational_clip", 0.05);
    auto_declare<double>("rotational_clip", 0.8);
    auto_declare<double>("translational_Ki", 1);
    auto_declare<double>("rotational_Ki", 1);
    sub_eq_pose_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/cartesian_impedance/equilibrium_pose", 1,
      std::bind(&CustomCartesianImpedanceController::equilibriumPoseCallback, this, std::placeholders::_1)
    );
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn CustomCartesianImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  pos_stiff_ = get_node()->get_parameter("pos_stiff").as_double();
  rot_stiff_ = get_node()->get_parameter("rot_stiff").as_double();
  ns_stiff_q1_to_4_ = get_node()->get_parameter("ns_stiff_q1_to_4").as_double();
  ns_stiff_q5_to_7_ = get_node()->get_parameter("ns_stiff_q5_to_7").as_double();
  translational_clip_ = get_node()->get_parameter("translational_clip").as_double();
  rotational_clip_ = get_node()->get_parameter("rotational_clip").as_double();
  translational_Ki_ = get_node()->get_parameter("translational_Ki").as_double();
  rotational_Ki_ = get_node()->get_parameter("rotational_Ki").as_double();
  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(arm_id_ + "/robot_model",
                                                   arm_id_));
        
  return CallbackReturn::SUCCESS;
}

CallbackReturn CustomCartesianImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  start_time_ = this->get_node()->now();
  init_pose_matrix_ = Matrix4d(franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector).data());
  position_d_ = Vector3d(init_pose_matrix_.block<3,1>(0,3));
  position_d_target_ = position_d_;
  orientation_d_ = Quaterniond(init_pose_matrix_.block<3,3>(0,0));
  orientation_d_target_ = orientation_d_;
  q_d_nullspace_ = Vector7d(franka_robot_model_->getRobotState()->q.data());

  stiffness_.setIdentity();
  stiffness_.topLeftCorner(3, 3) << pos_stiff_ * Matrix3d::Identity();
  stiffness_.bottomRightCorner(3, 3) << rot_stiff_ * Matrix3d::Identity();
  // Simple critical damping
  damping_.setIdentity();
  damping_.topLeftCorner(3,3) << 2 * sqrt(pos_stiff_) * Matrix3d::Identity();
  damping_.bottomRightCorner(3, 3) << 0.4 * 2 * sqrt(rot_stiff_) * Matrix3d::Identity();

  translational_clip_min_ << -translational_clip_, -translational_clip_, -translational_clip_;
  translational_clip_max_ << translational_clip_, translational_clip_, translational_clip_;
  rotational_clip_min_ << -rotational_clip_, -rotational_clip_, -rotational_clip_;
  rotational_clip_max_ << rotational_clip_, rotational_clip_, rotational_clip_;
  
  Ki_.setIdentity();
  Ki_.topLeftCorner(3, 3)
      << translational_Ki_ * Eigen::Matrix3d::Identity();
  Ki_.bottomRightCorner(3, 3)
      << rotational_Ki_ * Eigen::Matrix3d::Identity();

  return CallbackReturn::SUCCESS;
}

CallbackReturn CustomCartesianImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/){
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

void CustomCartesianImpedanceController::equilibriumPoseCallback(
  const geometry_msgs::msg::PoseStamped& msg) {
  position_d_target_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
  error_i_.setZero();
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg.pose.orientation.x, msg.pose.orientation.y,
      msg.pose.orientation.z, msg.pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

Vector7d CustomCartesianImpedanceController::saturateTorqueRate(
    const Vector7d& tau_d_calculated, const Vector7d& tau_J_d) {  
  Vector7d tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CustomCartesianImpedanceController,
                       controller_interface::ControllerInterface)