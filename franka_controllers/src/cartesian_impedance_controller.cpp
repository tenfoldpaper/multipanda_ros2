#include <franka_controllers/cartesian_impedance_controller.hpp>

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


namespace franka_controllers {

controller_interface::InterfaceConfiguration
CartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
CartesianImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // should be model interface
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(franka_robot_model_name);
  }
  return config;
}

controller_interface::return_type CartesianImpedanceController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  Eigen::Map<const Matrix4d> current(franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector).data());
  Eigen::Vector3d current_position(current.block<3,1>(0,3));
  Eigen::Quaterniond current_orientation(current.block<3,3>(0,0));
  Eigen::Map<const Matrix7d> inertia(franka_robot_model_->getMassMatrix().data());
  Eigen::Map<const Vector7d> coriolis(franka_robot_model_->getCoriolisForceVector().data());
  Eigen::Matrix<double, 6, 7> jacobian(
      franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector).data());
  Eigen::Map<const Vector7d> qD(franka_robot_model_->getRobotState()->dq.data());
  Eigen::Map<const Vector7d> q(franka_robot_model_->getRobotState()->q.data());
  Vector6d error;

  auto desired_position_cur = desired_position;
  error.head(3) << current_position - desired_position_cur;
  if (desired_orientation.coeffs().dot(current_orientation.coeffs()) < 0.0) {
    current_orientation.coeffs() << -current_orientation.coeffs();
  }
  Eigen::Quaterniond rot_error(
      current_orientation * desired_orientation.inverse());
  Eigen::AngleAxisd rot_error_aa(rot_error);
  error.tail(3) << rot_error_aa.axis() * rot_error_aa.angle();
  Vector7d tau_task, tau_nullspace, tau_d;
  tau_task.setZero();
  tau_nullspace.setZero();
  tau_d.setZero();
  tau_task << jacobian.transpose() * (-stiffness*error - damping*(jacobian*qD));

  Eigen::MatrixXd jacobian_transpose_pinv;
    pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                      jacobian.transpose() * jacobian_transpose_pinv) *
                         (n_stiffness * (desired_qn - q) -
                          (2.0 * sqrt(n_stiffness)) * qD);

  tau_d <<  tau_task + coriolis + tau_nullspace;
  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
  }
  return controller_interface::return_type::OK;
}

CallbackReturn CartesianImpedanceController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<double>("pos_stiff", 100);
    auto_declare<double>("rot_stiff", 10);
    sub_desired_cartesian_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/cartesian_impedance/pose_desired", 1,
      std::bind(&CartesianImpedanceController::desiredCartesianCallback, this, std::placeholders::_1)
    );
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  pos_stiff = get_node()->get_parameter("pos_stiff").as_double();
  rot_stiff = get_node()->get_parameter("rot_stiff").as_double();
  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(arm_id_ + "/robot_model",
                                                   arm_id_));
  auto parameters = get_node()->list_parameters({}, 10);
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);
  desired = Matrix4d(franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector).data());
  desired_position = Vector3d(desired.block<3,1>(0,3));
  desired_orientation = Quaterniond(desired.block<3,3>(0,0));
  desired_qn = Vector7d(franka_robot_model_->getRobotState()->q.data());

  stiffness.setIdentity();
  stiffness.topLeftCorner(3, 3) << pos_stiff * Matrix3d::Identity();
  stiffness.bottomRightCorner(3, 3) << rot_stiff * Matrix3d::Identity();
  // Simple critical damping
  damping.setIdentity();
  damping.topLeftCorner(3,3) << 2 * sqrt(pos_stiff) * Matrix3d::Identity();
  damping.bottomRightCorner(3, 3) << 0.8 * 2 * sqrt(rot_stiff) * Matrix3d::Identity();
  n_stiffness = 10.0;

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/){
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

void CartesianImpedanceController::desiredCartesianCallback(
  const std_msgs::msg::Float64MultiArray& msg) {
  if (msg.data[0]){
    for (auto i = 0; i < 3; ++i) {
      desired_position[i] = msg.data[i];
    }
    if (msg.data[11]){ // for orientation matrix
      Matrix3d desired_orientation_mat;
      for (auto i = 0; i < 3; ++i) {
        for (auto j = 0; j < 3; ++j) {
          desired_orientation_mat(i, j) = msg.data[3+3*i+j];
        }
      }
      desired_orientation = Eigen::Quaterniond(desired_orientation_mat);
    }

  }
}

}  // namespace franka_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_controllers::CartesianImpedanceController,
                       controller_interface::ControllerInterface)
