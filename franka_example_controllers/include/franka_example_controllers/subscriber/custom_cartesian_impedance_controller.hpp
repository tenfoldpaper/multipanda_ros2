#pragma once

#include <string>

#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "franka_semantic_components/franka_robot_model.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {
using Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix<double, 4, 4>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix7d = Eigen::Matrix<double, 7, 7>;

using Vector3d = Eigen::Matrix<double, 3, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;

using Eigen::Quaterniond;

/**
 * The cartesian impedance example controller implements the Hogan formulation.
 */
class CustomCartesianImpedanceController : public controller_interface::ControllerInterface {
 public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  Vector7d saturateTorqueRate(const Vector7d& tau_d_calculated,
                              const Vector7d& tau_J_d);  // Saturation
  void publishData(void);                                // Publish data

  std::string arm_id_;
  const int num_joints_ = 7;
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  rclcpp::Time start_time_;
  Quaterniond orientation_d_;
  Quaterniond orientation_d_target_;
  Vector3d position_d_;
  Vector3d position_d_target_;
  Vector7d q_d_nullspace_;
  Matrix4d init_pose_matrix_;
  Vector6d error_;
  Vector6d error_i_;

  Eigen::Vector3d current_position_;
  Eigen::Quaterniond current_orientation_;
  Vector7d q_;
  Vector7d dq_;
  Vector7d tau_J_d_;

  double filter_params_{0.008};
  const double delta_tau_max_{0.5};
  Matrix6d stiffness_;
  Matrix6d damping_;
  double pos_stiff_;
  double rot_stiff_;
  double ns_stiff_q1_to_4_;
  double ns_stiff_q5_to_7_;

  double translational_Ki_;
  double rotational_Ki_;
  Matrix6d Ki_;

  double pub_frequency_;
  rclcpp::TimerBase::SharedPtr pub_timer_;

  std::mutex data_mutex_;

  double translational_clip_;
  Vector3d translational_clip_min_;
  Vector3d translational_clip_max_;
  double rotational_clip_;
  Vector3d rotational_clip_min_;
  Vector3d rotational_clip_max_;

  // subscribers
  void equilibriumPoseCallback(const geometry_msgs::msg::PoseStamped& msg);
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_eq_pose_;

  // publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cartesian_pos_des_filt_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cartesian_pos_curr_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pos_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_vel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_torques_pub_;
};

}  // namespace franka_example_controllers