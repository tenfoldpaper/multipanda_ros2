#pragma once

#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include "franka_semantic_components/franka_robot_model.hpp"
#include <rclcpp/rclcpp.hpp>
/**
 * TODO
 * include needed libraries like the ros2 msgs you need.
 * Notice: the format of ros2 std msgs is std_msgs/msg/<xxx_xxx_xxx>.hpp,
 *         e.g. #include "std_msgs/msg/float64_multi_array.hpp"
 */

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_controllers {

/**
 * <Description of your controller>
 */
class CustomController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::string arm_id_;
  const int num_joints = 7;
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;
  Vector7d q_;
  Vector7d dq_;
  rclcpp::Time start_time_;
  void updateJointStates();
 /**
 * TODO
 * Declare your custom variables & entities like subscription & functions
 * - Example variable:
 * Vector7d q_d_;
 * - Example subscription:
 * rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_desired_joint_; 
 * - Example function:
 * void ExampleCallback(const std_msgs::msg::Float64MultiArray& msg);
 */
};

}  // namespace franka_controllers
