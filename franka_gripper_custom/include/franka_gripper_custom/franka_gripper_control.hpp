#pragma once

#include <memory>
#include <string>

#include <franka/exception.h>
#include <franka/gripper.h>  // Franka C++ API
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>     // TEMP
#include <std_msgs/msg/float64.hpp>  // message type for gripper width command

class GripperSubscriber : public rclcpp::Node {
 public:
  /// Constructor
  GripperSubscriber();

 private:
  // Franka gripper object (connection to hardware)
  std::unique_ptr<franka::Gripper> gripper_;

  // protect concurrent access to gripper
  std::mutex gripper_state_mutex_;

  // gripper state
  franka::GripperState current_gripper_state_;

  // ROS2 subscription
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;

  // ROS2 publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr width_pub_;

  // Timer for periodic state publishing
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  double default_speed_;
  std::string robot_ip_;
  std::vector<std::string> joint_names_;
  int pub_frequency_;
  double gripper_max_effort_;
  double default_epsilon_inner_;
  double default_epsilon_outer_;

  // Callback function executed when a message is received
  void commandCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void stopCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void publishGripperState();
};
