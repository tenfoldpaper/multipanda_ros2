#pragma once

#include "franka/exception.h"
#include "franka_hardware/robot.hpp"
#include "franka_msgs/srv/error_recovery.hpp"

#include <rclcpp/rclcpp.hpp>

namespace franka_hardware{
class FrankaErrorRecoveryServiceServer : public rclcpp::Node {
public:
  FrankaErrorRecoveryServiceServer(const rclcpp::NodeOptions& options, std::shared_ptr<Robot> robot, std::string prefix="");

private:
  
  void triggerAutomaticRecovery(const franka_msgs::srv::ErrorRecovery::Request::SharedPtr& request,
                                const franka_msgs::srv::ErrorRecovery::Response::SharedPtr& response);

  std::shared_ptr<Robot> robot_;
  rclcpp::Service<franka_msgs::srv::ErrorRecovery>::SharedPtr error_recovery_service_;

  
};

} // namespace franka_hardware