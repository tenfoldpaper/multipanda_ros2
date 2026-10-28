// Copyright (c) 2021 Franka Emika GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <array>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

/**
 * The Cartesian pose example controller moves the end effector back and forth on a circular arc in
 * the x-z plane, keeping its orientation (as the libfranka generate_cartesian_pose_motion example).
 */
class CartesianPoseExampleController : public controller_interface::ControllerInterface {
 public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::string arm_id_;
  const std::string k_cartesian_position_interface_name{"_ee_cartesian_position"};
  static constexpr int kPoseSize = 16;
  const double radius_ = 0.1;  // [m] of the circular arc
  std::array<double, kPoseSize> initial_pose_{};
  rclcpp::Duration init_time_ = rclcpp::Duration(0, 0);
  // Time the start pose is held after activation, before the motion begins
  const rclcpp::Duration hold_duration_ = rclcpp::Duration::from_seconds(0.1);
  rclcpp::Duration hold_elapsed_ = rclcpp::Duration(0, 0);
  std::string poseInterfaceName(int index) const;
  void setPose(const std::array<double, kPoseSize>& pose);
};

}  // namespace franka_example_controllers
