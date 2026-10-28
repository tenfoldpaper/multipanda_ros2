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

#include "franka_example_controllers/comless/cartesian_pose_example_controller.hpp"

#include <cmath>
#include <exception>
#include <string>

namespace franka_example_controllers {

std::string CartesianPoseExampleController::poseInterfaceName(int index) const {
  // The hardware names the column-major pose entries "00" to "15"
  const std::string entry = index < 10 ? "0" + std::to_string(index) : std::to_string(index);
  return arm_id_ + k_cartesian_position_interface_name + "/" + entry;
}

controller_interface::InterfaceConfiguration
CartesianPoseExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 0; i < kPoseSize; ++i) {
    config.names.push_back(poseInterfaceName(i));
  }
  return config;
}

controller_interface::InterfaceConfiguration
CartesianPoseExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 0; i < kPoseSize; ++i) {
    config.names.push_back(poseInterfaceName(i));
  }
  return config;
}

controller_interface::return_type CartesianPoseExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  // Hold the start pose first, so the robot settles and the long first period after a
  // controller switch does not make the trajectory jump
  if (hold_elapsed_ < hold_duration_) {
    hold_elapsed_ = hold_elapsed_ + period;
    setPose(initial_pose_);
    return controller_interface::return_type::OK;
  }
  init_time_ = init_time_ + period;

  double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * init_time_.seconds()));
  std::array<double, kPoseSize> pose = initial_pose_;
  pose[12] += radius_ * std::sin(angle);
  pose[14] += radius_ * (std::cos(angle) - 1);
  setPose(pose);
  return controller_interface::return_type::OK;
}

void CartesianPoseExampleController::setPose(const std::array<double, kPoseSize>& pose) {
  for (int i = 0; i < kPoseSize; ++i) {
    command_interfaces_[i].set_value(pose[i]);
  }
}

CallbackReturn CartesianPoseExampleController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPoseExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPoseExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  for (int i = 0; i < kPoseSize; ++i) {
    initial_pose_[i] = state_interfaces_[i].get_value();
  }
  // Command the start pose right away, so the hardware never forwards a stale command
  setPose(initial_pose_);
  init_time_ = rclcpp::Duration(0, 0);
  hold_elapsed_ = rclcpp::Duration(0, 0);
  return CallbackReturn::SUCCESS;
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianPoseExampleController,
                       controller_interface::ControllerInterface)
