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

#include "franka_example_controllers/comless/cartesian_velocity_example_controller.hpp"

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
CartesianVelocityExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // The hardware prefixes the Cartesian interfaces with the arm id
  const std::string prefix = arm_id_ + "_ee_cartesian_velocity/";
  config.names.push_back(prefix + "tx");
  config.names.push_back(prefix + "ty");
  config.names.push_back(prefix + "tz");
  config.names.push_back(prefix + "omega_x");
  config.names.push_back(prefix + "omega_y");
  config.names.push_back(prefix + "omega_z");

  return config;
}

controller_interface::InterfaceConfiguration
CartesianVelocityExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  const std::string prefix = arm_id_ + "_ee_cartesian_velocity/";
  for (int i = 0; i < 10; ++i) {
    config.names.push_back(prefix + "0" + std::to_string(i));
  }
  for (int i = 10; i < 16; ++i) {
    config.names.push_back(prefix + std::to_string(i));
  }
  
  return config;
}

controller_interface::return_type CartesianVelocityExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  // updateJointStates();
  init_time_ = init_time_ + period;
  // Trace a circle in the x-y plane of the base frame through the start point, counter-clockwise
  // seen from above. The angular rate ramps up smoothly, so the motion starts at rest.
  const double radius = 0.05;       // [m]
  const double circle_time = 5.0;   // [s] per revolution
  const double ramp_time = 2.0;     // [s] until full speed
  const double omega = 2.0 * M_PI / circle_time;
  const double t = init_time_.seconds();
  double rate = omega;                          // angular rate along the circle
  double phase = omega * (t - 0.5 * ramp_time);  // angle along the circle
  if (t < ramp_time) {
    rate = omega * 0.5 * (1.0 - std::cos(M_PI * t / ramp_time));
    phase = omega * (0.5 * t - ramp_time / (2.0 * M_PI) * std::sin(M_PI * t / ramp_time));
  }
  double v_x = -radius * rate * std::sin(phase);
  double v_y = radius * rate * std::cos(phase);
  std::array<double, 6> command = {{v_x, v_y, 0.0, 0.0, 0.0, 0.0}};
  for(int i = 0; i < 6; i++){
    command_interfaces_[i].set_value(command[i]);
  }
  return controller_interface::return_type::OK;
}

CallbackReturn CartesianVelocityExampleController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianVelocityExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianVelocityExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // updateJointStates();
  initial_q_ = q_;
  start_time_ = this->get_node()->now();
  init_time_ = rclcpp::Duration(0, 0);
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianVelocityExampleController::on_error(
  const rclcpp_lifecycle::State& /*previous_state*/){
    RCLCPP_ERROR(this->get_node()->get_logger(), "error encountered!");
    return CallbackReturn::ERROR;
  }

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianVelocityExampleController,
                       controller_interface::ControllerInterface)