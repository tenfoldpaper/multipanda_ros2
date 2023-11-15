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

#include "franka_example_controllers/dual_joint_velocity_example_controller.hpp"

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
DualJointVelocityExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for(auto& arm_container_pair : arms_){
    for (int i = 1; i <= num_joints; ++i) {
      config.names.push_back(arm_container_pair.first + "_joint" + std::to_string(i) + "/velocity");
    }
  }
  return config;
}

controller_interface::InterfaceConfiguration
DualJointVelocityExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  for(auto& arm_container_pair : arms_){
    for (int i = 1; i <= num_joints; ++i) {
      config.names.push_back(arm_container_pair.first + "_joint" + std::to_string(i) + "/position");
      config.names.push_back(arm_container_pair.first + "_joint" + std::to_string(i) + "/velocity");
    }
  }
  return config;
}

controller_interface::return_type DualJointVelocityExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
//   updateJointStates();
  init_time_ = init_time_ + period;
  double omega = 0.1 * std::sin(init_time_.seconds());
  size_t k = 0;
  for(auto& arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;

    for (int i = 0; i < num_joints; i++) {
      if(i > 2 && i < 7){
        command_interfaces_[k].set_value(omega);
      }
      else{
        command_interfaces_[k].set_value(0);
      }
      k++; // BIG assumption: That the command interfaces are always in the same order
    }
  }
  return controller_interface::return_type::OK;
}

CallbackReturn DualJointVelocityExampleController::on_init() {
  try {
    try {
    auto_declare<std::string>("arm_1.arm_id", "panda");
    auto_declare<std::string>("arm_2.arm_id", "panda");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_node()->get_logger(), "Finished initializing dual joint velocity example controller");
  return CallbackReturn::SUCCESS;
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DualJointVelocityExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  rclcpp::Parameter arm_id_1_param = this->get_node()->get_parameter("arm_1.arm_id");
  rclcpp::Parameter arm_id_2_param = this->get_node()->get_parameter("arm_2.arm_id");
  if(arm_id_1_param.as_string() == arm_id_2_param.as_string()){
    RCLCPP_FATAL(this->get_node()->get_logger(), "Arms do not have unique ids!");
    return CallbackReturn::ERROR;
  }
  arms_.insert(std::make_pair(arm_id_1_param.as_string(), ArmContainer()));
  arms_.insert(std::make_pair(arm_id_2_param.as_string(), ArmContainer()));
  
  for(auto& arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    arm.arm_id_ = arm_container_pair.first;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DualJointVelocityExampleController::on_activate(    
  const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  start_time_ = this->get_node()->now();
  init_time_ = rclcpp::Duration(0, 0);
  return CallbackReturn::SUCCESS;
}

CallbackReturn DualJointVelocityExampleController::on_error(
  const rclcpp_lifecycle::State& /*previous_state*/){
    RCLCPP_ERROR(this->get_node()->get_logger(), "error encountered!");
    return CallbackReturn::ERROR;
  }

void DualJointVelocityExampleController::updateJointStates() {
  for(auto& arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    size_t k = 0;
    for (size_t i = 0; i < state_interfaces_.size(); i++) {
      const auto& position_interface = state_interfaces_.at(2 * i);
      const auto& velocity_interface = state_interfaces_.at(2 * i + 1);
      if(position_interface.get_prefix_name().find(arm_container_pair.first) == std::string::npos || 
         velocity_interface.get_prefix_name().find(arm_container_pair.first) == std::string::npos ){
          // if either position or velocity interface does not contain the ID of the arm, skip
          continue;
      };

      assert(position_interface.get_interface_name() == "position");
      assert(velocity_interface.get_interface_name() == "velocity");

      arm.q_(k) = position_interface.get_value();
      arm.dq_(k) = velocity_interface.get_value();

      k++;
      if(k == 7){
        break;
      }
    }
  }
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::DualJointVelocityExampleController,
                       controller_interface::ControllerInterface)