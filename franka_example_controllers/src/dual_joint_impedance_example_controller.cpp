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

#include <franka_example_controllers/dual_joint_impedance_example_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
DualJointImpedanceExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for(auto& arm_container_pair : arms_){
    for (int i = 1; i <= num_joints; ++i) {
      config.names.push_back(arm_container_pair.first + "_joint" + std::to_string(i) + "/effort");
    }
  }
  return config;
}

controller_interface::InterfaceConfiguration
DualJointImpedanceExampleController::state_interface_configuration() const {
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

controller_interface::return_type DualJointImpedanceExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  updateJointStates();
  for(auto& arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    Vector7d q_goal = arm.initial_q_;
    auto time = this->get_node()->now() - start_time_;
    double delta_angle = M_PI / 8.0 * (1 - std::cos(M_PI / 2.5 * time.seconds()));
    q_goal(3) += delta_angle;
    q_goal(4) += delta_angle;

    const double kAlpha = 0.99;
    arm.dq_filtered_ = (1 - kAlpha) * arm.dq_filtered_ + kAlpha * arm.dq_;
    Vector7d tau_d_calculated =
        arm.k_gains_.cwiseProduct(q_goal - arm.q_) + arm.d_gains_.cwiseProduct(-arm.dq_filtered_);
    for (int i = 0; i < num_joints; ++i) {
      //command_interfaces_[i].set_value(tau_d_calculated(i));
      RCLCPP_INFO(get_node()->get_logger(), "%s", command_interfaces_[i].get_name().c_str());
    }
  }
  return controller_interface::return_type::OK;
}

CallbackReturn DualJointImpedanceExampleController::on_init() {
  try {
    auto_declare<std::string>("arm_1.arm_id", "panda");
    auto_declare<std::vector<double>>("arm_1.k_gains", {});
    auto_declare<std::vector<double>>("arm_1.d_gains", {});
    auto_declare<std::string>("arm_2.arm_id", "panda");
    auto_declare<std::vector<double>>("arm_2.k_gains", {});
    auto_declare<std::vector<double>>("arm_2.d_gains", {});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DualJointImpedanceExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  rclcpp::Parameter arm_id_1_param = this->get_node()->get_parameter("arm_1.arm_id");
  rclcpp::Parameter arm_id_2_param = this->get_node()->get_parameter("arm_2.arm_id");
  if(arm_id_1_param.as_string() == arm_id_2_param.as_string()){
    RCLCPP_FATAL(this->get_node()->get_logger(), "Arms do not have unique ids!");
    return CallbackReturn::ERROR;
  }
  arms_.insert(std::make_pair(arm_id_1_param.as_string(), ArmContainer()));
  arms_.insert(std::make_pair(arm_id_2_param.as_string(), ArmContainer()));
  
  int i = 1;
  for(auto& arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    arm.arm_id_ = arm_container_pair.first;
    auto k_gains = get_node()->get_parameter("arm_" + std::to_string(i) + ".k_gains").as_double_array();
    auto d_gains = get_node()->get_parameter("arm_" + std::to_string(i) + ".d_gains").as_double_array();

    if (k_gains.empty()) {
      RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
      return CallbackReturn::FAILURE;
    }
    if (k_gains.size() != static_cast<uint>(num_joints)) {
      RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
                  num_joints, k_gains.size());
      return CallbackReturn::FAILURE;
    }
    if (d_gains.empty()) {
      RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
      return CallbackReturn::FAILURE;
    }
    if (d_gains.size() != static_cast<uint>(num_joints)) {
      RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                  num_joints, d_gains.size());
      return CallbackReturn::FAILURE;
    }
    for (int i = 0; i < num_joints; ++i) {
      arm.d_gains_(i) = d_gains.at(i);
      arm.k_gains_(i) = k_gains.at(i);
    }
    arm.dq_filtered_.setZero();
    i++;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DualJointImpedanceExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  for(auto& arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    arm.initial_q_ = arm.q_;
  }
  start_time_ = this->get_node()->now();
  return CallbackReturn::SUCCESS;
}

void DualJointImpedanceExampleController::updateJointStates() {
  for(auto& arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    for (auto i = 0; i < num_joints; ++i) {
      const auto& position_interface = state_interfaces_.at(2 * i);
      const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

      assert(position_interface.get_interface_name() == "position");
      assert(velocity_interface.get_interface_name() == "velocity");

      arm.q_(i) = position_interface.get_value();
      arm.dq_(i) = velocity_interface.get_value();
    }
  }
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::DualJointImpedanceExampleController,
                       controller_interface::ControllerInterface)