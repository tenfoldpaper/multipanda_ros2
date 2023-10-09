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

#include <franka_hardware/franka_hardware_interface.hpp>

#include <algorithm>
#include <cmath>
#include <exception>

#include <franka/exception.h>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace franka_hardware {

using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

std::vector<StateInterface> FrankaHardwareInterface::export_state_interfaces() {
  std::vector<StateInterface> state_interfaces;
  // seems like, there is no way to properly format the topic... But is this interface even a topic? 
  // We do see it in the topic list, but it must be handled rather differently.
  for (auto i = 0U; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_.at(i)));
    state_interfaces.emplace_back(StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_.at(i)));
        
    state_interfaces.emplace_back(
        StateInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_.at(i)));
  }

  state_interfaces.emplace_back(StateInterface(
      k_robot_name, k_robot_state_interface_name,
      reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
          &hw_franka_robot_state_addr_)));
  state_interfaces.emplace_back(StateInterface(
      k_robot_name, k_robot_model_interface_name,
      reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
          &hw_franka_model_ptr_)));
  
  return state_interfaces;
}

std::vector<CommandInterface> FrankaHardwareInterface::export_command_interfaces() {
  std::vector<CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());
  for (auto i = 0U; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(CommandInterface( // JOINT EFFORT
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_joint_effort.at(i)));
    command_interfaces.emplace_back(CommandInterface( // JOINT POSITION
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_joint_position.at(i)));
  }

  // Franka ros provides:
  // joint torque, position, velocity
  // cartesian position, velocity 
  // in total, 5 interfaces

  return command_interfaces;
}

CallbackReturn FrankaHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  robot_->initializeContinuousReading();
  hw_commands_joint_effort.fill(0);
  read(rclcpp::Time(0),
       rclcpp::Duration(0, 0));  // makes sure that the robot state is properly initialized.
  RCLCPP_INFO(getLogger(), "Started");
  return CallbackReturn::SUCCESS;
}

CallbackReturn FrankaHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(getLogger(), "trying to Stop...");
  robot_->stopRobot();
  RCLCPP_INFO(getLogger(), "Stopped");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type FrankaHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/) {
  
  if (hw_franka_model_ptr_ == nullptr) {
    hw_franka_model_ptr_ = robot_->getModel();
  }
  hw_franka_robot_state_ = robot_->read(); // readOnce franka_ros2 uses active control, which isn't available for Panda
  hw_positions_ = hw_franka_robot_state_.q;
  hw_velocities_ = hw_franka_robot_state_.dq;
  hw_efforts_ = hw_franka_robot_state_.tau_J;
  // const auto kState = robot_->read(); // kState is basically franka state, so why wasn't frankastate just implemented?
  // hw_positions_ = kState.q;
  // hw_velocities_ = kState.dq;
  // hw_efforts_ = kState.tau_J;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                               const rclcpp::Duration& /*period*/) {
  if (std::any_of(hw_commands_joint_effort.begin(), hw_commands_joint_effort.end(),
                  [](double c) { return !std::isfinite(c); })) {
    return hardware_interface::return_type::ERROR;
  }
  if (std::any_of(hw_commands_joint_position.begin(), hw_commands_joint_position.end(),
                  [](double c) { return !std::isfinite(c); })) {
    return hardware_interface::return_type::ERROR;
  }

  robot_->write(hw_commands_joint_effort, hw_commands_joint_position);
  return hardware_interface::return_type::OK;
}

CallbackReturn FrankaHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (info_.joints.size() != kNumberOfJoints) {
    RCLCPP_FATAL(getLogger(), "Got %ld joints. Expected %ld.", info_.joints.size(),
                 kNumberOfJoints);
    return CallbackReturn::ERROR;
  }

  for (const auto& joint : info_.joints) {

    // Check number of command interfaces
    if (joint.command_interfaces.size() != 2) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has %zu command interfaces found. 2 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    // Check that the interfaces are named correctly
    for (const auto & cmd_interface : joint.command_interfaces){
      if (cmd_interface.name != hardware_interface::HW_IF_EFFORT &&   // Effort "effort"
          cmd_interface.name != hardware_interface::HW_IF_POSITION) { // Joint position "position"
        RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected command interface '%s'",
                    joint.name.c_str(), cmd_interface.name.c_str());
        return CallbackReturn::ERROR;
      }
    }

    // Check number of state interfaces
    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has %zu state interfaces found. 3 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_VELOCITY);
    }
    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_EFFORT);
    }
  }
  std::string robot_ip;
  try {
    robot_ip = info_.hardware_parameters.at("robot_ip");
  } catch (const std::out_of_range& ex) {
    RCLCPP_FATAL(getLogger(), "Parameter 'robot_ip' ! set");
    return CallbackReturn::ERROR;
  }
  try {
    RCLCPP_INFO(getLogger(), "Connecting to robot at \"%s\" ...", robot_ip.c_str());
    robot_ = std::make_unique<Robot>(robot_ip, getLogger());
  } catch (const franka::Exception& e) {
    RCLCPP_FATAL(getLogger(), "Could ! connect to robot");
    RCLCPP_FATAL(getLogger(), "%s", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(getLogger(), "Successfully connected to robot");

  // executor is tied to the param_service_server, for setting stiffness and stuff. So that will be added later on.
  return CallbackReturn::SUCCESS;
}

rclcpp::Logger FrankaHardwareInterface::getLogger() {
  return rclcpp::get_logger("FrankaHardwareInterface");
}

hardware_interface::return_type FrankaHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/) {

  RCLCPP_INFO(this->getLogger(),"Performing command mode switch");
  if(control_mode_[0] == ControlMode::None){
    robot_->stopRobot();
    robot_->initializeContinuousReading();
  }
  else if(control_mode_[0] == ControlMode::JointTorque){
    robot_->stopRobot();
    robot_->initializeTorqueControl();
  }
  else if(control_mode_[0] == ControlMode::JointPosition){
    robot_->stopRobot();
    robot_->initializeJointPositionControl();
  }
//  if (!effort_interface_running_ && effort_interface_claimed_) {
//     robot_->stopRobot();
//     robot_->initializeTorqueControl();
//     effort_interface_running_ = true;
//   } else if (effort_interface_running_ && !effort_interface_claimed_) {
//     robot_->stopRobot();
//     robot_->initializeContinuousReading();
//     effort_interface_running_ = false;
//   }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  RCLCPP_INFO(this->getLogger(),"Preparing command mode switch");
  for(auto& start_item : start_interfaces){
    RCLCPP_INFO(this->getLogger(),"++ %s", start_item.c_str());
  }
  for(auto& stop_item : stop_interfaces){
    RCLCPP_INFO(this->getLogger(),"-- %s", stop_item.c_str());
  }

  std::vector<ControlMode> new_modes = {};

  // Initialize the new_modes
  for (std::string key : start_interfaces){
    for (std::size_t i =0; i< info_.joints.size(); i++){
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT)
      {
        new_modes.push_back(ControlMode::JointTorque);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION)
      {
        new_modes.push_back(ControlMode::JointPosition);
      }

    }
  }
  // do we need a separate list later?
  if(new_modes.size() == 0){
    for (std::string key : stop_interfaces){
      for (std::size_t i =0; i< info_.joints.size(); i++){
        if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT)
        {
          new_modes.push_back(ControlMode::None); 
        }
        if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION)
        {
          new_modes.push_back(ControlMode::None);
        }
      }
    }
  }

  RCLCPP_FATAL(this->getLogger(),"%zu", new_modes.size());

  // Checks before proceeding
  // All joints must be given new command mode at the same time
  if (new_modes.size() != info_.joints.size())
  {
    RCLCPP_FATAL(this->getLogger(),"Mode size not equal to joint size!");
    return hardware_interface::return_type::ERROR;
  }
  // All joints must have the same command mode
  if (!std::all_of(
        new_modes.begin() + 1, new_modes.end(),
        [&](ControlMode mode) { return mode == new_modes[0]; }))
  {
    RCLCPP_FATAL(this->getLogger(),"Joints do not all have the same mode!");
    return hardware_interface::return_type::ERROR;
  }

  // Now that checks have passed, do the actual work.
  // Stop motion on all relevant joints that are stopping
  for (std::string key : stop_interfaces)
  {
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      if (key.find(info_.joints[i].name) != std::string::npos)
      {
        hw_commands_joint_effort[i] = 0;
        hw_commands_joint_position[i] = 0;
        control_mode_[i] = ControlMode::None;  // Revert to undefined
      }
    }
  }

  // Set the new command modes
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    if (control_mode_[i] != ControlMode::None)
    {
      // Something else is using the joint! Abort!
      RCLCPP_FATAL(this->getLogger(),"Joint %ld is already claimed as mode %d", i+1, int(control_mode_[i]));
      return hardware_interface::return_type::ERROR;
    }
    control_mode_[i] = new_modes[i];
  }

  RCLCPP_INFO(this->getLogger(), "Joints mode: %d %d %d %d %d %d %d ", int(control_mode_[0]),int(control_mode_[1]),int(control_mode_[2]),int(control_mode_[3]),int(control_mode_[4]),int(control_mode_[5]),int(control_mode_[6]));
  return hardware_interface::return_type::OK;

  // auto is_effort_interface = [](const std::string& interface) {
  //   return interface.find(hardware_interface::HW_IF_EFFORT) != std::string::npos;
  // };

  // int64_t num_stop_effort_interfaces =
  //     std::count_if(stop_interfaces.begin(), stop_interfaces.end(), is_effort_interface);
  // if (num_stop_effort_interfaces == kNumberOfJoints) {
  //   effort_interface_claimed_ = false;
  // } else if (num_stop_effort_interfaces != 0) {
  //   RCLCPP_FATAL(this->getLogger(), "Expected %ld effort interfaces to stop, but got %ld instead.",
  //                kNumberOfJoints, num_stop_effort_interfaces);
  //   std::string error_string = "Invalid number of effort interfaces to stop. Expected ";
  //   error_string += std::to_string(kNumberOfJoints);
  //   throw std::invalid_argument(error_string);
  // }

  // int64_t num_start_effort_interfaces =
  //     std::count_if(start_interfaces.begin(), start_interfaces.end(), is_effort_interface);
  // if (num_start_effort_interfaces == kNumberOfJoints) {
  //   effort_interface_claimed_ = true;
  // } else if (num_start_effort_interfaces != 0) {
  //   RCLCPP_FATAL(this->getLogger(), "Expected %ld effort interfaces to start, but got %ld instead.",
  //                kNumberOfJoints, num_start_effort_interfaces);
  //   std::string error_string = "Invalid number of effort interfaces to start. Expected ";
  //   error_string += std::to_string(kNumberOfJoints);
  //   throw std::invalid_argument(error_string);
  // }
  return hardware_interface::return_type::OK;
}
}  // namespace franka_hardware

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_hardware::FrankaHardwareInterface,
                       hardware_interface::SystemInterface)