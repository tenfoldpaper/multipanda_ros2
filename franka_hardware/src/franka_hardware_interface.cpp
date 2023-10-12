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

  for (auto i = 0U; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_.at(i)));
    state_interfaces.emplace_back(StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_.at(i)));
    state_interfaces.emplace_back(StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_.at(i)));
  }

  for (auto i = 0; i < 16; i++){
    state_interfaces.emplace_back(StateInterface(
        "ee_cartesian_position", cartesian_matrix_names[i], &hw_cartesian_positions_[i]));
    state_interfaces.emplace_back(StateInterface(
        "ee_cartesian_velocity", cartesian_matrix_names[i], &hw_cartesian_velocities_[i]));
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
  RCLCPP_INFO(getLogger(), "%ld", info_.joints.size());

  for (auto i = 0U; i < info_.joints.size(); i++) {
    RCLCPP_INFO(getLogger(), "%s", info_.joints[i].name.c_str());
    command_interfaces.emplace_back(CommandInterface( // JOINT EFFORT
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_joint_effort.at(i)));
    command_interfaces.emplace_back(CommandInterface( // JOINT POSITION
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_joint_position.at(i)));
    command_interfaces.emplace_back(CommandInterface( // JOINT VELOCITY
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_joint_velocity.at(i)));
  }
  for (auto i = 0; i < 16; i++){
    command_interfaces.emplace_back(CommandInterface(
        "ee_cartesian_position", cartesian_matrix_names[i], &hw_commands_cartesian_position[i]));
  }
  for (auto i = 0; i < 6; i++){
    command_interfaces.emplace_back(CommandInterface(
        "ee_cartesian_velocity", cartesian_velocity_command_names[i], &hw_commands_cartesian_velocity[i]));
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
  hw_franka_robot_state_ = robot_->read();
  hw_positions_ = hw_franka_robot_state_.q;
  hw_velocities_ = hw_franka_robot_state_.dq;
  hw_efforts_ = hw_franka_robot_state_.tau_J;
  hw_cartesian_positions_ = hw_franka_robot_state_.O_T_EE;
  hw_cartesian_velocities_ = hw_franka_robot_state_.O_T_EE_d;
  
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
  if (std::any_of(hw_commands_joint_velocity.begin(), hw_commands_joint_velocity.end(),
                  [](double c) { return !std::isfinite(c); })) {
    return hardware_interface::return_type::ERROR;
  }
  if (std::any_of(hw_commands_cartesian_position.begin(), hw_commands_cartesian_position.end(),
                  [](double c) { return !std::isfinite(c); })) {
    return hardware_interface::return_type::ERROR;
  }
  if (std::any_of(hw_commands_cartesian_velocity.begin(), hw_commands_cartesian_velocity.end(),
                  [](double c) { return !std::isfinite(c); })) {
    return hardware_interface::return_type::ERROR;
  }

  robot_->write(hw_commands_joint_effort, 
                hw_commands_joint_position, 
                hw_commands_joint_velocity, 
                hw_commands_cartesian_position, 
                hw_commands_cartesian_velocity);

  if(robot_->hasError()){
    return hardware_interface::return_type::ERROR;
  }
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
    if (joint.command_interfaces.size() != 3) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has %ld command interfaces found. 3 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    // Check that the interfaces are named correctly
    for (const auto & cmd_interface : joint.command_interfaces){
      if (cmd_interface.name != hardware_interface::HW_IF_EFFORT &&   // Effort "effort"
          cmd_interface.name != hardware_interface::HW_IF_POSITION && // Joint position "position"
          cmd_interface.name != hardware_interface::HW_IF_VELOCITY && // Joint velocity "velocity"
          cmd_interface.name != "cartesian_position" &&               // Cartesian position
          cmd_interface.name != "cartesian_velocity"){                // Cartesian velocity
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

  // Start the service nodes
  error_recovery_service_node_ = std::make_shared<FrankaErrorRecoveryServiceServer>(rclcpp::NodeOptions(), robot_);
  param_service_node_ = std::make_shared<FrankaParamServiceServer>(rclcpp::NodeOptions(), robot_);
  executor_ = std::make_shared<FrankaExecutor>();
  executor_->add_node(error_recovery_service_node_);
  executor_->add_node(param_service_node_);
  

  // Init the cartesian values to 0
  hw_cartesian_positions_.fill({});
  hw_cartesian_velocities_.fill({});
  hw_commands_cartesian_position.fill({});
  hw_commands_cartesian_velocity.fill({});
  return CallbackReturn::SUCCESS;
}

rclcpp::Logger FrankaHardwareInterface::getLogger() {
  return rclcpp::get_logger("FrankaHardwareInterface");
}

hardware_interface::return_type FrankaHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/) {

  RCLCPP_INFO(this->getLogger(),"Performing command mode switch");
  if(control_mode_ == ControlMode::None){
    robot_->stopRobot();
    robot_->initializeContinuousReading();
  }
  else if(control_mode_ == ControlMode::JointTorque){
    robot_->stopRobot();
    robot_->initializeTorqueControl();
  }
  else if(control_mode_ == ControlMode::JointPosition){
    robot_->stopRobot();
    robot_->initializeJointPositionControl();
  }
  else if(control_mode_ == ControlMode::JointVelocity){
    robot_->stopRobot();
    robot_->initializeJointVelocityControl();
  }
  else if(control_mode_ == ControlMode::CartesianPose){
    robot_->stopRobot();
    robot_->initializeCartesianPositionControl();
  }
  else if(control_mode_ == ControlMode::CartesianVelocity){
    robot_->stopRobot();
    robot_->initializeCartesianVelocityControl();
  }
  

  return hardware_interface::return_type::OK;
}

bool all_of_element_has_string(std::vector<std::string> vec, std::string content){
  if(vec.size() == 0){
    return false;
  }
  return std::all_of(
        vec.begin(), vec.end(),
        [&](std::string elem) { return elem.find(content) != std::string::npos; });
  
}
// bool all_of_element_are_same(std::vector<std::string> vec){
//   // Checks that all the elements are identical
//   return std::all_of(
//         vec.begin()+1, vec.end(),
//         [&](std::string elem) { return vec[0] == elem; });
// }
int check_command_mode_type(std::vector<std::string> interfaces){
  if(interfaces.size() != 0){
    bool is_cartesian = all_of_element_has_string(interfaces, "ee_cartesian");
    bool is_joint = all_of_element_has_string(interfaces, "joint");
    if(!(is_cartesian || is_joint)){
      return -1;
    }
    if(is_joint){
      return 1;
    }
    if(is_cartesian){
      return 2;
    }
  }
  else{
    return 0;
  }
  return -1;
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
  
  bool is_effort;
  bool is_position;
  bool is_velocity;
  bool is_duplicate;
  
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  //              Handle the stop case first                    //
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//

  ////////////////////////////////////////////////////////////////
  //              Verify that the names are valid               //
  ////////////////////////////////////////////////////////////////
  int stop_type = check_command_mode_type(stop_interfaces);

  if(stop_type == -1){
    RCLCPP_ERROR(this->getLogger(), "Requested stop interfaces do not all have the same type!\n" \
                                    "Please make sure they are either Cartesian or Joint.");
    return hardware_interface::return_type::ERROR;
  }

  if(stop_type != 0){
    // If stop_type is not empty, i.e. 1 or 2
    is_effort = all_of_element_has_string(stop_interfaces, "effort");
    is_position = all_of_element_has_string(stop_interfaces, "position");
    is_velocity = all_of_element_has_string(stop_interfaces, "velocity");
    is_duplicate = (is_effort && is_position) || (is_effort && is_velocity) || (is_velocity && is_position);
    if(!(is_effort || is_position || is_velocity)){
      RCLCPP_ERROR(this->getLogger(), "Requested stop interface is not a supported type!\n" \
                                  "Please make sure they are either effort, position or velocity.");
      return hardware_interface::return_type::ERROR;
    }
    if((is_duplicate)){
      RCLCPP_ERROR(this->getLogger(), "Requested stop interface has a confusing name!\n" \
                                  "Please make sure they are either effort (for joints), position or velocity, only.");
      return hardware_interface::return_type::ERROR;
    }
  }

  switch(stop_type){
    case 1: // stop the joint controllers
      // Make sure there are 7
      if(stop_interfaces.size() != kNumberOfJoints){
        RCLCPP_ERROR(this->getLogger(), "Requested joint stop interface's size is not 7 (got %ld)", stop_interfaces.size());
        return hardware_interface::return_type::ERROR;
      }
      
      for(size_t i = 0 ; i < kNumberOfJoints; i++){
        if(is_effort){
          hw_commands_joint_effort[i] = 0;
        }
        // position command is not reset, since that can be dangerous
        else if(is_velocity){
          hw_commands_joint_velocity[i] = 0;
        }
      }
      control_mode_ = ControlMode::None;
      break;

    case 2: // stop the cartesian controllers
      if(is_position){
        if(stop_interfaces.size() != 16){
          RCLCPP_ERROR(this->getLogger(), "Requested Cartesian position stop interface's size is not 16 (got %ld)", stop_interfaces.size());
          return hardware_interface::return_type::ERROR;
        }
      }
      if(is_velocity){
        if(stop_interfaces.size() != 6){
          RCLCPP_ERROR(this->getLogger(), "Requested Cartesian velocity stop interface's size is not 6 (got %ld)", stop_interfaces.size());
          return hardware_interface::return_type::ERROR;

        }
        // set the commands to zero
        for(int i = 0; i < 6; i++){
          hw_commands_cartesian_velocity[i] = 0;
        }
      }
      control_mode_ = ControlMode::None;
      break;

    default:
      break;
  }

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  //              Handle the start case                         //
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  int start_type = check_command_mode_type(start_interfaces);
  
  if(start_type == -1){
    RCLCPP_ERROR(this->getLogger(), "Requested start interfaces do not all have the same type!\n" \
                                    "Please make sure they are either Cartesian or Joint.");
    return hardware_interface::return_type::ERROR;
  }
  ////////////////////////////////////////////////////////////////
  //              Verify that the names are valid               //
  ////////////////////////////////////////////////////////////////
  if(start_type != 0){
    // If start_type is not empty, i.e. 1 or 2
    is_effort = all_of_element_has_string(start_interfaces, "effort");
    is_position = all_of_element_has_string(start_interfaces, "position");
    is_velocity = all_of_element_has_string(start_interfaces, "velocity");
    is_duplicate = (is_effort && is_position) || (is_effort && is_velocity) || (is_velocity && is_position);
    if(!(is_effort || is_position || is_velocity)){
      RCLCPP_ERROR(this->getLogger(), "Requested start interface is not a supported type!\n" \
                                  "Please make sure they are either effort (for joints), position or velocity.");
      return hardware_interface::return_type::ERROR;
    }
    if((is_duplicate)){
      RCLCPP_ERROR(this->getLogger(), "Requested start interface has a confusing name!\n" \
                                  "Please make sure they are either effort (for joints), position or velocity, only.");
      return hardware_interface::return_type::ERROR;
    }
  }

  // Just in case: only allow new controller to be started if the current control mode is NONE
  if(control_mode_ != ControlMode::None){
    RCLCPP_ERROR(this->getLogger(), "Switching between control modes without stopping it first is not supported.\n"\
                                    "Please stop the running controller first.");
    return hardware_interface::return_type::ERROR;
  }

  switch(start_type){
    case 1:
      if(start_interfaces.size() != kNumberOfJoints){
        RCLCPP_ERROR(this->getLogger(), "Requested joint start interface's size is not 7 (got %ld)", start_interfaces.size());
        return hardware_interface::return_type::ERROR;
      }
      if(is_effort){
        control_mode_ = ControlMode::JointTorque;
      }
      if(is_position){
        control_mode_ = ControlMode::JointPosition;
      }
      if(is_velocity){
        control_mode_ = ControlMode::JointVelocity;
      }
      break;

    case 2:

      if(start_interfaces.size() != 16U && start_interfaces.size() != 6U){
        RCLCPP_ERROR(this->getLogger(), "Requested Cartesian start interface's size is not 6 nor 16 (got %ld)", start_interfaces.size());
        return hardware_interface::return_type::ERROR;
      }
      if(is_position){
        control_mode_ = ControlMode::CartesianPose;
      }
      if(is_velocity){
        control_mode_ = ControlMode::CartesianVelocity;
      }
      break;

    default:
      break;
  }

  return hardware_interface::return_type::OK;

}
}  // namespace franka_hardware

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_hardware::FrankaHardwareInterface,
                       hardware_interface::SystemInterface)