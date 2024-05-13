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

#include "franka_hardware/franka_mujoco_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <thread>

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

CallbackReturn FrankaMujocoHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
   
  std::lock_guard<std::mutex> guard(mj_mutex_);
  if (info_.joints.size() != kNumberOfJoints) {
    RCLCPP_FATAL(getLogger(), "Got %ld joints. Expected %ld.", info_.joints.size(),
                 kNumberOfJoints);
    return CallbackReturn::ERROR;
  }

  for (const auto& joint : info_.joints) {

    // Check number of command interfaces
    if (joint.command_interfaces.size() != 3) { // 3 if including position
      RCLCPP_FATAL(getLogger(), "Joint '%s' has %ld command interfaces found. 3 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

  //   // Check that the interfaces are named correctly
    for (const auto & cmd_interface : joint.command_interfaces){
      if (cmd_interface.name != hardware_interface::HW_IF_EFFORT &&   // Effort "effort"
          cmd_interface.name != hardware_interface::HW_IF_POSITION && // Joint position "position"
          cmd_interface.name != hardware_interface::HW_IF_VELOCITY //&& // Joint velocity "velocity"
          // cmd_interface.name != "cartesian_position" &&               // Cartesian position
          // cmd_interface.name != "cartesian_velocity"                  // Cartesian velocity
          ){                
        RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected command interface '%s'",
                    joint.name.c_str(), cmd_interface.name.c_str());
        return CallbackReturn::ERROR;
      }
    }

  //   // Check number of state interfaces
    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has %zu state interfaces found. 3 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }
    // if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
    //   RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
    //                joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
    //                hardware_interface::HW_IF_POSITION);
    // }
    for (const auto & state_interface : joint.state_interfaces){
      if (state_interface.name != hardware_interface::HW_IF_EFFORT &&   // Effort "effort"
          state_interface.name != hardware_interface::HW_IF_POSITION && // Joint position "position"
          state_interface.name != hardware_interface::HW_IF_VELOCITY //&& // Joint velocity "velocity"
          // state_interface.name != "cartesian_position" &&               // Cartesian position
          // state_interface.name != "cartesian_velocity"                  // Cartesian velocity
          ){                
        RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected command interface '%s'",
                    joint.name.c_str(), state_interface.name.c_str());
        return CallbackReturn::ERROR;
      }
    }
  }
  RCLCPP_INFO(getLogger(), "Initial checks all passed");
  clock_ = rclcpp::Clock(RCL_ROS_TIME);

  // Set up the mujoco and RobotSim 
  std::string xml_path = info_.hardware_parameters.at("scene_xml");
  bool has_gripper = info_.hardware_parameters.at("hand") == std::string("True") ? true : false;
  std::string mj_robot_name = "panda";
  
  m_ = mj_loadXML(xml_path.c_str(), 0, error, 1000);
  if(!m_){
      std::cerr << error << std::endl;
      throw std::exception();
  }
  d_ = mj_makeData(m_);

  robot_ = std::make_unique<RobotSim>(mj_robot_name, has_gripper);
  robot_->franka_hardware_model_ = std::make_unique<ModelSim>(m_, d_);
  robot_->populateIndices();

  // // Start the service nodes
  if(robot_->has_gripper_){
    gripper_states_ptr_ = std::make_shared<std::array<double, 3>>();
    gripper_action_node_ = std::make_shared<franka_gripper::GripperSimActionServer>(rclcpp::NodeOptions());
    gripper_action_node_->initGripperPtrs(gripper_states_ptr_);
  }
  executor_ = std::make_shared<FrankaExecutor>();
  executor_->add_node(gripper_action_node_);

  mj_visualizer_ = std::make_shared<MujocoVisualizer>(m_, d_);
  mj_thread_ = boost::thread(std::mem_fn(&MujocoVisualizer::startMujoco), mj_visualizer_);
  
  hw_commands_joint_effort_.fill(0);
  hw_commands_joint_position_.fill(0);
  hw_commands_joint_velocity_.fill(0);

  // set the gains on the position and velocity to 0 to disable them at the start
  for(size_t i=0; i<kNumberOfJoints; i++){
    set_torque_control(m_, robot_->act_trq_indices_[i], 0);
    set_position_servo(m_, robot_->act_pos_indices_[i], 0);
    set_velocity_servo(m_, robot_->act_vel_indices_[i], 0);
  }

  mj_resetDataKeyframe(m_, d_, 0); // reset the simulation to the "home" keyframe defined in the scene.xml
  mj_step(m_, d_); // step once to prepare the model and data
  control_mode_ = ControlMode::None;
  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> FrankaMujocoHardwareInterface::export_state_interfaces() {
  std::vector<StateInterface> state_interfaces;

  for (auto i = 0U; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_.at(i)));
    state_interfaces.emplace_back(StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_.at(i)));
    state_interfaces.emplace_back(StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_.at(i)));
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

std::vector<CommandInterface> FrankaMujocoHardwareInterface::export_command_interfaces() {
  std::vector<CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());

  for (auto i = 0U; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(CommandInterface( // JOINT EFFORT
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_joint_effort_.at(i)));
    command_interfaces.emplace_back(CommandInterface( // JOINT POSITION
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_joint_position_.at(i)));
    command_interfaces.emplace_back(CommandInterface( // JOINT VELOCITY
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_joint_velocity_.at(i)));
  }

  return command_interfaces;
}

CallbackReturn FrankaMujocoHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  hw_commands_joint_effort_.fill(0);
  RCLCPP_INFO(getLogger(), "Started");
  return CallbackReturn::SUCCESS;
}

CallbackReturn FrankaMujocoHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(getLogger(), "trying to Stop...");
  // robot_->stopRobot();
  // RCLCPP_INFO(getLogger(), "Stopped"); 
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type FrankaMujocoHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/) {
  std::lock_guard<std::mutex> guard(mj_mutex_);
  mj_step1(m_, d_); // ideally, mj_step1 then mj_step2 in write
  if (hw_franka_model_ptr_ == nullptr) {
    hw_franka_model_ptr_ = robot_->getModel();
  }
  hw_franka_robot_state_ = robot_->populateFrankaState();
  hw_positions_ = hw_franka_robot_state_.q;
  hw_velocities_ = hw_franka_robot_state_.dq;
  hw_efforts_ = hw_franka_robot_state_.tau_J;
  gripper_states_ptr_->at(1) = d_->qpos[robot_->gripper_joint_qpos_indices_[0]];
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaMujocoHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                               const rclcpp::Duration& /*period*/) {
  std::lock_guard<std::mutex> guard(mj_mutex_);
  if (std::any_of(hw_commands_joint_effort_.begin(), hw_commands_joint_effort_.end(),
                  [](double c) { return !std::isfinite(c); })) {
    return hardware_interface::return_type::ERROR;
  }
  if (std::any_of(hw_commands_joint_position_.begin(), hw_commands_joint_position_.end(),
                  [](double c) { return !std::isfinite(c); })) {
    return hardware_interface::return_type::ERROR;
  }
  if (std::any_of(hw_commands_joint_velocity_.begin(), hw_commands_joint_velocity_.end(),
                  [](double c) { return !std::isfinite(c); })) {
    return hardware_interface::return_type::ERROR;
  }

  switch(control_mode_){
    case ControlMode::JointTorque:
      for(size_t i = 0; i < kNumberOfJoints; i++){
        d_->ctrl[robot_->act_trq_indices_[i]] = hw_commands_joint_effort_[i];
      }
      break;
    case ControlMode::JointPosition:
      for(size_t i = 0; i < kNumberOfJoints; i++){
        d_->ctrl[robot_->act_pos_indices_[i]] = hw_commands_joint_position_[i];
      }
      break;
    case ControlMode::JointVelocity:
      for(size_t i = 0; i < kNumberOfJoints; i++){
        d_->ctrl[robot_->act_vel_indices_[i]] = hw_commands_joint_velocity_[i];
      }
      break;
    default:
      // by default, set all controls to 0
      // refine this later by only setting the joint actuators to 0
      for(int i = 0; i < m_->nu; i++){
        d_->ctrl[i] = 0;
      }
      break;
  }
  if(robot_->has_gripper_){
    d_->ctrl[robot_->gripper_act_idx_] = gripper_states_ptr_->at(0);
  }
  mj_step2(m_, d_);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaMujocoHardwareInterface::prepare_command_mode_switch(
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
  
  // //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  // //              Handle the stop case first                    //
  // //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//

  // ////////////////////////////////////////////////////////////////
  // //              Verify that the names are valid               //
  // ////////////////////////////////////////////////////////////////
  int stop_type = check_command_mode_type(stop_interfaces);

  if(stop_type == -1){
    RCLCPP_ERROR(this->getLogger(), "Requested stop interfaces do not all have the same type!\n" \
                                    "Please make sure they are either Cartesian or Joint.");
    return hardware_interface::return_type::ERROR;
  }

  if(stop_type == 2){
    RCLCPP_ERROR(this->getLogger(), "Cartesian interfaces are not supported for simulation!\n" \
                                    "Please make sure they are Joint.");
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
                                  "Please make sure they are either effort or velocity.");
      return hardware_interface::return_type::ERROR;
    }
    if((is_duplicate)){
      RCLCPP_ERROR(this->getLogger(), "Requested stop interface has a confusing name!\n" \
                                  "Please make sure they are either effort or velocity, only.");
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
          hw_commands_joint_effort_[i] = 0;
        }
        // position command is not reset, since that can be dangerous
        else if(is_velocity){
          hw_commands_joint_velocity_[i] = 0;
        }
      }
      control_mode_ = ControlMode::None;
      break;

  //   case 2: // Cartesian is not supported for simulation

    default:
      break;
  }

  // //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  // //              Handle the start case                         //
  // //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  int start_type = check_command_mode_type(start_interfaces);
  
  if(start_type == -1){
    RCLCPP_ERROR(this->getLogger(), "Requested start interfaces do not all have the same type!\n" \
                                    "Please make sure they are either Cartesian or Joint.");
    return hardware_interface::return_type::ERROR;
  }
  if(start_type == 2){
    RCLCPP_ERROR(this->getLogger(), "Cartesian interfaces are not supported for simulation!\n" \
                                    "Please make sure they are Joint.");
    return hardware_interface::return_type::ERROR;
  }
  
  // ////////////////////////////////////////////////////////////////
  // //              Verify that the names are valid               //
  // ////////////////////////////////////////////////////////////////
  if(start_type != 0){
    // If start_type is not empty, i.e. 1 or 2
    is_effort = all_of_element_has_string(start_interfaces, "effort");
    is_position = all_of_element_has_string(start_interfaces, "position");
    is_velocity = all_of_element_has_string(start_interfaces, "velocity");
    is_duplicate = (is_effort && is_position) || (is_effort && is_velocity) || (is_velocity && is_position);
    if(!(is_effort || is_position || is_velocity)){
      RCLCPP_ERROR(this->getLogger(), "Requested start interface is not a supported type!\n" \
                                  "Please make sure they are either effort or velocity.");
      return hardware_interface::return_type::ERROR;
    }
    if((is_duplicate)){
      RCLCPP_ERROR(this->getLogger(), "Requested start interface has a confusing name!\n" \
                                  "Please make sure they are either effort or velocity, only.");
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

  //   case 2: // Cartesian is not supported for simulation
    default:
      break;
  }

  return hardware_interface::return_type::OK;

}

hardware_interface::return_type FrankaMujocoHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/) {

  RCLCPP_INFO(this->getLogger(),"Performing command mode switch");
  std::cout << "Current mode: " << control_mode_ << std::endl;
  
  if(control_mode_ == ControlMode::None){
    for(size_t i=0; i<kNumberOfJoints; i++){
      set_torque_control(m_, robot_->act_trq_indices_[i], 0);
      set_position_servo(m_, robot_->act_pos_indices_[i], 0);
      set_velocity_servo(m_, robot_->act_vel_indices_[i], 0);
    }
  }
  else if(control_mode_ == ControlMode::JointTorque){
    for(size_t i=0; i<kNumberOfJoints; i++){
      set_torque_control(m_, robot_->act_trq_indices_[i], 1);
      set_position_servo(m_, robot_->act_pos_indices_[i], 0);
      set_velocity_servo(m_, robot_->act_vel_indices_[i], 0);
    }
  }
  else if(control_mode_ == ControlMode::JointPosition){
    for(size_t i=0; i<kNumberOfJoints; i++){
      set_torque_control(m_, robot_->act_trq_indices_[i], 0);
      set_position_servo(m_, robot_->act_pos_indices_[i], 100);
      set_velocity_servo(m_, robot_->act_vel_indices_[i], 0);
    }
  }
  else if(control_mode_ == ControlMode::JointVelocity){
    for(size_t i=0; i<kNumberOfJoints; i++){
      set_torque_control(m_, robot_->act_trq_indices_[i], 0);
      set_position_servo(m_, robot_->act_pos_indices_[i], 0);
      set_velocity_servo(m_, robot_->act_vel_indices_[i], 10);
    }
  }

  return hardware_interface::return_type::OK;
}


rclcpp::Logger FrankaMujocoHardwareInterface::getLogger() {
  return rclcpp::get_logger("FrankaMujocoHardwareInterface");
}

}  // namespace franka_hardware

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_hardware::FrankaMujocoHardwareInterface,
                       hardware_interface::SystemInterface)