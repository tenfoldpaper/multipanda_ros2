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

#include "franka_mj_hardware/franka_mj_hardware_system.hpp"

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

#include <yaml-cpp/yaml.h> 
#include <fstream>

namespace franka_hardware {

using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

bool FrankaMjHardwareSystem::initSim(rclcpp::Node::SharedPtr & model_nh,
    const hardware_interface::HardwareInfo & hardware_info,
    const mjModel* m,
    mjData* d,
    int & update_rate){
    
    info_ = hardware_info;
    this->nh_ = model_nh;
    this->d_ = d;
    this->m_ = m;
    this->update_rate_ = &update_rate;
    std::stringstream rc_stream(info_.hardware_parameters.at("robot_count"));
    rc_stream >> robot_count_;

    if (info_.joints.size() != kNumberOfJoints * robot_count_) {
        RCLCPP_FATAL(getLogger(), "Got %ld joints. Expected %ld.", info_.joints.size(),
                    kNumberOfJoints * robot_count_);
        return false;
    }

    for (const auto& joint : info_.joints) {

        // Check number of command interfaces
        if (joint.command_interfaces.size() != 3) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has %ld command interfaces found. 3 expected.",
                        joint.name.c_str(), joint.command_interfaces.size());
            return false;
        }

    //   // Check that the interfaces are named correctly
        for (const auto & cmd_interface : joint.command_interfaces){
            if (cmd_interface.name != hardware_interface::HW_IF_EFFORT &&   // Effort "effort"
                cmd_interface.name != hardware_interface::HW_IF_POSITION && // Joint position "position"
                cmd_interface.name != hardware_interface::HW_IF_VELOCITY //&& // Joint velocity "velocity"
                ){                
                RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected command interface '%s'",
                            joint.name.c_str(), cmd_interface.name.c_str());
                return false;
            }
        }

    //   // Check number of state interfaces
        if (joint.state_interfaces.size() != 3) {
            RCLCPP_FATAL(getLogger(), "Joint '%s' has %zu state interfaces found. 3 expected.",
                        joint.name.c_str(), joint.state_interfaces.size());
            return false;
        }

        for (const auto & state_interface : joint.state_interfaces){
            if (state_interface.name != hardware_interface::HW_IF_EFFORT &&   // Effort "effort"
                state_interface.name != hardware_interface::HW_IF_POSITION && // Joint position "position"
                state_interface.name != hardware_interface::HW_IF_VELOCITY //&& // Joint velocity "velocity"
                ){                
                RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected command interface '%s'",
                            joint.name.c_str(), state_interface.name.c_str());
                return false;
            }
        }
    }
    RCLCPP_INFO(getLogger(), "Initial checks all passed");
    clock_ = rclcpp::Clock(RCL_ROS_TIME);

    // Set up the mujoco and RobotSim 
    executor_ = std::make_shared<FrankaExecutor>();
    for(size_t i = 1; i <= robot_count_; i++){
        // Setup arm container
        std::string suffix = "_" + std::to_string(i);
        std::string mj_robot_name;
        try{
            mj_robot_name = info_.hardware_parameters.at("ns"+suffix);
        }
        catch(const std::out_of_range& ex){
            RCLCPP_FATAL(getLogger(), "Parameter 'ns%s' ! set\nMake sure all multi-robot parameters follow the format {ns/robot_ip/etc.}_n", suffix.c_str());
            return false;
        }
        if(!arms_.insert(std::make_pair(mj_robot_name, ArmContainer())).second){
            RCLCPP_FATAL(getLogger(),"The provided robot namespace %s already exists! Make sure they are unique.", mj_robot_name.c_str());
            return false;
        }
        auto &arm = arms_[mj_robot_name];
        state_pointers_.insert(std::make_pair(mj_robot_name, &arm.hw_franka_robot_state_));
        model_pointers_.insert(std::make_pair(mj_robot_name, nullptr));
        arm.robot_name_ = mj_robot_name;

        bool has_gripper = info_.hardware_parameters.at("hand" + suffix) == std::string("True") ? true : false;
        arm.robot_ = std::make_unique<RobotSim>(mj_robot_name, has_gripper);
        arm.robot_->franka_hardware_model_ = std::make_unique<ModelSim>(m_, d_);
        arm.robot_->populateIndices();

        // Start the gripper node if true
        if(arm.robot_->has_gripper_){
            gripper_states_ptrs_.insert(std::make_pair(arm.robot_name_, std::make_shared<std::array<double,3>>()));
            gripper_nodes_.insert(std::make_pair(arm.robot_name_, std::make_shared<franka_gripper::GripperSimActionServer>(rclcpp::NodeOptions(), arm.robot_name_)));
            gripper_nodes_[arm.robot_name_]->initGripperPtrs(gripper_states_ptrs_[arm.robot_name_]);
            executor_->add_node(gripper_nodes_[arm.robot_name_]);
        }

        // Initialize with all the actuators set to "off"
        for(size_t i=0; i<kNumberOfJoints; i++){
            set_torque_control(m_, arm.robot_->act_trq_indices_[i], 0);
            set_position_servo(m_, arm.robot_->act_pos_indices_[i], 0);
            set_velocity_servo(m_, arm.robot_->act_vel_indices_[i], 0);
        }
        arm.hw_commands_joint_effort_.fill(0);
        arm.hw_commands_joint_velocity_.fill(0);
    }
    
    return true;
  }
CallbackReturn FrankaMjHardwareSystem::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}



std::vector<StateInterface> FrankaMjHardwareSystem::export_state_interfaces() {
  std::vector<StateInterface> state_interfaces;

  for (auto i = 0U; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &arms_[get_ns(info_.joints[i].name)].hw_positions_.at(get_joint_no(info_.joints[i].name))));
    state_interfaces.emplace_back(StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,  &arms_[get_ns(info_.joints[i].name)].hw_velocities_.at(get_joint_no(info_.joints[i].name))));
    state_interfaces.emplace_back(StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT,  &arms_[get_ns(info_.joints[i].name)].hw_efforts_.at(get_joint_no(info_.joints[i].name))));
  }

  for(auto arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    state_interfaces.emplace_back(StateInterface(
        arm.robot_name_, k_robot_state_interface_name,
        reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
            &state_pointers_[arm_container_pair.first])));
    state_interfaces.emplace_back(StateInterface(
        arm.robot_name_, k_robot_model_interface_name,
        reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
            &model_pointers_[arm_container_pair.first])));
  }
  return state_interfaces;
}

std::vector<CommandInterface> FrankaMjHardwareSystem::export_command_interfaces() {
  std::vector<CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());

  for (auto i = 0U; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(CommandInterface( // JOINT EFFORT
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &arms_[get_ns(info_.joints[i].name)].hw_commands_joint_effort_.at(get_joint_no(info_.joints[i].name))));
    command_interfaces.emplace_back(CommandInterface( // JOINT POSITION
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &arms_[get_ns(info_.joints[i].name)].hw_commands_joint_position_.at(get_joint_no(info_.joints[i].name))));
    command_interfaces.emplace_back(CommandInterface( // JOINT VELOCITY
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &arms_[get_ns(info_.joints[i].name)].hw_commands_joint_velocity_.at(get_joint_no(info_.joints[i].name))));
  }

  return command_interfaces;
}

CallbackReturn FrankaMjHardwareSystem::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  for(auto& arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    arm.hw_commands_joint_effort_.fill(0);
    arm.control_mode_ = ControlMode::None;
  }
  RCLCPP_INFO(getLogger(), "Started");
  return CallbackReturn::SUCCESS;
}

CallbackReturn FrankaMjHardwareSystem::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(getLogger(), "trying to Stop...");
  for(auto& arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    arm.control_mode_ = ControlMode::None;
  }
  RCLCPP_INFO(getLogger(), "Stopped"); 
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type FrankaMjHardwareSystem::read(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/) {

//   mj_step1(m_, d_); // ideally, mj_step1 then mj_step2 in write
  for(auto& arm_container_pair: arms_){
    auto &arm = arm_container_pair.second;
    if (model_pointers_[arm_container_pair.first] == nullptr) {
      model_pointers_[arm_container_pair.first] = arm.robot_->getModel();
    }
    arm.hw_franka_robot_state_ = arm.robot_->populateFrankaState();
    arm.hw_positions_ = arm.hw_franka_robot_state_.q;
    arm.hw_velocities_ = arm.hw_franka_robot_state_.dq;
    arm.hw_efforts_ = arm.hw_franka_robot_state_.tau_J;
    gripper_states_ptrs_[arm.robot_name_]->at(1) = d_->qpos[arm.robot_->gripper_joint_qpos_indices_[0]];
  }
//   for(auto& obj_pair: mj_objs_){
//     updateObjectContainer(obj_pair.first);
//   }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaMjHardwareSystem::write(const rclcpp::Time& /*time*/,
                                                               const rclcpp::Duration& /*period*/) {
//   std::lock_guard<std::mutex> guard(mj_mutex_);
  for(auto& arm_container_pair: arms_){
    auto &arm = arm_container_pair.second;
    if (std::any_of(arm.hw_commands_joint_effort_.begin(), arm.hw_commands_joint_effort_.end(),
                    [](double c) { return !std::isfinite(c); })) {
      return hardware_interface::return_type::ERROR;
    }
    if (std::any_of(arm.hw_commands_joint_position_.begin(), arm.hw_commands_joint_position_.end(),
                    [](double c) { return !std::isfinite(c); })) {
      return hardware_interface::return_type::ERROR;
    }
    if (std::any_of(arm.hw_commands_joint_velocity_.begin(), arm.hw_commands_joint_velocity_.end(),
                    [](double c) { return !std::isfinite(c); })) {
      return hardware_interface::return_type::ERROR;
    }

    switch(arm.control_mode_){
      case ControlMode::JointTorque:
        for(size_t i = 0; i < kNumberOfJoints; i++){
          d_->ctrl[arm.robot_->act_trq_indices_[i]] = arm.hw_commands_joint_effort_[i];
        }
        break;
      case ControlMode::JointPosition:
        for(size_t i = 0; i < kNumberOfJoints; i++){
          d_->ctrl[arm.robot_->act_pos_indices_[i]] = arm.hw_commands_joint_position_[i];
        }
        break;
      case ControlMode::JointVelocity:
        for(size_t i = 0; i < kNumberOfJoints; i++){
          d_->ctrl[arm.robot_->act_vel_indices_[i]] = arm.hw_commands_joint_velocity_[i];
        }
        break;
      default:
        break;
    }
    if(arm.robot_->has_gripper_){
      d_->ctrl[arm.robot_->gripper_act_idx_] = gripper_states_ptrs_[arm.robot_name_]->at(0);
    }
  }
//   if(mj_pose_service_node_->hasUpdates()){
//     auto new_object_poses = mj_pose_service_node_->getNewObjectPoses();
//     for(auto& new_obj : new_object_poses){
//       changeMjObjPose(new_obj.obj_name_,new_obj.x,new_obj.y,new_obj.z,new_obj.qx,new_obj.qy,new_obj.qz,new_obj.qw);
//     }
//     mj_pose_service_node_->setHasUpdatesToFalse();
//   }
//   mj_step2(m_, d_);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaMjHardwareSystem::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  RCLCPP_INFO(this->getLogger(),"Preparing command mode switch");

  bool is_effort;
  bool is_position;
  bool is_velocity;
  bool is_duplicate;
  
  // //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  // //              Handle the stop case first                    //
  // //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  // selective control mode handling
  for(auto& arm : arms_){
    std::vector<std::string> arm_stop_interfaces;

    // query what is the requested stop interface's arm
    for(auto& stop : stop_interfaces){
      if(stop.find(arm.first) != std::string::npos){
        arm_stop_interfaces.push_back(stop);
      }
    }
    int stop_type = check_command_mode_type(arm_stop_interfaces);
    if(stop_type > 0){
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
                                    "Please make sure they are either effort, position or velocity, only.");
        return hardware_interface::return_type::ERROR;
      }
    }
    switch(stop_type){
      case -1: 
        RCLCPP_ERROR(this->getLogger(), "Requested stop interfaces do not all have the same type!\n" \
                                      "Please make sure they are either Cartesian or Joint.");
        return hardware_interface::return_type::ERROR;
      
      case 0: // if size is 0, move on to the next arm
        continue;
        break;

      case 1: // stop the joint controllers
        // Make sure there are 7
        if(arm_stop_interfaces.size() != kNumberOfJoints){
          RCLCPP_ERROR(this->getLogger(), "Requested joint stop interface's size is not 7 (got %ld)", arm_stop_interfaces.size());
          return hardware_interface::return_type::ERROR;
        }
        for(size_t i = 0 ; i < kNumberOfJoints; i++){
          if(is_effort){
            arm.second.hw_commands_joint_effort_[i] = 0;
          }
          // position command is not reset, since that can be dangerous
          else if(is_velocity){
            arm.second.hw_commands_joint_velocity_[i] = 0;
          }
        }
        arm.second.control_mode_ = ControlMode::None;
        arm.second.switch_cm_ = true;
        break;

      case 2: // Cartesian is not supported for simulation
        RCLCPP_ERROR(this->getLogger(), "Cartesian interfaces are not supported for simulation!\n" \
                                      "Please make sure they are Joint.");
        return hardware_interface::return_type::ERROR;

      default:
        break;
    }
  }

  // ////////////////////////////////////////////////////////////////
  // //              Verify that the names are valid               //
  // ////////////////////////////////////////////////////////////////

  // //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  // //              Handle the start case                         //
  // //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
  
  for(auto& arm : arms_){
    std::vector<std::string> arm_start_interfaces;

    // query what is the requested stop interface's arm
    for(auto& start : start_interfaces){
      if(start.find(arm.first) != std::string::npos){
        arm_start_interfaces.push_back(start);
      }
    }
    int start_type = check_command_mode_type(arm_start_interfaces);
    // ////////////////////////////////////////////////////////////////
    // //              Verify that the names are valid               //
    // ////////////////////////////////////////////////////////////////
    if(start_type > 0){
      // If start_type is not empty, i.e. 1 or 2
      is_effort = all_of_element_has_string(start_interfaces, "effort");
      is_position = all_of_element_has_string(start_interfaces, "position");
      is_velocity = all_of_element_has_string(start_interfaces, "velocity");
      is_duplicate = (is_effort && is_position) || (is_effort && is_velocity) || (is_velocity && is_position);
      if(!(is_effort || is_position || is_velocity)){
        RCLCPP_ERROR(this->getLogger(), "Requested start interface is not a supported type!\n" \
                                    "Please make sure they are either effort, position or velocity.");
        return hardware_interface::return_type::ERROR;
      }
      if((is_duplicate)){
        RCLCPP_ERROR(this->getLogger(), "Requested start interface has a confusing name!\n" \
                                    "Please make sure they are either effort, position or velocity, only.");
        return hardware_interface::return_type::ERROR;
      }
      // Just in case: only allow new controller to be started if the current control mode is NONE
      if(arm.second.control_mode_ != ControlMode::None){
        RCLCPP_ERROR(this->getLogger(), "Switching between control modes without stopping it first is not supported.\n"\
                                        "Please stop the running controller first.");
        return hardware_interface::return_type::ERROR;
      }
    }
    switch(start_type){
      case 0: // if size is 0, move on to the next arm
        continue;
        break;

      case -1:
        RCLCPP_ERROR(this->getLogger(), "Requested start interfaces do not all have the same type!\n" \
                                    "Please make sure they are either Cartesian or Joint.");
        return hardware_interface::return_type::ERROR;
      case 1:
        if(arm_start_interfaces.size() != kNumberOfJoints){
          RCLCPP_ERROR(this->getLogger(), "Requested joint start interface's size is not %ld (got %ld)", kNumberOfJoints, arm_start_interfaces.size());
          return hardware_interface::return_type::ERROR;
        }
        if(is_effort){
          arm.second.control_mode_ = ControlMode::JointTorque;
          arm.second.switch_cm_ = true;
        }
        if(is_position){
          arm.second.control_mode_ = ControlMode::JointPosition;
          arm.second.switch_cm_ = true;
        }
        if(is_velocity){
          arm.second.control_mode_ = ControlMode::JointVelocity;
          arm.second.switch_cm_ = true;
        }
        break;

      case 2: // Cartesian is not supported for simulation
        RCLCPP_ERROR(this->getLogger(), "Cartesian interfaces are not supported for simulation!\n" \
                                        "Please make sure they are Joint.");
        return hardware_interface::return_type::ERROR;

      default:
        break;
    }
  }
  return hardware_interface::return_type::OK;

}

hardware_interface::return_type FrankaMjHardwareSystem::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/) {

  RCLCPP_INFO(this->getLogger(),"Performing command mode switch");

  for(auto &arm_p : arms_){
    auto& arm = arm_p.second;
    RCLCPP_INFO_STREAM(this->getLogger(), "Arm " << arm.robot_name_ << 
                                          " current mode: " << arm.control_mode_);
    if(arm.switch_cm_){
      if(arm.control_mode_ == ControlMode::None){
        for(size_t i=0; i<kNumberOfJoints; i++){
          set_torque_control(m_, arm.robot_->act_trq_indices_[i], 0);
          set_position_servo(m_, arm.robot_->act_pos_indices_[i], 0);
          set_velocity_servo(m_, arm.robot_->act_vel_indices_[i], 0);
        }
      }
      else if(arm.control_mode_ == ControlMode::JointTorque){
        for(size_t i=0; i<kNumberOfJoints; i++){
          set_torque_control(m_, arm.robot_->act_trq_indices_[i], 1);
          set_position_servo(m_, arm.robot_->act_pos_indices_[i], 0);
          set_velocity_servo(m_, arm.robot_->act_vel_indices_[i], 0);
        }
      }
      else if(arm.control_mode_ == ControlMode::JointPosition){
        for(size_t i=0; i<kNumberOfJoints; i++){
          set_torque_control(m_, arm.robot_->act_trq_indices_[i], 0);
          set_position_servo(m_, arm.robot_->act_pos_indices_[i], 100);
          set_velocity_servo(m_, arm.robot_->act_vel_indices_[i], 0);
        }
      }
      else if(arm.control_mode_ == ControlMode::JointVelocity){
        for(size_t i=0; i<kNumberOfJoints; i++){
          set_torque_control(m_, arm.robot_->act_trq_indices_[i], 0);
          set_position_servo(m_, arm.robot_->act_pos_indices_[i], 0);
          set_velocity_servo(m_, arm.robot_->act_vel_indices_[i], 10);
        }
      }
      arm.switch_cm_ = false;
    }
  }


  return hardware_interface::return_type::OK;
}

rclcpp::Logger FrankaMjHardwareSystem::getLogger() {
  return rclcpp::get_logger("FrankaMjHardwareSystem");
}

}  // namespace franka_hardware

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_hardware::FrankaMjHardwareSystem,
                       mujoco_ros2_control::MujocoRos2SystemInterface)