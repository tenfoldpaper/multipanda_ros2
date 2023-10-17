#include <franka_hardware/franka_multi_hardware_interface.hpp>

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

CallbackReturn FrankaMultiHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  // First get number of robots
  std::stringstream rc_stream(info_.hardware_parameters.at("robot_count"));
  rc_stream >> robot_count_;
  if(robot_count_ < 2U){
    RCLCPP_FATAL(getLogger(), "Configured robot count is less than 2. Please use FrankaHardwareInterface instead.");
    return CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kNumberOfJoints * robot_count_) {
    RCLCPP_FATAL(getLogger(), "Got %ld joints. Expected %ld.", info_.joints.size(),
                 kNumberOfJoints * robot_count_);
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

  for(size_t i = 1; i <= robot_count_; i++){
    // Setup arm container
    std::string suffix = "_" + std::to_string(i);
    std::string robot_name;

    try{
      robot_name = info_.hardware_parameters.at("ns"+suffix);
    }
    catch(const std::out_of_range& ex){
      RCLCPP_FATAL(getLogger(), "Parameber 'ns%s' ! set\nMake sure all multi-robot parameters follow the format {ns/robot_ip/etc.}_n", suffix.c_str());
      return CallbackReturn::ERROR;
    }
    if(!arms_.insert(std::make_pair(robot_name, ArmContainer())).second){
      RCLCPP_FATAL(getLogger(),"The provided robot namespace %s already exists! Make sure they are unique.", robot_name.c_str());
      return CallbackReturn::ERROR;
    }
    auto &arm = arms_[robot_name];
    state_pointers_.insert(std::make_pair(robot_name, &arm.hw_franka_robot_state_));
    arm.robot_name_ = robot_name;
  
    try {
      arm.robot_ip_ = info_.hardware_parameters.at("robot_ip"+suffix);
    } catch (const std::out_of_range& ex) {
      RCLCPP_FATAL(getLogger(), "Parameter 'robot_ip%s' ! set", suffix.c_str());
      return CallbackReturn::ERROR;
    }
    try {
      RCLCPP_INFO(getLogger(), "Connecting to robot at \"%s\" ...", arm.robot_ip_.c_str());
      arm.robot_ = std::make_unique<Robot>(arm.robot_ip_, getLogger());
    } catch (const franka::Exception& e) {
      RCLCPP_FATAL(getLogger(), "Could ! connect to robot");
      RCLCPP_FATAL(getLogger(), "%s", e.what());
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(getLogger(), "Successfully connected to robot %s at %s", robot_name.c_str(), arm.robot_ip_.c_str());
    arm.hw_franka_robot_state_ = arm.robot_->read();
    // Start the service nodes
    arm.error_recovery_service_node_ = std::make_shared<FrankaErrorRecoveryServiceServer>(rclcpp::NodeOptions(), arm.robot_, robot_name + "_");
    arm.param_service_node_ = std::make_shared<FrankaParamServiceServer>(rclcpp::NodeOptions(), arm.robot_, robot_name + "_");
    executor_ = std::make_shared<FrankaExecutor>();
    executor_->add_node(arm.error_recovery_service_node_);
    executor_->add_node(arm.param_service_node_);
    
    // Init the cartesian values to 0
    arm.hw_cartesian_positions_.fill({});
    arm.hw_cartesian_velocities_.fill({});
    arm.hw_commands_cartesian_position_.fill({});
    arm.hw_commands_cartesian_velocity_.fill({});
  }
  RCLCPP_INFO(getLogger(), "All %ld robots have been intiialized (%ld)", robot_count_, arms_.size() );
  if(arms_.size() != robot_count_){
    RCLCPP_FATAL(getLogger(), "initialized arm container size %ld is not the same as robot_count %ld", arms_.size(), robot_count_);
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

// Function for extracting ns in joint names
std::string get_ns(std::string const& s)
{
    std::string::size_type pos = s.find('_');
    if (pos != std::string::npos)
    {
        return s.substr(0, pos);
    }
    else
    {
        return s;
    }
}

// Function for extracting joint number
int get_joint_no(std::string const& s){
  int no = s.back() - '0' - 1;
  return no;
}

std::vector<StateInterface> FrankaMultiHardwareInterface::export_state_interfaces() {
  std::vector<StateInterface> state_interfaces;
  for (auto i = 0U; i < info_.joints.size(); i++) {
    
    std::cout << get_ns(info_.joints[i].name) << std::endl;
    state_interfaces.emplace_back(StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &arms_[get_ns(info_.joints[i].name)].hw_positions_.at(get_joint_no(info_.joints[i].name))));
    state_interfaces.emplace_back(StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &arms_[get_ns(info_.joints[i].name)].hw_velocities_.at(get_joint_no(info_.joints[i].name))));
    state_interfaces.emplace_back(StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &arms_[get_ns(info_.joints[i].name)].hw_efforts_.at(get_joint_no(info_.joints[i].name))));
  }

  for(auto arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;

    std::string cartesian_position_prefix = arm.robot_name_ + "_ee_cartesian_position";
    std::string cartesian_velocity_prefix = arm.robot_name_ + "_ee_cartesian_velocity";

    for (auto i = 0; i < 16; i++){
      state_interfaces.emplace_back(StateInterface(
          cartesian_position_prefix, cartesian_matrix_names[i], &arm.hw_cartesian_positions_[i]));
      state_interfaces.emplace_back(StateInterface(
          cartesian_velocity_prefix, cartesian_matrix_names[i], &arm.hw_cartesian_velocities_[i]));
    }

    state_interfaces.emplace_back(StateInterface(
        arm.robot_name_, k_robot_state_interface_name,
        reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
            &state_pointers_[arm_container_pair.first])));
    state_interfaces.emplace_back(StateInterface(
        arm.robot_name_, k_robot_model_interface_name,
        reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
            &arm.hw_franka_model_ptr_)));
  }
  
  return state_interfaces;
}

std::vector<CommandInterface> FrankaMultiHardwareInterface::export_command_interfaces() {
  std::vector<CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());
  RCLCPP_INFO(getLogger(), "%ld", info_.joints.size());

  for (auto i = 0U; i < info_.joints.size(); i++) {
    RCLCPP_INFO(getLogger(), "%s", info_.joints[i].name.c_str());

    command_interfaces.emplace_back(CommandInterface( // JOINT EFFORT
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &arms_[get_ns(info_.joints[i].name)].hw_commands_joint_effort_.at(get_joint_no(info_.joints[i].name))));
    command_interfaces.emplace_back(CommandInterface( // JOINT POSITION
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &arms_[get_ns(info_.joints[i].name)].hw_commands_joint_position_.at(get_joint_no(info_.joints[i].name))));
    command_interfaces.emplace_back(CommandInterface( // JOINT VELOCITY
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &arms_[get_ns(info_.joints[i].name)].hw_commands_joint_velocity_.at(get_joint_no(info_.joints[i].name))));
  }

  for(auto arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    std::string cartesian_position_prefix = arm.robot_name_ + "_ee_cartesian_position";
    std::string cartesian_velocity_prefix = arm.robot_name_ + "_ee_cartesian_velocity";

    for (auto i = 0; i < 16; i++){
      command_interfaces.emplace_back(CommandInterface(
          cartesian_position_prefix, cartesian_matrix_names[i], &arm.hw_commands_cartesian_position_[i]));
    }
    for (auto i = 0; i < 6; i++){
      command_interfaces.emplace_back(CommandInterface(
          cartesian_velocity_prefix, cartesian_velocity_command_names[i], &arm.hw_commands_cartesian_velocity_[i]));
    }
  }

  return command_interfaces;
}

CallbackReturn FrankaMultiHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  for(auto& arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    RCLCPP_INFO(getLogger(), "key: %s", arm_container_pair.first.c_str());

    arm.robot_->initializeContinuousReading();
    arm.hw_commands_joint_effort_.fill(0);
  }
  read(rclcpp::Time(0),
       rclcpp::Duration(0, 0));  // makes sure that the robot state is properly initialized.
  RCLCPP_INFO(getLogger(), "Started");
  return CallbackReturn::SUCCESS;
}

CallbackReturn FrankaMultiHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(getLogger(), "trying to Stop...");
  for(auto arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    arm.robot_->stopRobot();
  }
  RCLCPP_INFO(getLogger(), "Stopped");
  return CallbackReturn::SUCCESS;
}


hardware_interface::return_type FrankaMultiHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/) {
  
  for(auto& arm_container_pair: arms_){
    auto &arm = arm_container_pair.second;
    if (arm.hw_franka_model_ptr_ == nullptr) {
      arm.hw_franka_model_ptr_ = arm.robot_->getModel();
    }
    arm.hw_franka_robot_state_ = arm.robot_->read();
    arm.hw_positions_ = arm.hw_franka_robot_state_.q;
    arm.hw_velocities_ = arm.hw_franka_robot_state_.dq;
    arm.hw_efforts_ = arm.hw_franka_robot_state_.tau_J;
    arm.hw_cartesian_positions_ = arm.hw_franka_robot_state_.O_T_EE;
    arm.hw_cartesian_velocities_ = arm.hw_franka_robot_state_.O_T_EE_d;
  }
  
  return hardware_interface::return_type::OK;                                                              
  }

hardware_interface::return_type FrankaMultiHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                               const rclcpp::Duration& /*period*/) {
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
    if (std::any_of(arm.hw_commands_cartesian_position_.begin(), arm.hw_commands_cartesian_position_.end(),
                    [](double c) { return !std::isfinite(c); })) {
      return hardware_interface::return_type::ERROR;
    }
    if (std::any_of(arm.hw_commands_cartesian_velocity_.begin(), arm.hw_commands_cartesian_velocity_.end(),
                    [](double c) { return !std::isfinite(c); })) {
      return hardware_interface::return_type::ERROR;
    }

    arm.robot_->write(arm.hw_commands_joint_effort_, 
                      arm.hw_commands_joint_position_, 
                      arm.hw_commands_joint_velocity_, 
                      arm.hw_commands_cartesian_position_, 
                      arm.hw_commands_cartesian_velocity_);

    if(arm.robot_->hasError()){
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaMultiHardwareInterface::prepare_command_mode_switch(
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
      if(stop_interfaces.size() != kNumberOfJoints * robot_count_){
        RCLCPP_ERROR(this->getLogger(), "Requested joint stop interface's size is not %ld (got %ld)", kNumberOfJoints*robot_count_, stop_interfaces.size());
        return hardware_interface::return_type::ERROR;
      }
      for(auto& arm_container_pair : arms_){
        auto& arm = arm_container_pair.second;
        for(size_t i = 0 ; i < kNumberOfJoints; i++){
          if(is_effort){
            arm.hw_commands_joint_effort_[i] = 0;
          }
          // position command is not reset, since that can be dangerous
          else if(is_velocity){
            arm.hw_commands_joint_velocity_[i] = 0;
          }
        }
      }
      control_mode_ = ControlMode::None;
      break;

    case 2: // stop the cartesian controllers
      if(is_position){
        if(stop_interfaces.size() != 16U * robot_count_){
          RCLCPP_ERROR(this->getLogger(), "Requested Cartesian position stop interface's size is not %ld (got %ld)", 16 * robot_count_, stop_interfaces.size());
          return hardware_interface::return_type::ERROR;
        }
      }
      if(is_velocity){
        if(stop_interfaces.size() != 6U * robot_count_){
          RCLCPP_ERROR(this->getLogger(), "Requested Cartesian velocity stop interface's size is not %ld (got %ld)", 6 * robot_count_, stop_interfaces.size());
          return hardware_interface::return_type::ERROR;

        }
        // set the commands to zero
        for(auto& arm_container_pair : arms_){
          auto& arm = arm_container_pair.second;
          for(int i = 0; i < 6; i++){
            arm.hw_commands_cartesian_velocity_[i] = 0;
          }
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
      if(start_interfaces.size() != kNumberOfJoints * robot_count_){
        RCLCPP_ERROR(this->getLogger(), "Requested joint start interface's size is not %ld (got %ld)", kNumberOfJoints * robot_count_, start_interfaces.size());
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

      if(start_interfaces.size() != 16U * robot_count_ && start_interfaces.size() != 6U * robot_count_){
        RCLCPP_ERROR(this->getLogger(), "Requested Cartesian start interface's size is not %ld nor %ld (got %ld)", 16U * robot_count_, 6U * robot_count_, start_interfaces.size());
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

hardware_interface::return_type FrankaMultiHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/) {

  RCLCPP_INFO(this->getLogger(),"Performing command mode switch");
  for(auto& arm_container_pair : arms_){
    auto& arm = arm_container_pair.second;
    if(control_mode_ == ControlMode::None){
      arm.robot_->stopRobot();
      arm.robot_->initializeContinuousReading();
    }
    else if(control_mode_ == ControlMode::JointTorque){
      arm.robot_->stopRobot();
      arm.robot_->initializeTorqueControl();
    }
    else if(control_mode_ == ControlMode::JointPosition){
      arm.robot_->stopRobot();
      arm.robot_->initializeJointPositionControl();
    }
    else if(control_mode_ == ControlMode::JointVelocity){
      arm.robot_->stopRobot();
      arm.robot_->initializeJointVelocityControl();
    }
    else if(control_mode_ == ControlMode::CartesianPose){
      arm.robot_->stopRobot();
      arm.robot_->initializeCartesianPositionControl();
    }
    else if(control_mode_ == ControlMode::CartesianVelocity){
      arm.robot_->stopRobot();
      arm.robot_->initializeCartesianVelocityControl();
    }
  }

  return hardware_interface::return_type::OK;
}


rclcpp::Logger FrankaMultiHardwareInterface::getLogger() {
  return rclcpp::get_logger("FrankaMultiHardwareInterface");
}
}


#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_hardware::FrankaMultiHardwareInterface,
                       hardware_interface::SystemInterface)