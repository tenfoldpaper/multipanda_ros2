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
  int robot_count;
  robot_count = std::stoi(info_.hardware_parameters.at("robot_count"));
  if(robot_count < 2){
    RCLCPP_FATAL(getLogger(), "Configured robot count is less than 2. Please use FrankaHardwareInterface instead.");
    return CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kNumberOfJoints * robot_count) {
    RCLCPP_FATAL(getLogger(), "Got %ld joints. Expected %ld.", info_.joints.size(),
                 kNumberOfJoints * robot_count);
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

  for(int i = 1; i <= robot_count; i++){
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
    if(!arm_container_.insert(std::make_pair(robot_name, ArmContainer())).second){
      RCLCPP_FATAL(getLogger(),"The provided robot namespace %s already exists! Make sure they are unique.", robot_name.c_str());
      return CallbackReturn::ERROR;
    }
    arm_container_[robot_name].robot_name = robot_name;
  
    try {
      arm_container_[robot_name].robot_ip = info_.hardware_parameters.at("robot_ip"+suffix);
    } catch (const std::out_of_range& ex) {
      RCLCPP_FATAL(getLogger(), "Parameter 'robot_ip%s' ! set", suffix.c_str());
      return CallbackReturn::ERROR;
    }
    try {
      RCLCPP_INFO(getLogger(), "Connecting to robot at \"%s\" ...", arm_container_[robot_name].robot_ip.c_str());
      arm_container_[robot_name].robot = std::make_unique<Robot>(arm_container_[robot_name].robot_ip, getLogger());
    } catch (const franka::Exception& e) {
      RCLCPP_FATAL(getLogger(), "Could ! connect to robot");
      RCLCPP_FATAL(getLogger(), "%s", e.what());
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(getLogger(), "Successfully connected to robot %s at %s", robot_name.c_str(), arm_container_[robot_name].robot_ip.c_str());

    // Start the service nodes
    arm_container_[robot_name].error_recovery_service_node = std::make_shared<FrankaErrorRecoveryServiceServer>(rclcpp::NodeOptions(), arm_container_[robot_name].robot, robot_name);
    arm_container_[robot_name].param_service_node = std::make_shared<FrankaParamServiceServer>(rclcpp::NodeOptions(), arm_container_[robot_name].robot, robot_name);
    executor_ = std::make_shared<FrankaExecutor>();
    executor_->add_node(arm_container_[robot_name].error_recovery_service_node);
    executor_->add_node(arm_container_[robot_name].param_service_node);
    

    // Init the cartesian values to 0
    arm_container_[robot_name].hw_cartesian_positions_.fill({});
    arm_container_[robot_name].hw_cartesian_velocities_.fill({});
    arm_container_[robot_name].hw_commands_cartesian_position.fill({});
    arm_container_[robot_name].hw_commands_cartesian_velocity.fill({});
  }
  RCLCPP_INFO(getLogger(), "All %d robots have been intiialized (%ld)", robot_count, arm_container_.size() );
  if(arm_container_.size() != robot_count){
    RCLCPP_FATAL(getLogger(), "initialized arm container size %ld is not the same as robot_count %d", arm_container_.size(), robot_count);
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
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &arm_container_[get_ns(info_.joints[i].name)].hw_positions_.at(get_joint_no(info_.joints[i].name))));
    state_interfaces.emplace_back(StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &arm_container_[get_ns(info_.joints[i].name)].hw_velocities_.at(get_joint_no(info_.joints[i].name))));
    state_interfaces.emplace_back(StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &arm_container_[get_ns(info_.joints[i].name)].hw_efforts_.at(get_joint_no(info_.joints[i].name))));
  }

  for(auto arm_container_pair : arm_container_){
    std::string cartesian_position_prefix = arm_container_pair.second.robot_name + "_ee_cartesian_position";
    std::string cartesian_velocity_prefix = arm_container_pair.second.robot_name + "_ee_cartesian_velocity";

    for (auto i = 0; i < 16; i++){
      state_interfaces.emplace_back(StateInterface(
          cartesian_position_prefix, cartesian_matrix_names[i], &arm_container_pair.second.hw_cartesian_positions_[i]));
      state_interfaces.emplace_back(StateInterface(
          cartesian_velocity_prefix, cartesian_matrix_names[i], &arm_container_pair.second.hw_cartesian_velocities_[i]));
    }

    state_interfaces.emplace_back(StateInterface(
        arm_container_pair.second.robot_name, k_robot_state_interface_name,
        reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
            &arm_container_pair.second.hw_franka_robot_state_addr_)));
    state_interfaces.emplace_back(StateInterface(
        arm_container_pair.second.robot_name, k_robot_model_interface_name,
        reinterpret_cast<double*>(  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
            &arm_container_pair.second.hw_franka_model_ptr_)));
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
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &arm_container_[get_ns(info_.joints[i].name)].hw_commands_joint_effort.at(get_joint_no(info_.joints[i].name))));
    command_interfaces.emplace_back(CommandInterface( // JOINT POSITION
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &arm_container_[get_ns(info_.joints[i].name)].hw_commands_joint_position.at(get_joint_no(info_.joints[i].name))));
    command_interfaces.emplace_back(CommandInterface( // JOINT VELOCITY
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &arm_container_[get_ns(info_.joints[i].name)].hw_commands_joint_velocity.at(get_joint_no(info_.joints[i].name))));
  }

  for(auto arm_container_pair : arm_container_){
    std::string cartesian_position_prefix = arm_container_pair.second.robot_name + "_ee_cartesian_position";
    std::string cartesian_velocity_prefix = arm_container_pair.second.robot_name + "_ee_cartesian_velocity";

    for (auto i = 0; i < 16; i++){
      command_interfaces.emplace_back(CommandInterface(
          cartesian_position_prefix, cartesian_matrix_names[i], &arm_container_pair.second.hw_commands_cartesian_position[i]));
    }
    for (auto i = 0; i < 6; i++){
      command_interfaces.emplace_back(CommandInterface(
          cartesian_velocity_prefix, cartesian_velocity_command_names[i], &arm_container_pair.second.hw_commands_cartesian_velocity[i]));
    }
  }

  return command_interfaces;
}

CallbackReturn FrankaMultiHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  for(auto& arm_container_pair : arm_container_){
    RCLCPP_INFO(getLogger(), "key: %s", arm_container_pair.first.c_str());

    arm_container_pair.second.robot->initializeContinuousReading();
    arm_container_pair.second.hw_commands_joint_effort.fill(0);
  }
  read(rclcpp::Time(0),
       rclcpp::Duration(0, 0));  // makes sure that the robot state is properly initialized.
  RCLCPP_INFO(getLogger(), "Started");
  return CallbackReturn::SUCCESS;
}

CallbackReturn FrankaMultiHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(getLogger(), "trying to Stop...");
  for(auto arm_container_pair : arm_container_){

    arm_container_pair.second.robot->stopRobot();
  }
  RCLCPP_INFO(getLogger(), "Stopped");
  return CallbackReturn::SUCCESS;
}


hardware_interface::return_type FrankaMultiHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/) {
  
  for(auto& arm_container_pair: arm_container_){
    auto &arm = arm_container_.at(arm_container_pair.first);
    if (arm.hw_franka_model_ptr_ == nullptr) {
      arm.hw_franka_model_ptr_ = arm.robot->getModel();
    }
    arm.hw_franka_robot_state_ = arm.robot->read();
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