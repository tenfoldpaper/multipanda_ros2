#include "franka_robot_state_broadcaster/franka_robot_state_broadcaster.hpp"

#include <stddef.h>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcpputils/split.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/header.hpp"

namespace franka_robot_state_broadcaster {

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<int>("frequency", 30);

  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id = get_node()->get_parameter("arm_id").as_string();
  frequency = get_node()->get_parameter("frequency").as_int();
  last_pub_ = get_node()->now();
  franka_robot_state = std::make_unique<franka_semantic_components::FrankaRobotState>(
      franka_semantic_components::FrankaRobotState(arm_id + "/" + state_interface_name, arm_id));

  try {
    franka_state_publisher = get_node()->create_publisher<franka_msgs::msg::FrankaState>(
        "~/" + state_interface_name, rclcpp::SystemDefaultsQoS());
    realtime_franka_state_publisher =
        std::make_shared<realtime_tools::RealtimePublisher<franka_msgs::msg::FrankaState>>(
            franka_state_publisher);
    ;
  } catch (const std::exception& e) {
    fprintf(stderr,
            "Exception thrown during publisher creation at configure stage with message : %s \n",
            e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_node()->get_logger(), "%s franka state broadcaster configuration successful", arm_id.c_str());
  return CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_state->assign_loaned_state_interfaces(state_interfaces_);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_state->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
FrankaRobotStateBroadcaster::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
FrankaRobotStateBroadcaster::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = franka_robot_state->get_state_interface_names();
  return state_interfaces_config;
}

controller_interface::return_type FrankaRobotStateBroadcaster::update(
    const rclcpp::Time& time,
    const rclcpp::Duration& /*period*/) {
  if(time.nanoseconds() - last_pub_.nanoseconds() < 1'000'000'000 / frequency){
    return controller_interface::return_type::OK;
  }
  if (realtime_franka_state_publisher && realtime_franka_state_publisher->trylock()) {
    realtime_franka_state_publisher->msg_.header.stamp = time;

    if (!franka_robot_state->get_values_as_message(realtime_franka_state_publisher->msg_)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Failed to get franka state via franka state interface.");
      realtime_franka_state_publisher->unlock();
      return controller_interface::return_type::ERROR;
    }
    realtime_franka_state_publisher->unlockAndPublish();
    last_pub_ = get_node()->now();
    return controller_interface::return_type::OK;

  } else {
    return controller_interface::return_type::ERROR;
  }
}

} // namespace franka_state_broadcaster

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_robot_state_broadcaster::FrankaRobotStateBroadcaster,
                       controller_interface::ControllerInterface)