#include "franka_robot_state_broadcaster/franka_robot_model_broadcaster.hpp"

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

namespace {
template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}  // anonymous namespace

namespace franka_robot_state_broadcaster {

controller_interface::CallbackReturn FrankaRobotModelBroadcaster::on_init() {
  try {
    auto_declare<std::string>("arm_id", "panda");
    auto_declare<int>("frequency", 30);

  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

// configure works
controller_interface::CallbackReturn FrankaRobotModelBroadcaster::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id = get_node()->get_parameter("arm_id").as_string();
  frequency = get_node()->get_parameter("frequency").as_int();
  last_pub_ = get_node()->now();
  franka_robot_model = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(arm_id + "/" + model_interface_name, 
                                                   arm_id));

  try {
    franka_model_publisher = get_node()->create_publisher<franka_msgs::msg::FrankaModel>(
        "~/" + model_interface_name, rclcpp::SystemDefaultsQoS());
    realtime_franka_model_publisher =
        std::make_shared<realtime_tools::RealtimePublisher<franka_msgs::msg::FrankaModel>>(
            franka_model_publisher);
    ;
  } catch (const std::exception& e) {
    fprintf(stderr,
            "Exception thrown during publisher creation at configure stage with message : %s \n",
            e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_node()->get_logger(), "%s franka model broadcaster configuration successful", arm_id.c_str());
  return CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn FrankaRobotModelBroadcaster::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model->assign_loaned_state_interfaces(state_interfaces_);
  for(auto& state : state_interfaces_){
    RCLCPP_INFO(get_node()->get_logger(),"on_activate: %s", state.get_name().c_str());
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaRobotModelBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_model->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
FrankaRobotModelBroadcaster::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
FrankaRobotModelBroadcaster::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for(const auto& name : franka_robot_model->get_state_interface_names()){
    state_interfaces_config.names.push_back(name);
  }
  return state_interfaces_config;
}

controller_interface::return_type FrankaRobotModelBroadcaster::update(
    const rclcpp::Time& time,
    const rclcpp::Duration& /*period*/) {
  
  if(time.nanoseconds() - last_pub_.nanoseconds() < 1'000'000'000 / frequency){
    return controller_interface::return_type::OK;
  }
  if (realtime_franka_model_publisher && realtime_franka_model_publisher->trylock()) {
    realtime_franka_model_publisher->msg_.header.stamp = time;

    if (!franka_robot_model->get_values_as_message(realtime_franka_model_publisher->msg_)) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Failed to get franka model via franka model interface.");
      realtime_franka_model_publisher->unlock();
      return controller_interface::return_type::ERROR;
    }
    realtime_franka_model_publisher->unlockAndPublish();
    last_pub_ = get_node()->now();
    return controller_interface::return_type::OK;
  } 
  
  else {
    return controller_interface::return_type::ERROR;
  }

  }
} // namespace franka_model_broadcaster

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_robot_state_broadcaster::FrankaRobotModelBroadcaster,
                       controller_interface::ControllerInterface)