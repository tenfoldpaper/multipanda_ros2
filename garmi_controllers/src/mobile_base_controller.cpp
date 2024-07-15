#include "garmi_controllers/mobile_base_controller.hpp"

using std::placeholders::_1;

namespace garmi_controllers{


controller_interface::InterfaceConfiguration 
  MobileBaseController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(robot_id_ + "_joint_left/velocity");
  config.names.push_back(robot_id_ + "_joint_right/velocity");
  return config;
};

controller_interface::InterfaceConfiguration 
  MobileBaseController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(robot_id_ + "_joint_left/velocity");
  config.names.push_back(robot_id_ + "_joint_left/position");
  config.names.push_back(robot_id_ + "_joint_right/velocity");
  config.names.push_back(robot_id_ + "_joint_right/position");
  return config;
};

controller_interface::return_type MobileBaseController::update(const rclcpp::Time& time,
                                                               const rclcpp::Duration& period){
  last_time_ = current_time_;
  current_time_ = this->get_node()->now();
  // RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "\nLeft vel:  %f\nRight vel: %f", state_interfaces_.at(0).get_value(), state_interfaces_.at(2).get_value());
  
  base_state_.set_theta(0, state_interfaces_.at(1).get_value()); // left 
  base_state_.set_theta(1, state_interfaces_.at(3).get_value()); // right

  command_interfaces_[0].set_value(base_state_.get_velocity_target(0));
  command_interfaces_[1].set_value(base_state_.get_velocity_target(1));
  return controller_interface::return_type::OK;
} ;
CallbackReturn MobileBaseController::on_init(){
  // nothing for now
  try {
    auto_declare<std::string>("robot_id", "garmi_base");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
} ;

CallbackReturn MobileBaseController::on_configure(const rclcpp_lifecycle::State& previous_state){
  // nothing for now
  robot_id_ = get_node()->get_parameter("robot_id").as_string();
  is_sim_ = get_node()->get_parameter("sim").as_bool();
  goal_twist_sub_ = this->get_node()->create_subscription<geometry_msgs::msg::Twist>(robot_id_ + "/twist_command", 1, std::bind(&MobileBaseController::command_listener, this, _1));
  return CallbackReturn::SUCCESS;
} ;

CallbackReturn MobileBaseController::on_activate(const rclcpp_lifecycle::State& previous_state){
  current_time_ = this->get_node()->now();
  last_time_ = this->get_node()->now();
  return CallbackReturn::SUCCESS;
} ;

CallbackReturn MobileBaseController::on_deactivate(const rclcpp_lifecycle::State& previous_state){
  return CallbackReturn::SUCCESS;
} ;
}   // namespace
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(garmi_controllers::MobileBaseController,
                       controller_interface::ControllerInterface)