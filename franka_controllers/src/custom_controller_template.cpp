/**
 * TODO: include the header file
 */
#include <franka_controllers/custom_controller_template.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace franka_controllers {

controller_interface::InterfaceConfiguration
CustomController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
/**
 * TODO: Determine the controller command interface. It decides your control
 *       variable, namely the output of the controller. More interface name can be 
 *       found in implementation of example controller under franka_example_controllers pkg.
 *       Example:
 *       for (int i = 1; i <= num_joints; ++i) {
 *        config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
 *       }
 * HINT: The order of the interface name is fixed here! You need to index
 *       the interface with the order defined here! This is the same for state interface below.
 

 */
  return config;
}

controller_interface::InterfaceConfiguration
CustomController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
/**
 * TODO: Determine the state interface. It decides the state variables you can
 *       read from the robot, like joint position q_.
 *       Example:
 *       for (int i = 1; i <= num_joints; ++i) {
 *         config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
 *         config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
 *       }
 * HINT: If you define the interface for joint position and joint velocity in the above way,
 *       the interface for the first joint position can be obtained by
 *       const auto& position_interface_1 = state_interfaces_.at(0);
 *       while for the second joint position can be obtained by
 *       const auto& position_interface_1 = state_interfaces_.at(2);
 *       Because the interfaces are pushed back in an order like position-velocity-position-.
 *       Detailed way to read the values can be found in function updateJointStates.
 */
  return config;
}

controller_interface::return_type
CustomController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  updateJointStates();
/**
 * TODO: Update the control variable
 *       Example:
 *       <some custom process>
 *       for (int i = 0; i < num_joints; ++i) {
 *         command_interfaces_[i].set_value(<tau>);
 *       }
 */
  return controller_interface::return_type::OK;
}

CallbackReturn
CustomController::on_init() {
  try {
    /**
    * TODO: Initialization of loaded parameters and your custom things
    *       - Loaded parameter example:
    *       auto_declare<std::string>("arm_id", "panda");
    *       - Subscription example:
    *       sub_desired_joint_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    *        "/joint_impedance/joints_desired", 1,
    *        std::bind(&CustomController::ExampleCallback, this, std::placeholders::_1)
    *       );
    *       }
    * HINT: You can use get_node()->xxx to create subscription or publisher.
    *       The normal way to firstly create a node and then create subscription or  
    *       publisher with this node should also work.
    */
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn
CustomController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  /**
    * TODO: Load parameters
    *       - Loaded parameter example:
    *       arm_id_ = get_node()->get_parameter("arm_id").as_string();
    */ 
  return CallbackReturn::SUCCESS;
}

CallbackReturn
CustomController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  /**
    * TODO: Do some initialization update to activate the controller
    *       if needed.
    */ 
  return CallbackReturn::SUCCESS;
}

void CustomController::updateJointStates() {
  /**
    * TODO: Update states
    * Example:
    * for (auto i = 0; i < num_joints; ++i) {
    *   const auto& position_interface = state_interfaces_.at(2 * i);
    *   const auto& velocity_interface = state_interfaces_.at(2 * i + 1);
    * 
    *   assert(position_interface.get_interface_name() == "position");
    *   assert(velocity_interface.get_interface_name() == "velocity");
    *
    *   q_(i) = position_interface.get_value();
    *   dq_(i) = velocity_interface.get_value();
        }
    */
  
  
}

// void CustomController::ExampleCallback(
//   const std_msgs::msg::Float64MultiArray& msg) {
//   /**
//     * TODO: Custom callback if you need
//     */ 
// }

}  // namespace franka_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
/**
  * TODO: Export the controller (use your controller name here)
  */ 
PLUGINLIB_EXPORT_CLASS(franka_controllers::CustomController,
                       controller_interface::ControllerInterface)
