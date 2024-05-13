#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "franka_msgs/msg/franka_state.hpp"
#include "franka_msgs/msg/errors.hpp"
#include "franka_semantic_components/franka_robot_state.hpp"

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_publisher.h"



namespace franka_robot_state_broadcaster{

class FrankaRobotStateBroadcaster : public controller_interface::ControllerInterface {
    public:

    // Lifecycle state transition functions
    controller_interface::CallbackReturn on_init() override;

    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    // Update loop
    controller_interface::return_type update(const rclcpp::Time& time,
                                            const rclcpp::Duration& period) override;

    // Configuration
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    
    protected:
    
    std::string arm_id;
    std::string state_interface_name{"robot_state"};
    std::shared_ptr<rclcpp::Publisher<franka_msgs::msg::FrankaState>> franka_state_publisher;
    std::shared_ptr<realtime_tools::RealtimePublisher<franka_msgs::msg::FrankaState>>
        realtime_franka_state_publisher;   
    std::unique_ptr<franka_semantic_components::FrankaRobotState> franka_robot_state;
    size_t arm_count;
    rclcpp::Time last_pub_;
    double frequency;
};

} // namespace franka_robot_state_broadcaster