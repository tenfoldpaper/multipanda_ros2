#include <franka_hardware/franka_error_recovery_service_server.hpp>

namespace franka_hardware{

FrankaErrorRecoveryServiceServer::FrankaErrorRecoveryServiceServer(const rclcpp::NodeOptions & options,
                                                                    std::shared_ptr<Robot> robot)
    : rclcpp::Node("service_server", options), robot_(std::move(robot))
{
    error_recovery_service_ = create_service<franka_msgs::srv::ErrorRecovery>(
        "~/error_recovery",
        std::bind(
            &FrankaErrorRecoveryServiceServer::triggerAutomaticRecovery, this, 
            std::placeholders::_1,
            std::placeholders::_2
        )
    );

    RCLCPP_INFO(get_logger(), "Error recovery service started");
}

void FrankaErrorRecoveryServiceServer::triggerAutomaticRecovery(const franka_msgs::srv::ErrorRecovery::Request::SharedPtr& request,
                                                                const franka_msgs::srv::ErrorRecovery::Response::SharedPtr& response)
{
    if(!this->robot_->hasError()){
        RCLCPP_INFO(this->get_logger(), "No errors detected; error recovery is not necessary.");
        response->error = "No errors";
        response->success = false;
    }
    else{
        try{
            this->robot_->doAutomaticErrorRecovery();
            this->robot_->setError(false);
            RCLCPP_INFO(this->get_logger(), "Successfully recovered from error.");
            response->success = true;
        }
        catch(franka::ControlException& e){
            RCLCPP_ERROR(this->get_logger(), "Error recovery failed: %s", e.what());
            response->error = e.what();
            response->success = false;
        }
        catch(franka::NetworkException& e){
            RCLCPP_ERROR(this->get_logger(), "Error recovery timed out. :%s", e.what());
            response->error = e.what();
            response->success = false;
        }
    }
};


} // namespace franka_hardware