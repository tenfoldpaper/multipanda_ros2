#pragma once

#include <multi_mode_control_msgs/srv/get_robot_states.hpp>
#include <multi_mode_controller/base/switch_engine_interface.h>
#include <multi_mode_controller/utils/get_robot_state_msg.h>
#include <franka/robot_state.h>
// franka_msgs::FrankaState getRobotStateMsg(const franka::RobotState& rs);


namespace switch_engine {

  using multi_mode_control_msgs::srv::GetRobotStates;
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

class SwitchEngine : public SwitchEngineInterface {
 private:
  
//   ros::ServiceServer get_robot_states_;
  virtual void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
      panda_controllers::PandaControllerInterface* control) override {
    return control->computeTau(tau);
  }
  // Service not working FIX:
  // if arg is shared pointers, they have to be both const
  // or you get an operator = error 
  // and making them just references don't work either.
  bool getRobotStatesCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const GetRobotStates::Request::SharedPtr& /*req*/,
        const GetRobotStates::Response::SharedPtr& res) {
    for (const auto& rd : robot_data_) {
      res->states.push_back(getRobotStateMsg(rd->state()));
    }
    return true;
  }
  virtual bool initImpl(
      const std::vector<panda_controllers::RobotData*>& robot_data,
      rclcpp_lifecycle::LifecycleNode::SharedPtr& node, std::string name, std::string resource) override {
    robot_data_ = robot_data;
    std::string topic_prefix = resource;
    std::size_t pos = topic_prefix.find("&");
    while (pos != std::string::npos) {
      topic_prefix.replace(pos, 1, "_and_");
      pos = topic_prefix.find("&");
    }
    
    get_robot_states_server_ = node->create_service<GetRobotStates>(
        topic_prefix + "/get_robot_states",
        std::bind(&SwitchEngine::getRobotStatesCallback, this, _1, _2, _3));

    return true;
  }
  virtual void startImpl(
      panda_controllers::PandaControllerInterface* control) override {
    return control->start();
  }
  virtual void stopImpl(
      panda_controllers::PandaControllerInterface* control) override {
    return control->stop();
  }

  rclcpp::Service<GetRobotStates>::SharedPtr get_robot_states_server_;
  std::vector<panda_controllers::RobotData*> robot_data_;
};

}
