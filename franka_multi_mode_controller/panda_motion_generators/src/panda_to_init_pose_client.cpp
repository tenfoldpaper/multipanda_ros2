#include <chrono>
#include <cinttypes>
#include <memory>

#include "panda_motion_generator_msgs/action/joint_via_motion.hpp"
#include "panda_motion_generator_msgs/msg/joint_pose.hpp"
#include "multi_mode_control_msgs/srv/get_robot_states.hpp"
#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"

using JVMotion = panda_motion_generator_msgs::action::JointViaMotion;
rclcpp::Node::SharedPtr jvm_node = nullptr;

using namespace std::chrono_literals;

void feedback_callback(
  rclcpp_action::ClientGoalHandle<JVMotion>::SharedPtr,
  const std::shared_ptr<const JVMotion::Feedback> feedback)
{
  RCLCPP_INFO(
    jvm_node->get_logger(),
    "Next number in sequence received: %f",
    feedback->progress);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  jvm_node = rclcpp::Node::make_shared("init_pose_action_client");
  auto action_client = rclcpp_action::create_client<JVMotion>(jvm_node, "joint_via_motion");

  if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
    RCLCPP_ERROR(jvm_node->get_logger(), "Action server not available after waiting");
    return 1;
  }

  // Populate a goal
  auto goal_msg = JVMotion::Goal();
  auto joint_pose = panda_motion_generator_msgs::msg::JointPose();

  // Quick and dirty way of getting current joints
  rclcpp::Client<multi_mode_control_msgs::srv::GetRobotStates>::SharedPtr client =
    jvm_node->create_client<multi_mode_control_msgs::srv::GetRobotStates>("/panda/get_robot_states");
  auto request = std::make_shared<multi_mode_control_msgs::srv::GetRobotStates::Request>();
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  std::cout << "Async \n";
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(jvm_node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    std::cout << "get\n";
    auto& q = result.get()->states[0].q;
    std::vector<double> temp(7);
    for(int i = 0; i < 7; i++){
      temp[i] = q[i];
    }
    joint_pose.q = temp;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service get_robot_state");
  }
  goal_msg.via_poses.push_back(joint_pose);
  joint_pose.q = std::vector<double>{0.,-0.7818251828441144,0.,-2.371614246736288,0.,1.571329215974543,0.7851527623989699};
  goal_msg.via_poses.push_back(joint_pose);
  goal_msg.v_scale = 0.1;

  RCLCPP_INFO(jvm_node->get_logger(), "Sending goal");
  // Ask server to achieve some goal and wait until it's accepted
  auto send_goal_options = rclcpp_action::Client<JVMotion>::SendGoalOptions();
  send_goal_options.feedback_callback = feedback_callback;
  auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);

  if (rclcpp::spin_until_future_complete(jvm_node, goal_handle_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(jvm_node->get_logger(), "send goal call failed :(");
    return 1;
  }

  rclcpp_action::ClientGoalHandle<JVMotion>::SharedPtr goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(jvm_node->get_logger(), "Goal was rejected by server");
    return 1;
  }

  // Wait for the server to be done with the goal
  auto result_future = action_client->async_get_result(goal_handle);

  RCLCPP_INFO(jvm_node->get_logger(), "Waiting for result");
  if (rclcpp::spin_until_future_complete(jvm_node, result_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(jvm_node->get_logger(), "get result call failed :(");
    return 1;
  }

  rclcpp_action::ClientGoalHandle<JVMotion>::WrappedResult wrapped_result = result_future.get();

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(jvm_node->get_logger(), "Goal was aborted");
      action_client.reset();
      jvm_node.reset();
      rclcpp::shutdown();
      return 1;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(jvm_node->get_logger(), "Goal was canceled");
      action_client.reset();
      jvm_node.reset();
      rclcpp::shutdown();
      return 1;
    default:
      RCLCPP_ERROR(jvm_node->get_logger(), "Unknown result code");
      action_client.reset();
      jvm_node.reset();
      rclcpp::shutdown();
      return 1;
  }

  RCLCPP_INFO(jvm_node->get_logger(), "result received: %d", wrapped_result.result->result.state);
  
  action_client.reset();
  jvm_node.reset();
  rclcpp::shutdown();
  return 0;
}
