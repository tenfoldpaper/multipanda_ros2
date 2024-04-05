#include <chrono>
#include <cinttypes>
#include <memory>

#include "panda_motion_generator_msgs/action/joint_via_motion.hpp"
#include "panda_motion_generator_msgs/msg/joint_pose.hpp"
#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"

using JVMotion = panda_motion_generator_msgs::action::JointViaMotion;
rclcpp::Node::SharedPtr jvm_node = nullptr;

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
  jvm_node = rclcpp::Node::make_shared("minimal_action_client");
  auto action_client = rclcpp_action::create_client<JVMotion>(jvm_node, "joint_via_motion");

  if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
    RCLCPP_ERROR(jvm_node->get_logger(), "Action server not available after waiting");
    return 1;
  }

  // Populate a goal
  auto goal_msg = JVMotion::Goal();
  auto joint_pose = panda_motion_generator_msgs::msg::JointPose();
  joint_pose.q = std::vector<double>{0.004986179361093002, -0.6725671965472394, -0.1266404387692794, -2.1807457477837273, -0.013390807396007906, 1.4507449921897155, 0.828980047582415};
  goal_msg.via_poses.push_back(joint_pose);
  joint_pose.q = std::vector<double>{0.34302745448079025, -0.558335750055592, -0.6795609559594702, -2.09007502857004, -0.013392886983345734, 1.4523769579369084, 0.8089155919190073};
  goal_msg.via_poses.push_back(joint_pose);
  joint_pose.q = std::vector<double>{0.42853091345753586,-0.6186942686638369,-0.9891775694311711,-2.0899018328482644,-0.012344067827694944,1.452391550654897,0.7979433008368835};
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
