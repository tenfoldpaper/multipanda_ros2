#include <chrono>
#include <cinttypes>
#include <memory>

#include "panda_motion_generator_msgs/action/cartesian_via_motion.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"
#include <tf2/LinearMath/Transform.h>

using CVMotion = panda_motion_generator_msgs::action::CartesianViaMotion;
rclcpp::Node::SharedPtr jvm_node = nullptr;

void feedback_callback(
  rclcpp_action::ClientGoalHandle<CVMotion>::SharedPtr,
  const std::shared_ptr<const CVMotion::Feedback> feedback)
{
  RCLCPP_INFO(
    jvm_node->get_logger(),
    "Next number in sequence received: %f",
    feedback->progress);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  jvm_node = rclcpp::Node::make_shared("cartesian_minimal_action_client");
  auto action_client = rclcpp_action::create_client<CVMotion>(jvm_node, "cartesian_via_motion");

  if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
    RCLCPP_ERROR(jvm_node->get_logger(), "Action server not available after waiting");
    return 1;
  }

  // Populate a goal
  auto goal_msg = CVMotion::Goal();
  auto cartesian_pose = geometry_msgs::msg::Pose();
  tf2::Matrix3x3 rMat = tf2::Matrix3x3(0.7470910402632442, -0.6631838340596983, 0.04519048351436458, -0.6647207827829965, -0.7454762486063343, 0.049106452734719044, 0.0011218265213126474, -0.06672604443322967, -0.9977707033680387);
  tf2::Vector3 tVec = tf2::Vector3(0.34318673468886807,-0.018589977965981136,0.5392410056104563);
  tf2::Transform tf = tf2::Transform(rMat, tVec);
  auto rQuat = tf.getRotation();
  cartesian_pose.orientation.x = rQuat.getX();
  cartesian_pose.orientation.y = rQuat.getY();
  cartesian_pose.orientation.z = rQuat.getZ();
  cartesian_pose.orientation.w = rQuat.getW();
  cartesian_pose.position.x = 0.34318673468886807;
  cartesian_pose.position.y = -0.018589977965981136;
  cartesian_pose.position.z = 0.5392410056104563;
  
  auto cartesian_pose2 = cartesian_pose;
  cartesian_pose2.position.y = -0.118589977965981136;

  // back and forth
  goal_msg.via_poses.push_back(cartesian_pose);
  goal_msg.via_poses.push_back(cartesian_pose2);
  goal_msg.via_poses.push_back(cartesian_pose);
  goal_msg.v_scale = 0.5;

  RCLCPP_INFO(jvm_node->get_logger(), "Sending goal");
  // Ask server to achieve some goal and wait until it's accepted
  auto send_goal_options = rclcpp_action::Client<CVMotion>::SendGoalOptions();
  send_goal_options.feedback_callback = feedback_callback;
  auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);

  if (rclcpp::spin_until_future_complete(jvm_node, goal_handle_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(jvm_node->get_logger(), "send goal call failed :(");
    return 1;
  }

  rclcpp_action::ClientGoalHandle<CVMotion>::SharedPtr goal_handle = goal_handle_future.get();
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

  rclcpp_action::ClientGoalHandle<CVMotion>::WrappedResult wrapped_result = result_future.get();

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
