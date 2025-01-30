// Copyright (c) 2021 Franka Emika GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <franka_mj_hardware/gripper_sim_action_server.hpp>

namespace franka_gripper {
GripperSimActionServer::GripperSimActionServer(const rclcpp::NodeOptions& options, std::string robot_name)
    : Node(robot_name + "_gripper_sim_node", options) {
  // this->declare_parameter("robot_ip", std::string());
  // this->declare_parameter("default_grasp_epsilon.inner", k_default_grasp_epsilon);
  // this->declare_parameter("default_grasp_epsilon.outer", k_default_grasp_epsilon);
  // this->declare_parameter("default_speed", k_default_speed);
  // this->declare_parameter("joint_names", std::vector<std::string>());
  // this->declare_parameter("state_publish_rate", k_default_state_publish_rate);
  // this->declare_parameter("feedback_publish_rate", k_default_feedback_publish_rate);
  // std::string robot_ip;
  // if (!this->get_parameter<std::string>("robot_ip", robot_ip)) {
  //   RCLCPP_FATAL(this->get_logger(), "Parameter 'robot_ip' not set");
  //   throw std::invalid_argument("Parameter 'robot_ip' not set");
  // }
  /*
  state_publish_rate: 50  # [Hz]
    feedback_publish_rate: 30 # [Hz]
    default_speed: 0.1  # [m/s]
    default_grasp_epsilon:
      inner: 0.005 # [m]
      outer: 0.005 # [m]
  */

  this->default_speed_ = 0.1;
  this->default_epsilon_inner_ = 0.005;
  this->default_epsilon_outer_ = 0.005;
  this->cmd_rate_ = std::make_shared<rclcpp::Rate>(1.0/dt);
  this->joint_names_ = {robot_name + "_finger_joint1", robot_name + "_finger_joint2"};
  // if (!this->get_parameter("joint_names", this->joint_names_)) {
  //   RCLCPP_WARN(this->get_logger(), "Parameter 'joint_names' not set");
  //   this->joint_names_ = {"", ""};
  // }

  // if (this->joint_names_.size() != 2) {
  //   RCLCPP_FATAL(this->get_logger(),
  //                "Parameter 'joint_names' needs exactly two arguments, got %ld instead",
  //                this->joint_names_.size());
  //   throw std::invalid_argument("Parameter 'joint_names' has wrong number of arguments");
  // }

  const double kStatePublishRate = 50.0;
  const double kFeedbackPublishRate = 30.0;
  this->future_wait_timeout_ = rclcpp::WallRate(kFeedbackPublishRate).period();

  // RCLCPP_INFO(this->get_logger(), "Trying to establish a connection with the gripper");
  // try {
  //   this->gripper_ = std::make_unique<franka::Gripper>(robot_ip);
  // } catch (const franka::Exception& exception) {
  //   RCLCPP_FATAL(this->get_logger(), exception.what());
  //   throw exception;
  // }
  // RCLCPP_INFO(this->get_logger(), "Connected to gripper");
  const auto kHomingTask = Task::kHoming;
  this->stop_service_ =  // NOLINTNEXTLINE
      create_service<Trigger>("~/stop",
                              [this](std::shared_ptr<Trigger::Request> /*request*/,  // NOLINT
                                     std::shared_ptr<Trigger::Response> response) {  // NOLINT
                                return stopServiceCallback(std::move(response));     // NOLINT
                              });

  this->homing_server_ = rclcpp_action::create_server<Homing>(
      this, "~/homing",
      [this, kHomingTask](auto /*uuid*/, auto /*goal*/) { return handleGoal(kHomingTask); },
      [this, kHomingTask](const auto& /*goal_handle*/) { return handleCancel(kHomingTask); },
      [this](const auto& goal_handle) {
        return std::thread{[goal_handle, this]() { executeHoming(goal_handle); }}.detach();
      });
  const auto kMoveTask = Task::kMove;
  this->move_server_ = rclcpp_action::create_server<Move>(
      this, "~/move",
      [this, kMoveTask](auto /*uuid*/, auto /*goal*/) { return handleGoal(kMoveTask); },
      [this, kMoveTask](const auto& /*goal_handle*/) { return handleCancel(kMoveTask); },
      [this](const auto& goal_handle) {
        return std::thread{[goal_handle, this]() { executeMove(goal_handle); }}.detach();
      });

  const auto kGraspTask = Task::kGrasp;
  this->grasp_server_ = rclcpp_action::create_server<Grasp>(
      this, "~/grasp",
      [this, kGraspTask](auto /*uuid*/, auto /*goal*/) { return handleGoal(kGraspTask); },
      [this, kGraspTask](const auto& /*goal_handle*/) { return handleCancel(kGraspTask); },
      [this](const auto& goal_handle) {
        return std::thread{[goal_handle, this]() { executeGrasp(goal_handle); }}.detach();
      });

  const auto kGripperCommandTask = Task::kGripperCommand;
  this->gripper_command_server_ = rclcpp_action::create_server<GripperCommand>(
      this, "~/gripper_action",
      [this, kGripperCommandTask](auto /*uuid*/, auto /*goal*/) {
        return handleGoal(kGripperCommandTask);
      },
      [this, kGripperCommandTask](const auto& /*goal_handle*/) {
        return handleCancel(kGripperCommandTask);
      },
      [this](const auto& goal_handle) {
        return std::thread{[goal_handle, this]() { onExecuteGripperCommand(goal_handle); }}
            .detach();
      });

  this->joint_states_publisher_ =
      this->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 1);
  this->timer_ = this->create_wall_timer(rclcpp::WallRate(kStatePublishRate).period(),
                                         [this]() { return publishGripperState(); });
  RCLCPP_INFO(get_logger(), "Started gripper sim");
}

void GripperSimActionServer::initGripperPtrs(std::shared_ptr<std::array<double, 3>> states_ptr){
  gripper_states_ptr_ = states_ptr;
  current_gripper_state_ = getGripperState();
}

rclcpp_action::CancelResponse GripperSimActionServer::handleCancel(Task task) {
  RCLCPP_INFO(this->get_logger(), "Received request to handleCancel %s", getTaskName(task).c_str());
  is_canceled_ = true;
  return rclcpp_action::CancelResponse::ACCEPT;
}

rclcpp_action::GoalResponse GripperSimActionServer::handleGoal(Task task) {
  RCLCPP_INFO(this->get_logger(), "Received %s request", getTaskName(task).c_str());
  is_canceled_ = false;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void GripperSimActionServer::executeHoming(const std::shared_ptr<GoalHandleHoming>& goal_handle) {
  const auto kCommand = [this]() { return simGripperHoming(); };
  executeCommand(goal_handle, Task::kHoming, kCommand);
}
bool GripperSimActionServer::simGripperHoming(){
  // homing: The gripper closes, then opens to the maximum width.
  double cur_width;
  double max_width;
  {
    std::lock_guard<std::mutex> lock(gripper_state_mutex_);
    cur_width = current_gripper_state_.width;
    max_width = current_gripper_state_.max_width;
  }
  while(cur_width > 0.001 && !is_canceled_){
    // max width = 0.08
    // command = [0, 255], 255 = open
    // 1 ~ 0.0003138
    cur_width = cur_width - (default_speed_ * dt);
    gripper_states_ptr_->at(0) = std::max(0.0, cur_width / 0.0003137);
    cmd_rate_->sleep();
  }
  while(cur_width < max_width && !is_canceled_){
    cur_width = cur_width + (default_speed_ * dt);
    gripper_states_ptr_->at(0) = std::min(255.0, cur_width / 0.0003137);
    cmd_rate_->sleep();
  }
  return true;
}

void GripperSimActionServer::executeMove(const std::shared_ptr<GoalHandleMove>& goal_handle) {
  auto command = [goal_handle, this]() {
    const auto kGoal = goal_handle->get_goal();
    return simGripperMove(kGoal->width, kGoal->speed);
  };
  executeCommand(goal_handle, Task::kMove, command);
}
bool GripperSimActionServer::simGripperMove(double width, double speed){
  double cur_width;
  double max_width;
  {
    std::lock_guard<std::mutex> lock(gripper_state_mutex_);
    cur_width = current_gripper_state_.width;
    max_width = current_gripper_state_.max_width;
  }
  double cur_speed = speed <= 0.0 ? default_speed_ : speed;
  double dist = std::min(max_width, width) - cur_width;
  double sgn = dist < 0 ? -1.0 : 1.0;
  int max_count = std::abs(dist / (cur_speed * dt));
  for(int count=0; count<max_count; count++){
    if(is_canceled_){
      break;
    }
    cur_width = cur_width + (sgn * cur_speed * dt);
    gripper_states_ptr_->at(0) = std::min(std::max(0.0, cur_width / 0.0003137), 255.0);
    cmd_rate_->sleep();
  }
  return true;
}

void GripperSimActionServer::executeGrasp(const std::shared_ptr<GoalHandleGrasp>& goal_handle) {
  auto command = [goal_handle, this]() {
    const auto kGoal = goal_handle->get_goal();
    return simGripperGrasp(kGoal->width, kGoal->speed, kGoal->force, kGoal->epsilon.inner,
                           kGoal->epsilon.outer);
  };
  executeCommand(goal_handle, Task::kGrasp, command);
}
bool GripperSimActionServer::simGripperGrasp(double width, double speed, double force, 
                                              double epsilon_inner, double epsilon_outer){
  // same as move, except max_count+50 to ensure "proper" grasping.
  // epsilon_inner and epsilon_outer are not used, since they are just confusing.
  // force is not really implemented either, since the real franka doesn't implement that...
  double cur_width;
  double max_width;
  {
    std::lock_guard<std::mutex> lock(gripper_state_mutex_);
    cur_width = current_gripper_state_.width;
    max_width = current_gripper_state_.max_width;
  }
  double cur_speed = speed <= 0.0 ? default_speed_ : speed;
  double dist = std::min(max_width, width) - cur_width;
  double sgn = dist < 0 ? -1.0 : 1.0;
  int max_count = std::abs(dist / (cur_speed * dt)) + 50;
  for(int count=0; count<max_count; count++){
    if(is_canceled_){
      break;
    }
    cur_width = cur_width + (sgn * cur_speed * dt);
    gripper_states_ptr_->at(0) = std::min(std::max(0.0, cur_width / 0.0003137), 255.0);
    cmd_rate_->sleep();
  }
  return true;
}

void GripperSimActionServer::onExecuteGripperCommand(
    const std::shared_ptr<GoalHandleGripperCommand>& goal_handle) {
  const auto kGoal = goal_handle->get_goal();
  const double kTargetWidth = 2 * kGoal->command.position;

  std::unique_lock<std::mutex> guard(gripper_state_mutex_);
  constexpr double kSamePositionThreshold = 1e-4;
  auto result = std::make_shared<control_msgs::action::GripperCommand::Result>();
  const double kCurrentWidth = current_gripper_state_.width;
  if (kTargetWidth > current_gripper_state_.max_width || kTargetWidth < 0) {
    RCLCPP_ERROR(this->get_logger(),
                 "GripperServer: Commanding out of range width! max_width = %f command = %f",
                 current_gripper_state_.max_width, kTargetWidth);
    goal_handle->abort(result);
    return;
  }
  if (std::abs(kTargetWidth - kCurrentWidth) < kSamePositionThreshold) {
    result->effort = 0;
    result->position = kCurrentWidth;
    result->reached_goal = true;
    result->stalled = false;
    goal_handle->succeed(result);
    return;
  }
  // guard.unlock();
  auto command = [kTargetWidth, kCurrentWidth, kGoal, this]() {
    if (kTargetWidth >= kCurrentWidth) {
      return simGripperMove(kTargetWidth, default_speed_);
    }
    return simGripperGrasp(kTargetWidth, default_speed_, kGoal->command.max_effort,
                           default_epsilon_inner_, default_epsilon_outer_);
  };

  executeGripperCommand(goal_handle, command);
}

void GripperSimActionServer::executeGripperCommand(
    const std::shared_ptr<GoalHandleGripperCommand>& goal_handle,
    const std::function<bool()>& command_handler) {
  const auto kTaskName = getTaskName(Task::kGripperCommand);
  RCLCPP_INFO(this->get_logger(), "Gripper %s...", kTaskName.c_str());

  auto command_execution_thread = [command_handler, this]() {
    auto result = std::make_shared<GripperCommand::Result>();
    try {
      result->reached_goal = command_handler();
    } catch (const std::exception& e) {
      result->reached_goal = false;
      RCLCPP_ERROR(this->get_logger(), e.what());
    }
    return result;
  };

  std::future<std::shared_ptr<typename GripperCommand ::Result>> result_future =
      std::async(std::launch::async, command_execution_thread);

  while (!resultIsReady(result_future, future_wait_timeout_) && rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      simGripperStop();
      auto result = result_future.get();
      RCLCPP_INFO(get_logger(), "Gripper %s canceled", kTaskName.c_str());
      goal_handle->canceled(result);
      return;
    }
    publishGripperCommandFeedback(goal_handle);
  }
  if (rclcpp::ok()) {
    const auto kResult = result_future.get();
    std::lock_guard<std::mutex> guard(gripper_state_mutex_);
    kResult->position = current_gripper_state_.width;
    kResult->effort = 0.;
    if (kResult->reached_goal) {
      RCLCPP_INFO(get_logger(), "Gripper %s succeeded", kTaskName.c_str());
      goal_handle->succeed(kResult);
    } else {
      RCLCPP_INFO(get_logger(), "Gripper %s failed", kTaskName.c_str());
      goal_handle->abort(kResult);
    }
  }
}

void GripperSimActionServer::stopServiceCallback(const std::shared_ptr<Trigger::Response>& response) {
  RCLCPP_INFO(this->get_logger(), "Stopping gripper_...");
  auto action_result = withResultGenerator<Homing>([this]() { return simGripperStop(); })();
  response->success = action_result->success;
  response->message = action_result->error;
  if (response->success) {
    RCLCPP_INFO(this->get_logger(), "Gripper stopped");
  } else {
    RCLCPP_INFO(this->get_logger(), "Gripper could not be stopped");
  }
  if (!response->message.empty()) {
    RCLCPP_ERROR(this->get_logger(), response->message.c_str());
  }
}

bool GripperSimActionServer::simGripperStop(){
  is_canceled_ = true;
  std::lock_guard<std::mutex> lock(gripper_state_mutex_);
  gripper_states_ptr_->at(0) = current_gripper_state_.width / 0.0003137;
  std::cout << "Should stop " << std::endl;
  return true;
}

franka::GripperState GripperSimActionServer::getGripperState(){
  // This doesn't need a mutex, since it doesn't modify current_gripper_state_
  franka::GripperState new_state;
  new_state.temperature = 0;
  new_state.time = franka::Duration(0); // time is honestly kinda useless?
  new_state.max_width = 0.08; // temporarily, until I figure out the sites
  new_state.width = gripper_states_ptr_->at(1) * 2; // symmetric, so just a x2 should do it
  new_state.is_grasped = false; // this should be decided by reading the force at the actuator, and the current width
  return new_state;
}

void GripperSimActionServer::publishGripperState() {
  std::lock_guard<std::mutex> lock(gripper_state_mutex_);
  try {
    current_gripper_state_ = getGripperState();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
  }
  sensor_msgs::msg::JointState joint_states;
  joint_states.header.stamp = this->now();
  joint_states.name.push_back(this->joint_names_[0]);
  joint_states.name.push_back(this->joint_names_[1]);
  joint_states.position.push_back(current_gripper_state_.width / 2);
  joint_states.position.push_back(current_gripper_state_.width / 2);
  joint_states.velocity.push_back(0.0);
  joint_states.velocity.push_back(0.0);
  joint_states.effort.push_back(0.0);
  joint_states.effort.push_back(0.0);
  joint_states_publisher_->publish(joint_states);
}

void GripperSimActionServer::publishGripperCommandFeedback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GripperCommand>>& goal_handle) {
  auto gripper_feedback = std::make_shared<GripperCommand::Feedback>();
  std::lock_guard<std::mutex> lock(gripper_state_mutex_);
  gripper_feedback->position = current_gripper_state_.width;
  gripper_feedback->effort = 0.;
  goal_handle->publish_feedback(gripper_feedback);
}
}  // namespace franka_gripper

RCLCPP_COMPONENTS_REGISTER_NODE(franka_gripper::GripperSimActionServer)  // NOLINT