#pragma once

#include <atomic>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/empty.hpp>

#include <multi_mode_controller/utils/sync_service_client.hpp>
#include <multi_mode_control_msgs/msg/wrench.hpp>
#include <multi_mode_control_msgs/srv/set_controllers.hpp>
#include <multi_mode_control_msgs/srv/get_robot_states.hpp>
#include <panda_motion_generator_msgs/msg/set_time_scaling.hpp>
#include <panda_motion_generators/base/motion_generator_base.h>

namespace panda_motion_generators {

enum struct State {IDLE, MOVING, RECOVERY, WAIT_FOR_RECOVERY};

using std::placeholders::_1;
using std::placeholders::_2;
using std::shared_ptr;

template <typename ViaPose, 
          typename ControlGoal, 
          typename Config,
          typename ControlGoalMsg, 
          typename Action>
class MotionGeneratorRosInterface :
    public virtual MotionGeneratorBase<ViaPose, ControlGoal, Config>{
 public:
  MotionGeneratorRosInterface() = delete;
  /**
   * Constructor for the MotionGeneratorRosInterface. 
   * Initializes the publishers, subscribers, service clients and action server.
   * Pub, sub, action server are part of the main node, rclcpp::Node node_.
   * The robot state getter client is part of the service node, ExecutorNode get_robot_state_node_.
   * The control setter client is part of the service node, ExecutorNode set_controllers_node_.
   * The parameter client is part of the rclcpp::Node param_node_.
   * @param[in] action_name Name of the action server.
   * @param[in] robot_state_srv_name Fully qualified name of the getRobotState service server, e.g. /panda/get_robot_states
   * @param[in] controller_name Name of the multi-mode controller, e.g. single_multi_mode_controller
   * @param[in] desired_pose_name Fully qualified name of the topic that subscribes to the desired pose. 
   */
  MotionGeneratorRosInterface(std::string action_name, 
                              std::string robot_state_srv_name, 
                              std::string controller_name, 
                              std::string desired_pose_name) : get_robot_state_node_(robot_state_srv_name),
                                                               set_controllers_node_("set_controllers") {
    // TODO: Remember to resolve namespaces later
    // TODO: Look into how to make service calls cleaner with a single neat executor
    name_ = action_name;
    node_ = rclcpp::Node::make_shared(action_name + "_node");
    param_node_ = rclcpp::Node::make_shared(action_name + "_param_node");
    parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this->param_node_, controller_name);
      while (!parameters_client->wait_for_service(0.5s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
          rclcpp::shutdown();
        }
        RCLCPP_INFO(this->node_->get_logger(), "service not available, waiting again...");
      }
    as_ = rclcpp_action::create_server<Action>(
      node_,
      action_name,
      std::bind(&MotionGeneratorRosInterface::handle_goal, this, _1, _2),
      std::bind(&MotionGeneratorRosInterface::handle_cancel, this, _1),
      std::bind(&MotionGeneratorRosInterface::handle_accepted, this, _1));
    this->goal_pub_ = this->node_->create_publisher<ControlGoalMsg>(desired_pose_name, 1);
    this->time_scaling_sub_ = this->node_->create_subscription<panda_motion_generator_msgs::msg::SetTimeScaling>("set_time_scaling", 1,
        std::bind(&MotionGeneratorRosInterface::setTimeScalingCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(node_->get_logger(), "Initialization of MGRosInterface done with name %s", action_name.c_str());

  }
  virtual ~MotionGeneratorRosInterface() = default;
  void spinROSNode(){
    rclcpp::spin(this->node_);
  }
 protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr param_node_;
  panda_controllers::ExecutorNode<multi_mode_control_msgs::srv::SetControllers> set_controllers_node_; // for setting controllers
  panda_controllers::ExecutorNode<multi_mode_control_msgs::srv::GetRobotStates> get_robot_state_node_; // for getting robot state
  std::shared_ptr<rclcpp::SyncParametersClient> parameters_client;
 private:
  typename rclcpp_action::Server<Action>::SharedPtr as_;
  std::mutex abort_reason_mutex_;
  std::string abort_reason_, name_;
  std::recursive_mutex state_mutex_;
  State state_ = State::IDLE;
  std::atomic<bool> abort_;
  std::atomic<bool> check_trajectory_;
  std::atomic<bool> contact_;
  std::atomic<bool> has_offset_;
  std::atomic<bool> start_recovery_;
  std::atomic<double> time_scaling_;
  std::atomic<int> steps_ = 0;
  std::shared_ptr<typename Action::Feedback> feedback_;
  double progress_, time_to_completion_;
  ControlGoal control_goal_;
  typename rclcpp::Publisher<ControlGoalMsg>::SharedPtr goal_pub_;
  rclcpp::Subscription<panda_motion_generator_msgs::msg::SetTimeScaling>::SharedPtr time_scaling_sub_;
  rclcpp::Subscription<multi_mode_control_msgs::msg::Wrench>::SharedPtr contact_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr offset_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr start_recovery_sub_;
  
  virtual bool checkGoal(shared_ptr<const typename Action::Goal>& goal) = 0;
  
  virtual ControlGoalMsg convertToMsg(const ControlGoal& goal) = 0;


  //***** Action server functions *****//
  /**
   * Callback function for the action server.
   * Runs checks on whether the goal is valid or not,
   * and returns a response accordingly.
  */
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    shared_ptr<const typename Action::Goal> goal)
  {
    RCLCPP_INFO(this->node_->get_logger(), (name_ + ": Received new goal " + printGoal(goal) + ".").c_str());
    (void)uuid;
    // checks for the goal needs to be done here
    // rclcpp has no fine-grained abort/preempt; just a rejection.
    if (!checkGoal(goal)) {
      RCLCPP_WARN(this->node_->get_logger(), (name_ + ": Goal has been rejected due to goal constraint "
               "violation. Error: " + this->error_).c_str());
      state_ = State::IDLE;
      return rclcpp_action::GoalResponse::REJECT;
      
    }
    RCLCPP_INFO(this->node_->get_logger(), "Passed checkGoal");
    ViaPose current;
    if (!getCurrentPose(current)) {
      RCLCPP_WARN(this->node_->get_logger(), (name_ + ": Could not get current robot pose, aborting goal " +
               printGoal(goal) + ".").c_str());
      // as_->setAborted(generateAbortedResult(), name_ + ": Could not get current "
      //                "robot pose, aborting goal " + printGoal(goal) + ".");
      state_ = State::IDLE;
      return rclcpp_action::GoalResponse::REJECT;
    }
    this->setCurrentPose(current);
    RCLCPP_INFO(this->node_->get_logger(), "Passed getCurrentPose");
    if (!this->generateTrajectory(getViaPoses(goal), getConfig(goal))) {
      {
        std::lock_guard<std::mutex> lock(abort_reason_mutex_);
        abort_reason_ = "being unable to generate the trajectory.";
      }
      // abort_ = true;
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->node_->get_logger(), "Passed generateTrajectory");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    shared_ptr<rclcpp_action::ServerGoalHandle<Action>> goal_handle)
  {
    RCLCPP_INFO(this->node_->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    // when a cancel request is received, set abort to true so that updateState can react
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  /**
   * Callback function for action server.
   * When goal is accepted, starts up a new thread with executeGoal().
  */
  void handle_accepted(const shared_ptr<rclcpp_action::ServerGoalHandle<Action>> goal_handle)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    RCLCPP_INFO(this->node_->get_logger(), "Goal accepted");
    std::thread{std::bind(&MotionGeneratorRosInterface::executeGoal, this, _1), goal_handle}.detach();
  }
  /**
   * The core while(...) loop.
   * After setting some initial parameters upon receiving a new goal,
   * runs a while loop with executeTrajectory.
  */
  void executeGoal(const shared_ptr<rclcpp_action::ServerGoalHandle<Action>> goal_handle) {
    const auto goal = goal_handle->get_goal();
    // auto feedback = std::make_shared<typename Action::Feedback>();
    // feedback->progress
    check_trajectory_ = false;
    abort_ = false;
    contact_ = false;
    has_offset_ = false;
    start_recovery_ = false;
    progress_ = 0;
    prepareMotion();
    rclcpp::Rate r(1/this->T);
    state_ = State::MOVING;
    check_trajectory_ = true;
    if (steps_ > 0) {
      RCLCPP_INFO(this->node_->get_logger(), "%s: Setting time scaling to %f.", name_.c_str(),
               time_scaling_.load());
      this->setTimeScaling(time_scaling_, steps_);
      steps_ = 0;
    }
    this->start();
    while (updateState(goal_handle)) {
      executeTrajectory(goal_handle);
      r.sleep();
    }
  }
  /**
   * Publishes the control goal that is obtained from the step function,
   * and publishes feedback to the action client to report on the progress.
  */
  void executeTrajectory(const shared_ptr<rclcpp_action::ServerGoalHandle<Action>> goal_handle) {
    if (state_ == State::MOVING || state_ == State::RECOVERY) {
      if (this->step(has_offset_, control_goal_, progress_,
          time_to_completion_)) {
        goal_pub_->publish(convertToMsg(control_goal_));
      }
      feedback_ = generateFeedback(progress_, time_to_completion_);
      goal_handle->publish_feedback(feedback_);
    }
  }
  /**
   * Checks for any cancel condition, such as cancel requests from the action client.
   * Performs time scaling if the time scaling subscriber receives a request.
   * Finally, checks if the member var progress_ of executeTrajectory exceeds 1,
   * then publishes a success result to the client to indicate that the trajectory is done.
  */
  bool updateState(const shared_ptr<rclcpp_action::ServerGoalHandle<Action>> goal_handle) {
    
    // Check for cancel conditions
    if (abort_ || goal_handle->is_canceling()) {
      std::lock_guard<std::mutex> lock(abort_reason_mutex_);
      auto goal = goal_handle->get_goal();
      RCLCPP_WARN(this->node_->get_logger(), (name_ + ": Goal " + printGoal(goal) + " has been aborted "
               "due to " + abort_reason_ + ".").c_str());
      // as_.setAborted(generateAbortedResult(), name_ + ": Goal " +
      //                printGoal(goal) + " has been aborted due to " +
      //                abort_reason_ + ".");
      auto result = generateAbortedResult();
      goal_handle->canceled(result);
      state_ = State::IDLE;
      abort_ = false;
      return false;
    }
    // do scaling if necessary
    if (steps_ > 0) {
      RCLCPP_INFO(this->node_->get_logger(), "%s: Setting time scaling to %f in %i steps.", name_.c_str(),
               time_scaling_.load(), steps_.load());
      this->setTimeScaling(time_scaling_, steps_);
      steps_ = 0;
    }

    // now do switching
    switch (state_) {
      case State::MOVING: { // without rf, we only have two states: moving or idle.
        if (progress_ >= 1) {
          auto goal = goal_handle->get_goal();
          RCLCPP_INFO(this->node_->get_logger(), (name_ + ": Goal " + printGoal(goal) + " has been "
                   "executed successfully.").c_str());
          auto result = generateSuccessResult();
          goal_handle->succeed(result);
          //(generateSuccessResult(), name_ + ": motion finished successful.");
          state_ = State::IDLE;
          return false;
        }
        break;
      }
      case State::IDLE: {
        break;
      }
      
    }
    return true;
  }
  //***** Action server functions *****//

  // Result and feedback generator functions //
  virtual std::shared_ptr<typename Action::Result> generateAbortedResult() = 0;
  virtual std::shared_ptr<typename Action::Result> generateFailureResult() = 0;
  virtual std::shared_ptr<typename Action::Feedback> generateFeedback(const double& progress,
                                    const double& time_to_completion) = 0;
  virtual std::shared_ptr<typename Action::Result> generatePreemptedResult() = 0;
  virtual std::shared_ptr<typename Action::Result> generateRejectedResult() = 0;
  virtual std::shared_ptr<typename Action::Result> generateSuccessResult() = 0;

  // Utility functions
  virtual Config getConfig(shared_ptr<const typename Action::Goal>& goal) = 0;
  virtual bool getCurrentPose(ViaPose& out) = 0;
  virtual std::vector<ViaPose> getViaPoses(shared_ptr<const typename Action::Goal>& goal) = 0;
  /**
   * The function for preparing the multi-mode controller to execute the trajectory.
   * It should, at the minimum, ensure that the correct controller is running.
  */
  virtual void prepareMotion() = 0;
  virtual std::string printGoal(shared_ptr<const typename Action::Goal>& goal) {
    return "";
  }
  
  void setTimeScalingCallback(
      const panda_motion_generator_msgs::msg::SetTimeScaling& msg) {
    time_scaling_ = msg.scaling;
    steps_ = msg.steps;
  }

  // unused, rf callbacks
  void hasOffsetCallback(const std_msgs::msg::Bool& msg) {
    has_offset_ = msg.data;
  }
  void startRecoveryCallback(const std_msgs::msg::Empty& msg) {
    start_recovery_ = true;
  }
  void contactCallback(const multi_mode_control_msgs::msg::Wrench& msg) {
    contact_ = true;
  }
  
};

}
