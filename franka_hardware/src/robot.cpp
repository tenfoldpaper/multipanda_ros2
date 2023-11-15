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

#include <franka_hardware/robot.hpp>

#include <cassert>
#include <mutex>

#include <stdio.h>
#include <iostream>
#include <franka/control_tools.h>
#include <rclcpp/logging.hpp>

namespace franka_hardware {

Robot::Robot(const std::string& robot_ip, const rclcpp::Logger& logger) {
  franka::RealtimeConfig rt_config = franka::RealtimeConfig::kEnforce;
  if (!franka::hasRealtimeKernel()) {
    rt_config = franka::RealtimeConfig::kIgnore;
    RCLCPP_WARN(
        logger,
        "You are not using a real-time kernel. Using a real-time kernel is strongly recommended!");
  }
  try{
    robot_ = std::make_unique<franka::Robot>(robot_ip, rt_config);
    setDefaultParams();
  }
  catch(franka::ControlException& e){
    RCLCPP_ERROR(logger, "Robot is in control error state! Please trigger automatic recovery first.");
    RCLCPP_ERROR(logger, "Error: %s\nType: %s", e.what(), typeid(e).name());
    setError(true);
  }
  catch(franka::CommandException& e){
    RCLCPP_ERROR(logger, "Robot is in command error state! Please trigger automatic recovery first.");
    RCLCPP_ERROR(logger, "Error: %s\nType: %s", e.what(), typeid(e).name());
    setError(true);

  }
  catch(std::exception& e){
    RCLCPP_ERROR(logger, "Unrecoverable error.\nError:%s\nType: %s", e.what(), typeid(e).name());
    throw franka::Exception("ERROR");
  }
  // will need a boolean to set this for the first time;
  // after the necessary runtime services, this part can be removed.
  
  tau_command_.fill({});
  joint_velocity_command_.fill({});
  cartesian_position_command_.fill({});
  cartesian_velocity_command_.fill({});
  model_ = std::make_unique<franka::Model>(robot_->loadModel());
  franka_hardware_model_ = std::make_unique<Model>(model_.get());
}

Robot::~Robot() {
  stopRobot();
}

void Robot::write(const std::array<double, 7>& efforts, 
                  const std::array<double, 7>& joint_positions, 
                  const std::array<double, 7>& joint_velocities,
                  const std::array<double, 16>& cartesian_positions,
                  const std::array<double, 6>& cartesian_velocities) {
  std::lock_guard<std::mutex> lock(write_mutex_);
  tau_command_ = efforts;
  joint_position_command_ = joint_positions;
  joint_velocity_command_ = joint_velocities;
  cartesian_position_command_ = cartesian_positions;
  cartesian_velocity_command_ = cartesian_velocities;
}

franka::RobotState Robot::read() {
  std::lock_guard<std::mutex> lock(read_mutex_);
  return {current_state_};
}
franka_hardware::Model* Robot::getModel() {
  return franka_hardware_model_.get();
}

void Robot::stopRobot() {
  if (!stopped_) {
    finish_ = true;
    control_thread_->join();
    finish_ = false;
    stopped_ = true;
  }
}

// Joint-level controls
void Robot::initializeTorqueControl() {
  assert(isStopped());
  stopped_ = false;
  std::cout << "Initializing joint torque control" << std::endl;
  const auto kTorqueControl = [this]() {
    try{
      robot_->control(
        [this](const franka::RobotState& state, const franka::Duration& /*period*/) {
          {
            std::lock_guard<std::mutex> lock(read_mutex_);
            current_state_ = state;
          }
          std::lock_guard<std::mutex> lock(write_mutex_);
          franka::Torques out(tau_command_);
          out.motion_finished = finish_;
          return out;
        },
        true, franka::kMaxCutoffFrequency);
    }
    catch(franka::ControlException& e){
      std::cout <<  e.what() << std::endl;
      setError(true);
    }
  };
  control_thread_ = std::make_unique<std::thread>(kTorqueControl);
}

void Robot::initializeJointPositionControl() {
  assert(isStopped());
  stopped_ = false;
  std::cout << "Initializing joint position control" << std::endl;
  const auto kJointPositionControl = [this]() {
    try{

    robot_->control(
      [this](const franka::RobotState& state, const franka::Duration& /*period*/) {
        {
          std::lock_guard<std::mutex> lock(read_mutex_);
          current_state_ = state;
        }
        std::lock_guard<std::mutex> lock(write_mutex_);
        franka::JointPositions out(joint_position_command_);
        out.motion_finished = finish_;
        return out;
      });
    }
    catch(franka::ControlException& e){
      std::cout <<  e.what() << std::endl;
      setError(true);
    }
      
  };
  control_thread_ = std::make_unique<std::thread>(kJointPositionControl);
}

void Robot::initializeJointVelocityControl() {
  assert(isStopped());
  stopped_ = false;
  std::cout << "Initializing joint velocity control" << std::endl;
  const auto kJointVelocityControl = [this]() {
    try{
      robot_->control(
          [this](const franka::RobotState& state, const franka::Duration& /*period*/) {
            {
              std::lock_guard<std::mutex> lock(read_mutex_);
              current_state_ = state;
            }
            std::lock_guard<std::mutex> lock(write_mutex_);
            franka::JointVelocities out(joint_velocity_command_);
            out.motion_finished = finish_;
            return out;
          });
      }
    catch(franka::ControlException& e){
      std::cout <<  e.what() << std::endl;
      setError(true);
    }
  };
  control_thread_ = std::make_unique<std::thread>(kJointVelocityControl);
}

// Cartesian controls
void Robot::initializeCartesianVelocityControl() {
  assert(isStopped());
  stopped_ = false;
  std::cout << "Initializing cartesian velocity control" << std::endl;
  const auto kCartesianVelocityControl = [this]() {
    try{
      robot_->control(
          [this](const franka::RobotState& state, const franka::Duration& /*period*/) {
            {
              std::lock_guard<std::mutex> lock(read_mutex_);
              current_state_ = state;
            }
            std::lock_guard<std::mutex> lock(write_mutex_);
            franka::CartesianVelocities out(cartesian_velocity_command_);
            out.motion_finished = finish_;
            return out;
          });
      }
    catch(franka::ControlException& e){
      std::cout <<  e.what() << std::endl;
      setError(true);
    }
  };
  control_thread_ = std::make_unique<std::thread>(kCartesianVelocityControl);
}

void Robot::initializeCartesianPositionControl() {
  assert(isStopped());
  stopped_ = false;
  std::cout << "Initializing cartesian position control" << std::endl;
  const auto kCartesianPositionControl = [this]() {
    try{
      robot_->control(
          [this](const franka::RobotState& state, const franka::Duration& /*period*/) {
            {
              std::lock_guard<std::mutex> lock(read_mutex_);
              current_state_ = state;
            }
            std::lock_guard<std::mutex> lock(write_mutex_);
            franka::CartesianPose out(cartesian_position_command_);
            out.motion_finished = finish_;
            return out;
          });
      }
    catch(franka::ControlException& e){
      std::cout <<  e.what() << std::endl;
      setError(true);
    }
  };
  control_thread_ = std::make_unique<std::thread>(kCartesianPositionControl);
}

void Robot::initializeContinuousReading() {
  std::cout << "Initializing continuous reading" << std::endl;
  assert(isStopped());
  stopped_ = false;
  const auto kReading = [this]() {
    try{
      robot_->read([this](const franka::RobotState& state) {
        {
          std::lock_guard<std::mutex> lock(read_mutex_);
          current_state_ = state;
        }
        std::cout << "normal: " << current_state_.q[6] << " " << current_state_.robot_mode << std::endl;
        if(current_state_.robot_mode == franka::RobotMode::kReflex){
          throw franka::ControlException("Reflex!");
        }
        return !finish_;
      });

    }
    catch(franka::ControlException& e){
      std::cout <<  e.what() << std::endl;
      setError(true);
    }
  };
  const auto kErrorReading = [this](){
    while(this->hasError()){
      std::lock_guard<std::mutex> lock(read_mutex_);
      current_state_ = robot_->readOnce();
      std::cout << "error: " << current_state_.q[6] << " " << current_state_.robot_mode  << std::endl;

      if(this->hasError() && current_state_.robot_mode == franka::RobotMode::kIdle){
        std::cout << " ERROR SET TO FALSE " << std::endl;
        this->setError(false);
      }
    }
    return !finish_;
  };
  if(!hasError()){
    std::cout << "NORMAL READING " << std::endl;
    control_thread_ = std::make_unique<std::thread>(kReading);
  }
  else{
    std::cout << "ERROR READING " << std::endl;
    control_thread_ = std::make_unique<std::thread>(kErrorReading);
  }
}


bool Robot::isStopped() const {
  return stopped_;
}

//##############################//
// Internal param setters       //
//##############################//

void Robot::setJointStiffness(const franka_msgs::srv::SetJointStiffness::Request::SharedPtr& req) {
  std::lock_guard<std::mutex> lock(write_mutex_);
  std::array<double, 7> joint_stiffness{};
  std::copy(req->joint_stiffness.cbegin(), req->joint_stiffness.cend(), joint_stiffness.begin());
  robot_->setJointImpedance(joint_stiffness);
}

void Robot::setCartesianStiffness(
    const franka_msgs::srv::SetCartesianStiffness::Request::SharedPtr& req) {
  std::lock_guard<std::mutex> lock(write_mutex_);
  std::array<double, 6> cartesian_stiffness{};
  std::copy(req->cartesian_stiffness.cbegin(), req->cartesian_stiffness.cend(),
            cartesian_stiffness.begin());
  robot_->setCartesianImpedance(cartesian_stiffness);
}

void Robot::setLoad(const franka_msgs::srv::SetLoad::Request::SharedPtr& req) {
  std::lock_guard<std::mutex> lock(write_mutex_);
  double mass(req->mass);
  std::array<double, 3> center_of_mass{};  // NOLINT [readability-identifier-naming]
  std::copy(req->center_of_mass.cbegin(), req->center_of_mass.cend(), center_of_mass.begin());
  std::array<double, 9> load_inertia{};
  std::copy(req->load_inertia.cbegin(), req->load_inertia.cend(), load_inertia.begin());

  robot_->setLoad(mass, center_of_mass, load_inertia);
}

void Robot::setTCPFrame(const franka_msgs::srv::SetTCPFrame::Request::SharedPtr& req) {
  std::lock_guard<std::mutex> lock(write_mutex_);

  std::array<double, 16> transformation{};  // NOLINT [readability-identifier-naming]
  std::copy(req->transformation.cbegin(), req->transformation.cend(), transformation.begin());
  robot_->setEE(transformation);
}

void Robot::setStiffnessFrame(const franka_msgs::srv::SetStiffnessFrame::Request::SharedPtr& req) {
  std::lock_guard<std::mutex> lock(write_mutex_);

  std::array<double, 16> transformation{};
  std::copy(req->transformation.cbegin(), req->transformation.cend(), transformation.begin());
  robot_->setK(transformation);
}

void Robot::setForceTorqueCollisionBehavior(
    const franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr& req) {
  std::lock_guard<std::mutex> lock(write_mutex_);

  std::array<double, 7> lower_torque_thresholds_nominal{};
  std::copy(req->lower_torque_thresholds_nominal.cbegin(),
            req->lower_torque_thresholds_nominal.cend(), lower_torque_thresholds_nominal.begin());
  std::array<double, 7> upper_torque_thresholds_nominal{};
  std::copy(req->upper_torque_thresholds_nominal.cbegin(),
            req->upper_torque_thresholds_nominal.cend(), upper_torque_thresholds_nominal.begin());
  std::array<double, 6> lower_force_thresholds_nominal{};
  std::copy(req->lower_force_thresholds_nominal.cbegin(),
            req->lower_force_thresholds_nominal.cend(), lower_force_thresholds_nominal.begin());
  std::array<double, 6> upper_force_thresholds_nominal{};
  std::copy(req->upper_force_thresholds_nominal.cbegin(),
            req->upper_force_thresholds_nominal.cend(), upper_force_thresholds_nominal.begin());

  robot_->setCollisionBehavior(lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
                               lower_force_thresholds_nominal, upper_force_thresholds_nominal);
}

void Robot::setFullCollisionBehavior(
    const franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr& req) {
  std::lock_guard<std::mutex> lock(write_mutex_);

  std::array<double, 7> lower_torque_thresholds_acceleration{};
  std::copy(req->lower_torque_thresholds_acceleration.cbegin(),
            req->lower_torque_thresholds_acceleration.cend(),
            lower_torque_thresholds_acceleration.begin());
  std::array<double, 7> upper_torque_thresholds_acceleration{};
  std::copy(req->upper_torque_thresholds_acceleration.cbegin(),
            req->upper_torque_thresholds_acceleration.cend(),
            upper_torque_thresholds_acceleration.begin());
  std::array<double, 7> lower_torque_thresholds_nominal{};
  std::copy(req->lower_torque_thresholds_nominal.cbegin(),
            req->lower_torque_thresholds_nominal.cend(), lower_torque_thresholds_nominal.begin());
  std::array<double, 7> upper_torque_thresholds_nominal{};
  std::copy(req->upper_torque_thresholds_nominal.cbegin(),
            req->upper_torque_thresholds_nominal.cend(), upper_torque_thresholds_nominal.begin());
  std::array<double, 6> lower_force_thresholds_acceleration{};
  std::copy(req->lower_force_thresholds_acceleration.cbegin(),
            req->lower_force_thresholds_acceleration.cend(),
            lower_force_thresholds_acceleration.begin());
  std::array<double, 6> upper_force_thresholds_acceleration{};
  std::copy(req->upper_force_thresholds_acceleration.cbegin(),
            req->upper_force_thresholds_acceleration.cend(),
            upper_force_thresholds_acceleration.begin());
  std::array<double, 6> lower_force_thresholds_nominal{};
  std::copy(req->lower_force_thresholds_nominal.cbegin(),
            req->lower_force_thresholds_nominal.cend(), lower_force_thresholds_nominal.begin());
  std::array<double, 6> upper_force_thresholds_nominal{};
  std::copy(req->upper_force_thresholds_nominal.cbegin(),
            req->upper_force_thresholds_nominal.cend(), upper_force_thresholds_nominal.begin());
  robot_->setCollisionBehavior(
      lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
      lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
      lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
      lower_force_thresholds_nominal, upper_force_thresholds_nominal);
}

void Robot::setDefaultParams(){
  robot_->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  robot_->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
  robot_->setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
  init_params_set = true;
}

//##############################//
// Internal param setters       //
//##############################//

}  // namespace franka_hardware
