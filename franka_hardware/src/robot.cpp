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
  }
  catch(franka::ControlException& e){
    RCLCPP_INFO(logger, "Robot is in error state! Please trigger automatic recovery first.");
    setError(true);
    // will need a boolean to set this for the first time;
    // after the necessary runtime services, this part can be removed.
    robot_->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    robot_->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
    robot_->setCollisionBehavior(
          {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
          {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
          {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
          {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
  }
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
  assert(isStopped());
  stopped_ = false;
  const auto kReading = [this]() {
    robot_->read([this](const franka::RobotState& state) {
      {
        std::lock_guard<std::mutex> lock(read_mutex_);
        current_state_ = state;
      }
      return !finish_;
    });
  };
  control_thread_ = std::make_unique<std::thread>(kReading);
}


bool Robot::isStopped() const {
  return stopped_;
}
}  // namespace franka_hardware
