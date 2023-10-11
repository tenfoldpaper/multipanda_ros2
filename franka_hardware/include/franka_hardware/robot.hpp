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

#pragma once

#include <array>
#include <atomic>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <franka/model.h>
#include <franka/robot.h>
#include <franka/exception.h>
#include <franka_hardware/model.hpp>
#include <rclcpp/logger.hpp>

namespace franka_hardware {

class Robot {
 public:
  /**
   * Connects to the robot. This method can block for up to one minute if the robot is not
   * responding. An exception will be thrown if the connection cannot be established.
   *
   * @param[in] robot_ip IP address or hostname of the robot.
   * @param[im] logger ROS Logger to print eventual warnings.
   */
  explicit Robot(const std::string& robot_ip, const rclcpp::Logger& logger);
  Robot(const Robot&) = delete;
  Robot& operator=(const Robot& other) = delete;
  Robot& operator=(Robot&& other) = delete;
  Robot(Robot&& other) = delete;

  /// Stops the currently running loop and closes the connection with the robot.
  virtual ~Robot();

  /**
   * Starts a torque control loop. Before using this method make sure that no other
   * control or reading loop is currently active.
   */
  void initializeTorqueControl();

  void initializeJointPositionControl();
  void initializeJointVelocityControl();

  /**
   * Starts a reading loop of the robot state. Before using this method make sure that no other
   * control or reading loop is currently active.
   */
  void initializeContinuousReading();

  /// stops the control or reading loop of the robot.
  void stopRobot();
  /**
   * Return pointer to the franka robot model object .
   * @return pointer to the current robot model.
   */
  virtual franka_hardware::Model* getModel();
  /**
   * Get the current robot state in a thread-safe way.
   * @return current robot state.
   */
  franka::RobotState read();

  bool hasError(){
    std::lock_guard<std::mutex> lock(error_mutex_);
    return has_error_;
  };

  void setError(bool new_state){
    std::lock_guard<std::mutex> lock(error_mutex_);
    has_error_ = new_state;
  }
  
  void doAutomaticErrorRecovery(){
    robot_->automaticErrorRecovery();
  }


  /**
   * Sends new desired torque commands to the control loop in a thread-safe way.
   * The robot will use these torques until a different set of torques are commanded.
   * @param[in] efforts torque command for each joint.
   */
  void write(const std::array<double, 7>& efforts, const std::array<double, 7>& joint_positions, const std::array<double, 7>& joint_velocities);

  /// @return true if there is no control or reading loop running.
  bool isStopped() const;

 private:
  std::unique_ptr<std::thread> control_thread_;
  std::unique_ptr<franka::Robot> robot_;
  std::unique_ptr<franka::Model> model_;
  std::unique_ptr<Model> franka_hardware_model_;
  std::mutex read_mutex_;
  std::mutex write_mutex_;
  std::atomic_bool finish_{false};
  bool stopped_ = true;
  std::mutex error_mutex_;
  bool has_error_ = false;
  franka::RobotState current_state_;
  std::array<double, 7> tau_command_ = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> joint_position_command_ = {0,-0.785398163397,0,-2.35619449019,0,1.57079632679,0.785398163397};
  std::array<double, 7> joint_velocity_command_;
};
}  // namespace franka_hardware
