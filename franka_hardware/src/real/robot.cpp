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

#include <franka_hardware/real/robot.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <mutex>

#include <stdio.h>
#include <iostream>
#include <Eigen/Geometry>
#include <franka/control_tools.h>
#include <rclcpp/logging.hpp>

namespace franka_hardware {

namespace {
// Joint position, joint velocity and Cartesian pose control loops
constexpr double kDeltaT = 1e-3;            // [s] robot control period
constexpr double kPositionGain = 1e-3;      // lambda of eq. 5
constexpr uint64_t kStallTicks = 20;        // ticks without a new command before slowing down
constexpr double kStallDecay = 0.995;       // per-tick decay of the velocity while stalled
constexpr double kBrakeAcceleration = 2.0;  // [rad/s^2], peak of the braking ramp
constexpr double kBrakeLinearAcceleration = 2.0;   // [m/s^2], Cartesian braking ramp
constexpr double kBrakeAngularAcceleration = 4.0;  // [rad/s^2], Cartesian braking ramp
constexpr double kMinBrakeDuration = 0.05;  // [s]
constexpr double kMaxBrakeDuration = 1.0;   // [s]
constexpr double kFinishTimeout = 0.5;      // [s] after the braking ramp, finish anyway

/// Duration of a cosine velocity ramp from v to zero whose peak acceleration is a_max.
double brakeDuration(double v, double a_max) {
  return std::clamp(std::abs(v) * M_PI / (2.0 * a_max), kMinBrakeDuration, kMaxBrakeDuration);
}

/// Factor of the cosine braking ramp, from 1 at the start to 0 after the duration.
double brakeScale(double t, double duration) {
  return t < duration ? 0.5 * (1.0 + std::cos(M_PI * t / duration)) : 0.0;
}

/// Rotation vector (axis * angle) of a rotation matrix.
Eigen::Vector3d rotationLog(const Eigen::Matrix3d& rotation) {
  const Eigen::AngleAxisd angle_axis(rotation);
  return angle_axis.angle() * angle_axis.axis();
}

/// Rotation matrix of a rotation vector (axis * angle).
Eigen::Matrix3d rotationExp(const Eigen::Vector3d& rotation_vector) {
  const double angle = rotation_vector.norm();
  if(angle < 1e-12){
    return Eigen::Matrix3d::Identity();
  }
  return Eigen::AngleAxisd(angle, rotation_vector / angle).toRotationMatrix();
}
}  // namespace

Robot::Robot(const std::string& robot_ip, const rclcpp::Logger& logger) {
  franka::RealtimeConfig rt_config = franka::RealtimeConfig::kEnforce;
  
  // measurement code
  robot_ip_ = robot_ip;
  tau_msmt_.reserve(max_count_);
  // measurement code
  
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
  franka_hardware_model_ = std::make_unique<ModelFranka>(model_.get());
}

Robot::~Robot() {
  stopRobot();
}

void Robot::write(const std::array<double, 7>& efforts, 
                  const std::array<double, 7>& joint_positions, 
                  const std::array<double, 7>& joint_velocities,
                  const std::array<double, 16>& cartesian_positions,
                  const std::array<double, 6>& cartesian_velocities,
                  double period) {
  std::lock_guard<std::mutex> lock(write_mutex_);
  tau_command_ = efforts;
  // The first command after a (re)start is not differenced, and without a valid period the previous
  // velocity is kept, since a velocity dip would reach the robot as a jerk
  if(!joint_position_velocity_valid_){
    joint_position_velocity_.fill(0.0);
  }
  else if(period > 0.0){
    for(size_t i = 0; i < joint_positions.size(); i++){
      joint_position_velocity_[i] = (joint_positions[i] - joint_position_command_[i]) / period;
    }
  }
  joint_position_velocity_valid_ = true;
  ++joint_position_count_;
  joint_position_command_ = joint_positions;
  joint_velocity_command_ = joint_velocities;
  // Same for the Cartesian pose, only while its loop runs since the command is no valid pose otherwise
  if(cartesian_pose_active_){
    if(!cartesian_pose_velocity_valid_){
      cartesian_pose_velocity_.fill(0.0);
    }
    else if(period > 0.0){
      const Eigen::Map<const Eigen::Matrix4d> pose(cartesian_positions.data());
      const Eigen::Map<const Eigen::Matrix4d> previous_pose(cartesian_position_command_.data());
      const Eigen::Vector3d linear =
          (pose.topRightCorner<3, 1>() - previous_pose.topRightCorner<3, 1>()) / period;
      const Eigen::Vector3d angular =
          rotationLog(pose.topLeftCorner<3, 3>() * previous_pose.topLeftCorner<3, 3>().transpose()) /
          period;
      for(size_t i = 0; i < 3; i++){
        cartesian_pose_velocity_[i] = linear[i];
        cartesian_pose_velocity_[i + 3] = angular[i];
      }
    }
    cartesian_pose_velocity_valid_ = true;
    ++cartesian_pose_count_;
  }
  cartesian_position_command_ = cartesian_positions;
  cartesian_velocity_command_ = cartesian_velocities;
}

franka::RobotState Robot::read() {
  std::lock_guard<std::mutex> lock(read_mutex_);
  if(hasError() || isStopped()){ // either the robot is in error, or it doesn't have an active control/read loop running
    try{
      current_state_ = robot_->readOnce();
      if(hasError()){
        stopRobot();
      }
    }
    catch(franka::InvalidOperationException& e){
      std::cout << "Invalid Operation Exception: " << e.what() << std::endl;
    }
  }
  return {current_state_};
}
franka_hardware::ModelFranka* Robot::getModel() {
  return franka_hardware_model_.get();
}

void Robot::stopRobot() {
  if (!stopped_) {
    finish_ = true;
    control_thread_->join();
    robot_->stop();
    finish_ = false;
    brake_requested_ = false;
    {
      std::lock_guard<std::mutex> lock(write_mutex_);
      cartesian_pose_active_ = false;
    }
    stopped_ = true;
    std::cout << "Stopping" << std::endl;
  }
}

// Joint-level controls
void Robot::initializeTorqueControl() {
  assert(isStopped());
  stopped_ = false;
  std::cout << "Initializing joint torque control" << std::endl;
  logged_ = false;
  cycle_count_ = 0;
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
          /* Uncomment for logging */
          // if(cycle_count_ < max_count_){
          //   double time = this->get_wall_time();
          //   this->measureTau(tau_command_, time);
          //   this->increaseCounter();
          // }
          // else{
          //   if(!logged_){
          //     std::cout << "Writing franka file" << std::endl;
          //     this->write_tau_to_file("franka_" + robot_ip_);
          //     logged_ = true;
          //   }
          // }
          /* Uncomment for logging */
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
  {
    // Start from the robot's own setpoint instead of a stale command, until the controller writes one
    std::lock_guard<std::mutex> read_lock(read_mutex_);
    std::lock_guard<std::mutex> write_lock(write_mutex_);
    joint_position_command_ = current_state_.q_d;
    joint_position_velocity_.fill(0.0);
    joint_position_velocity_valid_ = false;
  }
  brake_requested_ = false;
  brakeable_loop_running_ = true;
  // Position-domain reference generation (Langella et al., arXiv:2608.19740, eq. 5): advance the
  // robot's last commanded position q_d by the commanded velocity and pull it only weakly towards
  // the commanded position. A ROS cycle landing before or after a robot tick then no longer
  // reaches the robot as a jerk, since the ROS and robot clocks are independent.
  // Finishing a motion that is not at rest aborts with a discontinuity reflex. Instead, the
  // velocity is ramped smoothly to zero and the motion finished once the robot is at rest.
  const auto kJointPositionControl = [this]() {
    try{

    robot_->control(
      [this, last_count = uint64_t{0}, stalled_ticks = uint64_t{0}, stall_scale = 1.0,
       braking = false, t_brake = 0.0, brake_duration = 0.0, v_brake = std::array<double, 7>{}](
          const franka::RobotState& state, const franka::Duration& period) mutable {
        {
          std::lock_guard<std::mutex> lock(read_mutex_);
          current_state_ = state;
        }
        std::array<double, 7> q_target = state.q_d;
        std::array<double, 7> velocity{};
        double gain = kPositionGain;
        if(!finish_ && !brake_requested_){
          std::lock_guard<std::mutex> lock(write_mutex_);
          if(joint_position_count_ != last_count){
            last_count = joint_position_count_;
            stalled_ticks = 0;
            stall_scale = 1.0;
          }
          else if(++stalled_ticks > kStallTicks){
            stall_scale *= kStallDecay;
          }
          q_target = joint_position_command_;
          for(size_t i = 0; i < velocity.size(); i++){
            velocity[i] = stall_scale * joint_position_velocity_[i];
          }
        }
        else{
          if(!braking){
            braking = true;
            v_brake = state.dq_d;
            for(double v : v_brake){
              brake_duration = std::max(brake_duration, brakeDuration(v, kBrakeAcceleration));
            }
          }
          t_brake += period.toSec();
          const double scale = brakeScale(t_brake, brake_duration);
          for(size_t i = 0; i < velocity.size(); i++){
            velocity[i] = scale * v_brake[i];
          }
          gain = 0.0;
        }
        std::array<double, 7> q_command{};
        bool at_rest = true;
        for(size_t i = 0; i < q_command.size(); i++){
          q_command[i] = state.q_d[i] + kDeltaT * velocity[i] + gain * (q_target[i] - state.q_d[i]);
          at_rest = at_rest && std::abs(state.dq_d[i]) < 1e-4 && std::abs(state.ddq_d[i]) < 1e-2;
        }
        franka::JointPositions out(q_command);
        out.motion_finished = braking && t_brake >= brake_duration &&
                              (at_rest || t_brake >= brake_duration + kFinishTimeout);
        return out;
      },
      // The low-pass filter would scale the per-tick increment and with it the velocity
      franka::ControllerMode::kJointImpedance, true, franka::kMaxCutoffFrequency);
    }
    catch(franka::ControlException& e){
      std::cout <<  e.what() << std::endl;
      setError(true);
    }
    brakeable_loop_running_ = false;
  };
  control_thread_ = std::make_unique<std::thread>(kJointPositionControl);
}

void Robot::requestBrake() {
  brake_requested_ = true;
}

bool Robot::waitForBrake(std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while(brakeable_loop_running_){
    if(std::chrono::steady_clock::now() > deadline){
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return true;
}

void Robot::initializeJointVelocityControl() {
  assert(isStopped());
  stopped_ = false;
  std::cout << "Initializing joint velocity control" << std::endl;
  {
    // Start at rest instead of a stale command, until the controller writes one
    std::lock_guard<std::mutex> lock(write_mutex_);
    joint_velocity_command_.fill(0.0);
  }
  brake_requested_ = false;
  brakeable_loop_running_ = true;
  // Finishing a motion that is not at rest, or stepping the command to zero, aborts with a
  // discontinuity reflex. Instead, the velocity is ramped smoothly to zero before finishing.
  const auto kJointVelocityControl = [this]() {
    try{
      robot_->control(
          [this, braking = false, t_brake = 0.0, brake_duration = 0.0,
           v_brake = std::array<double, 7>{}](
              const franka::RobotState& state, const franka::Duration& period) mutable {
            {
              std::lock_guard<std::mutex> lock(read_mutex_);
              current_state_ = state;
            }
            if(!finish_ && !brake_requested_){
              std::lock_guard<std::mutex> lock(write_mutex_);
              return franka::JointVelocities(joint_velocity_command_);
            }
            if(!braking){
              braking = true;
              v_brake = state.dq_d;
              for(double v : v_brake){
                brake_duration = std::max(brake_duration, brakeDuration(v, kBrakeAcceleration));
              }
            }
            t_brake += period.toSec();
            const double scale = brakeScale(t_brake, brake_duration);
            std::array<double, 7> velocity{};
            bool at_rest = true;
            for(size_t i = 0; i < velocity.size(); i++){
              velocity[i] = scale * v_brake[i];
              at_rest = at_rest && std::abs(state.dq_d[i]) < 1e-4 && std::abs(state.ddq_d[i]) < 1e-2;
            }
            franka::JointVelocities out(velocity);
            out.motion_finished = t_brake >= brake_duration &&
                                  (at_rest || t_brake >= brake_duration + kFinishTimeout);
            return out;
          });
      }
    catch(franka::ControlException& e){
      std::cout <<  e.what() << std::endl;
      setError(true);
    }
    brakeable_loop_running_ = false;
  };
  control_thread_ = std::make_unique<std::thread>(kJointVelocityControl);
}

// Cartesian controls
void Robot::initializeCartesianVelocityControl() {
  assert(isStopped());
  stopped_ = false;
  std::cout << "Initializing cartesian velocity control" << std::endl;
  {
    // Start at rest instead of a stale command, until the controller writes one
    std::lock_guard<std::mutex> lock(write_mutex_);
    cartesian_velocity_command_.fill(0.0);
  }
  brake_requested_ = false;
  brakeable_loop_running_ = true;
  // As for joint velocities: ramp the twist smoothly to zero before finishing, since finishing a
  // motion that is not at rest, or stepping the command to zero, aborts with a discontinuity reflex.
  const auto kCartesianVelocityControl = [this]() {
    try{
      robot_->control(
          [this, braking = false, t_brake = 0.0, brake_duration = 0.0,
           v_brake = std::array<double, 6>{}](
              const franka::RobotState& state, const franka::Duration& period) mutable {
            {
              std::lock_guard<std::mutex> lock(read_mutex_);
              current_state_ = state;
            }
            if(!finish_ && !brake_requested_){
              std::lock_guard<std::mutex> lock(write_mutex_);
              return franka::CartesianVelocities(cartesian_velocity_command_);
            }
            if(!braking){
              braking = true;
              v_brake = state.O_dP_EE_c;
              for(size_t i = 0; i < 3; i++){
                brake_duration = std::max({brake_duration,
                                           brakeDuration(v_brake[i], kBrakeLinearAcceleration),
                                           brakeDuration(v_brake[i + 3], kBrakeAngularAcceleration)});
              }
            }
            t_brake += period.toSec();
            const double scale = brakeScale(t_brake, brake_duration);
            std::array<double, 6> twist{};
            bool at_rest = true;
            for(size_t i = 0; i < twist.size(); i++){
              twist[i] = scale * v_brake[i];
              at_rest = at_rest && std::abs(state.O_dP_EE_c[i]) < 1e-4 &&
                        std::abs(state.O_ddP_EE_c[i]) < 1e-2;
            }
            franka::CartesianVelocities out(twist);
            out.motion_finished = t_brake >= brake_duration &&
                                  (at_rest || t_brake >= brake_duration + kFinishTimeout);
            return out;
          });
      }
    catch(franka::ControlException& e){
      std::cout <<  e.what() << std::endl;
      setError(true);
    }
    brakeable_loop_running_ = false;
  };
  control_thread_ = std::make_unique<std::thread>(kCartesianVelocityControl);
}

void Robot::initializeCartesianPositionControl() {
  assert(isStopped());
  stopped_ = false;
  std::cout << "Initializing cartesian position control" << std::endl;
  {
    // Start from the measured pose instead of a stale command, until the controller writes one
    std::lock_guard<std::mutex> read_lock(read_mutex_);
    std::lock_guard<std::mutex> write_lock(write_mutex_);
    cartesian_position_command_ = current_state_.O_T_EE;
    cartesian_pose_velocity_.fill(0.0);
    cartesian_pose_velocity_valid_ = false;
    cartesian_pose_active_ = true;
  }
  brake_requested_ = false;
  brakeable_loop_running_ = true;
  // Same scheme as the joint position loop on SE(3): advance the robot's last commanded pose
  // O_T_EE_c by the commanded twist, pull it weakly towards the commanded pose, and brake the
  // twist to zero before finishing.
  const auto kCartesianPositionControl = [this]() {
    try{
      robot_->control(
          [this, last_count = uint64_t{0}, stalled_ticks = uint64_t{0}, stall_scale = 1.0,
           braking = false, t_brake = 0.0, brake_duration = 0.0, v_brake = std::array<double, 6>{}](
              const franka::RobotState& state, const franka::Duration& period) mutable {
            {
              std::lock_guard<std::mutex> lock(read_mutex_);
              current_state_ = state;
            }
            std::array<double, 16> pose_target = state.O_T_EE_c;
            std::array<double, 6> twist{};
            double gain = kPositionGain;
            if(!finish_ && !brake_requested_){
              std::lock_guard<std::mutex> lock(write_mutex_);
              if(cartesian_pose_count_ != last_count){
                last_count = cartesian_pose_count_;
                stalled_ticks = 0;
                stall_scale = 1.0;
              }
              else if(++stalled_ticks > kStallTicks){
                stall_scale *= kStallDecay;
              }
              pose_target = cartesian_position_command_;
              for(size_t i = 0; i < twist.size(); i++){
                twist[i] = stall_scale * cartesian_pose_velocity_[i];
              }
            }
            else{
              if(!braking){
                braking = true;
                v_brake = state.O_dP_EE_c;
                for(size_t i = 0; i < 3; i++){
                  brake_duration = std::max({brake_duration,
                                             brakeDuration(v_brake[i], kBrakeLinearAcceleration),
                                             brakeDuration(v_brake[i + 3], kBrakeAngularAcceleration)});
                }
              }
              t_brake += period.toSec();
              const double scale = brakeScale(t_brake, brake_duration);
              for(size_t i = 0; i < twist.size(); i++){
                twist[i] = scale * v_brake[i];
              }
              gain = 0.0;
            }
            const Eigen::Map<const Eigen::Matrix4d> pose_c(state.O_T_EE_c.data());
            const Eigen::Map<const Eigen::Matrix4d> target(pose_target.data());
            const Eigen::Map<const Eigen::Vector3d> linear(twist.data());
            const Eigen::Map<const Eigen::Vector3d> angular(twist.data() + 3);
            const Eigen::Matrix3d rotation_c = pose_c.topLeftCorner<3, 3>();
            Eigen::Matrix4d pose = pose_c;
            pose.topRightCorner<3, 1>() +=
                kDeltaT * linear +
                gain * (target.topRightCorner<3, 1>() - pose_c.topRightCorner<3, 1>());
            const Eigen::Matrix3d rotation =
                rotationExp(kDeltaT * angular +
                            gain * rotationLog(target.topLeftCorner<3, 3>() * rotation_c.transpose())) *
                rotation_c;
            // libfranka rejects poses whose rotation is not orthonormal
            pose.topLeftCorner<3, 3>() = Eigen::Quaterniond(rotation).normalized().toRotationMatrix();
            std::array<double, 16> pose_command{};
            Eigen::Map<Eigen::Matrix4d>(pose_command.data()) = pose;
            bool at_rest = true;
            for(size_t i = 0; i < 6; i++){
              at_rest = at_rest && std::abs(state.O_dP_EE_c[i]) < 1e-4 &&
                        std::abs(state.O_ddP_EE_c[i]) < 1e-2;
            }
            franka::CartesianPose out(pose_command);
            out.motion_finished = braking && t_brake >= brake_duration &&
                                  (at_rest || t_brake >= brake_duration + kFinishTimeout);
            return out;
          },
          // The low-pass filter would scale the per-tick increment and with it the twist
          franka::ControllerMode::kJointImpedance, true, franka::kMaxCutoffFrequency);
      }
    catch(franka::ControlException& e){
      std::cout <<  e.what() << std::endl;
      setError(true);
    }
    brakeable_loop_running_ = false;
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
        if(current_state_.robot_mode == franka::RobotMode::kReflex){
          this->has_error_ = true;
          throw franka::ControlException("Reflex!");
        }
        return !finish_;
      }); // robot_->read()

    }
    catch(franka::ControlException& e){
      std::cout << "Control Exception: " << e.what() << std::endl;
      setError(true);
    }
    catch(franka::InvalidOperationException& e){
      std::cout << "Invalid Operation Exception: " << e.what() << std::endl;
      setError(true);
    }
    
    return;
  };
  control_thread_ = std::make_unique<std::thread>(kReading);
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
