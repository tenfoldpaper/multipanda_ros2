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

#include <string>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Odometry;


namespace garmi_controllers {

class State {
  public:
    double theta_[2] = {0.0, 0.0};
    double velocity_target_[2] = {0.0, 0.0};
    double tau_[2] = {0.0, 0.0};
    std::mutex mux_;

    void set_theta(int index, double theta) {
        mux_.lock();
        theta_[index] = theta;
        mux_.unlock();
    }

    double get_theta(int index) {
        mux_.lock();
        double theta = theta_[index];
        mux_.unlock();
        return theta;
    }

    void set_tau(int index, double tau){
        mux_.lock();
        tau_[index] = tau;
        mux_.unlock();

    }

    double get_tau(int index) {
        mux_.lock();
        double tau = tau_[index];
        mux_.unlock();
        return tau;
    }
    /**
     * Sets velocity of left and right wheel. Right wheel is inversed.
     */
    void set_velocity_target(double velocity_target_1, double velocity_target_2) {
        mux_.lock();
        velocity_target_[0] = velocity_target_1;
        velocity_target_[1] = velocity_target_2;
        mux_.unlock();
    }

    double get_velocity_target(int index) {
        mux_.lock();
        double velocity_target = velocity_target_[index];
        mux_.unlock();
        return velocity_target;
    }
};

class MobileBaseController : public controller_interface::ControllerInterface {
 public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  
  // basic parameters
  std::string robot_id_ = "garmi_base";
  bool is_sim_ = true;
  const int num_joints = 2;

  // base parameters
  double pt1_filter_ = 0.003; 
  double pt1_filter_target_;
  double radius_ = 0.08;
  double separation_ = 0.75;
  double timeout_ = 20.0;
  std::shared_ptr<rclcpp::Subscription<Twist>> goal_twist_sub_;
  std::shared_ptr<rclcpp::Publisher<Odometry>> odom_pub_;

  State base_state_;

  double linear_velocity_;
  double angular_velocity_;
  const double max_speed_ = 2.5;

  rclcpp::Time current_time_, last_time_, last_command_time_;
  rclcpp::Clock clock_;

  double x_current_;
  double y_current_;
  double th_current_;
  
  double x_last_;
  double y_last_;
  double th_last_;
  
  double vx_current_;
  double vy_current_;
  double vth_current_;

  double vx_last_;
  double vy_last_;
  double vth_last_;

  /**
   * Calculates the angular velocity of the wheels from the linear and angular velocity of the base
   */
  double driving_velocity_to_dq(double linear_velocity, double angular_velocity) {
    return (linear_velocity + angular_velocity * separation_ / 2.0) / (radius_);
  }

  void set_velocity_with_xz_twist(double linear_velocity, double angular_velocity){
    double dq_left = driving_velocity_to_dq(linear_velocity, -angular_velocity);
    double dq_right = driving_velocity_to_dq(linear_velocity, angular_velocity);
    const double dq_norm = std::max(std::abs(dq_left),std::abs(dq_right));

    dq_left = dq_norm == 0 ? 0 : dq_left/dq_norm*(std::min(max_speed_, dq_norm));
    dq_right = dq_norm == 0 ? 0 : dq_right/dq_norm*(std::min(max_speed_, dq_norm));
    // if sim, then maybe the -ve on the dq_right is not necessary?
    if(!is_sim_){ // so in the real case, dq_right should be set to its negative.
        dq_right = -dq_right;
    }
    
    // Prioritize move and rotate commands over command listener
    // if (!move_command_ongoing) {
    base_state_.set_velocity_target(dq_left, dq_right);
    last_command_time_ = rclcpp::Clock(RCL_ROS_TIME).now();

    // }
  }

  void command_listener(const Twist twist) {
    set_velocity_with_xz_twist(twist.linear.x, twist.angular.z);
  }

};

}  // namespace franka_example_controllers