// Copyright (c) 2023 Franka Robotics GmbH
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

#include "franka_semantic_components/franka_robot_model.hpp"

#include <cstring>
#include <iostream>
#include "rclcpp/logging.hpp"
namespace {

// Example implementation of bit_cast: https://en.cppreference.com/w/cpp/numeric/bit_cast
template <class To, class From>
std::enable_if_t<sizeof(To) == sizeof(From) && std::is_trivially_copyable_v<From> &&
                     std::is_trivially_copyable_v<To>,
                 To>
bit_cast(const From& src) noexcept {
  static_assert(std::is_trivially_constructible_v<To>,
                "This implementation additionally requires "
                "destination type to be trivially constructible");

  To dst;
  std::memcpy(&dst, &src, sizeof(To));
  return dst;
}

}  // namespace

namespace franka_semantic_components {
// order of args is correct
FrankaRobotModel::FrankaRobotModel(const std::string& model_name,
                                   const std::string& robot_name)
                                   : SemanticComponentInterface(model_name, 2) 
{
  arm_id_ = robot_name;
  interface_names_.emplace_back(model_name);
  interface_names_.emplace_back(arm_id_ + "/" + robot_state_interface_name_);
  RCLCPP_INFO(rclcpp::get_logger("franka_model_semantic_component"), 
              "Initialized FrankaRobotModel with params %s, %s", arm_id_.c_str(), model_name.c_str());
}


bool FrankaRobotModel::update_state_and_model(){
  auto franka_state_interface =
      std::find_if(state_interfaces_.begin(), state_interfaces_.end(), [&](const auto& interface) {
        return interface.get().get_name() == arm_id_ + "/" + robot_state_interface_name_;
      });

  auto franka_model_interface =
      std::find_if(state_interfaces_.begin(), state_interfaces_.end(), [&](const auto& interface) {
        return interface.get().get_name() == arm_id_ + "/" + robot_model_interface_name_;
      });

  if (franka_state_interface != state_interfaces_.end() &&
      franka_model_interface != state_interfaces_.end()) {
    robot_model = bit_cast<franka_hardware::ModelBase*>((*franka_model_interface).get().get_value());
    robot_state = bit_cast<franka::RobotState*>((*franka_state_interface).get().get_value());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("franka_model_semantic_component"),
                 "Franka interface does not exist! Did you assign the loaned state in the "
                 "controller? %s", arm_id_.c_str());
    return false;
  }
  return true;
};

void FrankaRobotModel::initialize() {
  if(update_state_and_model()){
    initialized = true;
  }
  // add an exception here to throw
}

bool FrankaRobotModel::get_values_as_message(franka_msgs::msg::FrankaModel& message){
  if(!update_state_and_model()){
    return false;
  }
  franka::Frame frame = franka::Frame::kEndEffector;
  message.coriolis = getCoriolisForceVector();
  message.mass = getMassMatrix();
  message.ee_body_jacobian = getBodyJacobian(frame);
  message.ee_zero_jacobian = getZeroJacobian(frame);
  return true;
};
}  // namespace franka_semantic_components