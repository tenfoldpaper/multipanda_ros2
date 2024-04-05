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
#include <mutex>
#include <shared_mutex>
#include <utility>
// #include <vector>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

// panda_ros2 includes
#include "franka_msgs/msg/franka_state.hpp"
#include "franka_msgs/msg/errors.hpp"

// multi_mode_controller includes
#include "multi_mode_controller/utils/robot_data.h"
#include "multi_mode_controller/utils/conversions.h"
#include "multi_mode_controller/base/switch_engine.h"
#include "multi_mode_controller/base/panda_controller_interface.h"
#include "multi_mode_control_msgs/srv/get_controllers.hpp"
#include "multi_mode_control_msgs/srv/set_controllers.hpp"
#include "multi_mode_controller/utils/controller_factory.h"


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace multi_mode_controller {   

    using panda_controllers::PandaControllerInterface;
/**
 * The joint impedance example controller moves joint 4 and 5 in a very compliant periodic movement.
 */
class SingleMultiModeController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  
  struct ArmContainer{
    std::string arm_id_;
    std::array<double, 7> tau_;
    int resource_usage_;
  };

  int num_robots;
  const int num_joints = 7;

  // multi_mode_controller classes
  struct ControllerPtrs{
    PandaControllerInterface* control;
    std::vector<std::array<double, 7>*> tau;
    switch_engine::SwitchEngineInterface* switcher;
  };
  struct ControllerInfo {
    std::string name;
    std::string resource;
  };
  struct Controller {
    ControllerPtrs ptrs;
    ControllerInfo info;
  };

  // arm container.
  std::map<std::string, size_t> robot_index_;
  // Cannot use maps for this, as it seems that indexing map with map[key] calls the default constructor ().
  // Need the more annoying way of going through robot_index_.
  // Seems like this is avoidable using map.find[key]... but leave it as it is for now
  std::vector<panda_controllers::RobotData> robot_data_;
  std::vector<ArmContainer> arms_;
  
  std::map<std::string, 
            std::map<std::string, std::unique_ptr<PandaControllerInterface>>>
            control_map_; // a nested map of control_map_[controller][resource] 

  // possibly, a resource container.
  std::map<std::string, std::vector<panda_controllers::RobotData*>> rd_map_; // controller-centric map, of type rd_map_[controller] = ["panda_left", "panda_right"]
  std::map<std::string, std::unique_ptr<switch_engine::SwitchEngineInterface>>
      switch_engines_; // same key as rd_map_
  std::map<std::string, std::vector<std::array<double, 7>*>> tau_map_; // same key as rd_map_


  std::vector<Controller> active_control_set_;
  std::shared_mutex control_mtx_;

  rclcpp::Service<multi_mode_control_msgs::srv::SetControllers>::SharedPtr set_ctrl_srv_;
  rclcpp::Service<multi_mode_control_msgs::srv::GetControllers>::SharedPtr get_ctrl_srv_;

  // loop functions
  void updateJointStates();
  void updateFrankaState();
  
  // custom initializers
  bool initServices(); // initROS
  //initJointHandles is done by the controller itself
  bool initControllers();

  // Utilities
  std::unique_ptr<std::vector<Controller>> getControlSetFromInfo(
      const std::vector<ControllerInfo>& info);
  void printControlSet();

  std::unique_ptr<std::vector<Controller>> setupControlSet(
        const std::vector<ControllerInfo>& info);

  // Service callbacks
  // const in the arguments is always required, otherwise you get a compiler error
  // but it can be a shared ptr or just a reference. for now.
  bool setControllersCallback(const multi_mode_control_msgs::srv::SetControllers::Request::SharedPtr req, 
                              const multi_mode_control_msgs::srv::SetControllers::Response::SharedPtr res);

  bool getControllersCallback(const multi_mode_control_msgs::srv::GetControllers::Request::SharedPtr req, 
                              const multi_mode_control_msgs::srv::GetControllers::Response::SharedPtr res);

  // helper funtion for parameters, because ros2 hates you
  std::vector<std::string> get_string_array_param(std::string param){
    // returns the given parameter as a string array.
    // Returns an empty vector if it does not exist.
    if(get_node()->has_parameter(param)){
      auto out = get_node()->get_parameter(param).as_string_array();
      return out;
    }
    else return std::vector<std::string>();
  }                              

};

}  // namespace franka_example_controllers