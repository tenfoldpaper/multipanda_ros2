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

#include <multi_mode_controller_impl/single_multi_mode_controller.h>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

using namespace panda_controllers;

namespace multi_mode_controller {

using panda_controllers::PandaControllerInterface;
using SetControllers = multi_mode_control_msgs::srv::SetControllers;
using GetControllers = multi_mode_control_msgs::srv::GetControllers;


//*************** ros2_control overrides *************//
controller_interface::InterfaceConfiguration
SingleMultiModeController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for(auto& arm : arms_){
    
    for (int i = 1; i <= num_joints; ++i) {
      config.names.push_back(arm.arm_id_ + "_joint" + std::to_string(i) + "/effort");
    }
  }
  return config;
}

controller_interface::InterfaceConfiguration
SingleMultiModeController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // so actually, the entire joint position and velocity interface is unnecessary...
  for(auto& arm : arms_){
    // for (int i = 1; i <= num_joints; ++i) {
    //   config.names.push_back(arm.arm_id_ + "_joint" + std::to_string(i) + "/position");
    //   config.names.push_back(arm.arm_id_ + "_joint" + std::to_string(i) + "/velocity");
    // }
    config.names.push_back(arm.arm_id_ + "/robot_state");
    config.names.push_back(arm.arm_id_ + "/robot_model");
  }
  return config;
}

controller_interface::return_type SingleMultiModeController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  updateFrankaState();
  std::shared_lock<std::shared_mutex> lock(control_mtx_);
  for (auto& control : active_control_set_){
    control.ptrs.switcher->computeTau(control.ptrs.tau, control.ptrs.control);
  }

  int k = 0;
  for(auto& arm : arms_){
    for (int i = 0; i < num_joints; i++) {
      command_interfaces_[k].set_value(arm.tau_[i]);
      k++; // BIG assumption: That the command interfaces are always in the same order
    }
  }

  return controller_interface::return_type::OK;
}

CallbackReturn SingleMultiModeController::on_init() {
  // declare all the shit
  // doesn't do jack shit
  // try{
  //   auto_declare<int>("arm_count", 1);
  //   auto_declare<std::vector<std::string>>("controllers", {});
  // } 
  // catch (const std::exception& e) {
  //   fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
  //   return CallbackReturn::ERROR;
  // }

  try {
    rclcpp::Parameter arm_count;
    bool bHas_arm_count = get_node()->get_parameter("arm_count", arm_count);
    //num_robots = get_node()->get_parameter("arm_count").as_int();
    if(!bHas_arm_count){
      fprintf(stderr, "Failed to get arm_count parameter. Make sure it's set in the yaml file.\n");
      return CallbackReturn::ERROR;
    }
    num_robots = arm_count.as_int();

  } catch (const std::exception& e) {
    fprintf(stderr, "Failed to get arm_count parameter. Make sure it's set in the yaml file.\n%s \n", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_node()->get_logger(), "Finished initializing multi joint impedance example controller for %d arms", num_robots);
  return CallbackReturn::SUCCESS;
}

CallbackReturn SingleMultiModeController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
      
  
  // Make the arm container for each of the arms, up to number defined by num_robots
  for(int i = 1; i <= num_robots; i++){
    try{
      std::string arm_id_param_name = "arm_" + std::to_string(i) + ".arm_id";
      rclcpp::Parameter arm_id_param = this->get_node()->get_parameter(arm_id_param_name);
      auto arm_id_name = arm_id_param.as_string();
      // initRobots
      robot_index_.insert(std::make_pair(arm_id_name, i-1));
      arms_.push_back(ArmContainer{arm_id_name, std::array<double, 7>(), 0});
      robot_data_.push_back(RobotData(arm_id_name));

      if(!robot_data_[robot_index_[arm_id_name]].init("/robot_model", "/robot_state")){
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize the robot data for arm %s", arm_id_name.c_str());
        return CallbackReturn::ERROR;  
      };
      // initRobots
      RCLCPP_INFO(get_node()->get_logger(), "Arm set for %s", arm_id_param.as_string().c_str());
    }
    catch(std::exception& e){
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize the arm container for index %d", i);
      return CallbackReturn::ERROR;
    }
  }

  initControllers();
  initServices();

  return CallbackReturn::SUCCESS;
}
//*************** ros2_control overrides *************//

//*************** Custom init functions *************//
bool SingleMultiModeController::initServices(){
  set_ctrl_srv_ = this->get_node()->create_service<SetControllers>("set_controllers", 
                                                  std::bind(&SingleMultiModeController::setControllersCallback, 
                                                            this, 
                                                            std::placeholders::_1, 
                                                            std::placeholders::_2));

  get_ctrl_srv_ = this->get_node()->create_service<GetControllers>("get_controllers",
                                                  std::bind(&SingleMultiModeController::getControllersCallback, 
                                                            this, 
                                                            std::placeholders::_1, 
                                                            std::placeholders::_2));
  return true;
}

bool SingleMultiModeController::initControllers(){
  // Need to figure out how the resource mapping crap works
  // Can I summarize them somehow and put them into a single arm container? 
  // Get rid of some of the map-vector bloat in the header file?

  // TODO: Print out the stuff first and see where that goes
  // Need to get the joint handle stuff sorted out
  std::vector<std::string> controllers;
  // bool switcher = false; 

  // if (!get_node()->get_parameter("switcher").as_string()) { // not using switcher here
  //   ROS_ERROR("MultiPandaMultiModeController: could not get parameter switcher "
  //             "from parameter server.");
  //   return false;
  // }

  // Grab the controllers
  // ["comless_panda_joint_impedance_controller", "comless_panda_cartesian_impedance_controller", "panda_zero_g_controller"]
  controllers = get_string_array_param("controllers");
  if (controllers.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "MultiPandaMultiModeController: could not get controllers from "
              "parameter server.");
    return false;
  }

  for (const auto& controller : controllers) { // for each of comless..., zero_g, ...
    // yaml file:
    // resources: comless...: ["panda_left", "panda_right"]
    //            panda_zero_g...: ["panda_left", "panda_right"]
    std::vector<std::string> resources = get_string_array_param("resources." + controller);;
    // ends up getting ["panda_left", "panda_right"] as a vector
    if (resources.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "MultiPandaMultiModeController: Could not get resources for "
                "controller %s from parameter server.", controller.c_str());
      return false;
    }
    for (const auto& resource : resources) { // for each "panda_left" and "panda_right"
      // This would output
      // MPMMC: Creating controller "comless ..." for resource "panda_left/panda_right" 
      RCLCPP_INFO_STREAM(get_node()->get_logger(), "MultiPandaMultiModeController: Creating controller "
                      << controller << " for resource " << resource << ".");
      control_map_[controller][resource] =
          ControllerFactory::create(controller); // controller-resource pair.
      if (switch_engines_.find(resource) == switch_engines_.end()) {
        // if (switcher) {
        //   switch_engines_[resource] =
        //       std::make_unique<switch_engine::RfEngine>();
        // } else {
        switch_engines_[resource] =
              std::make_unique<switch_engine::SwitchEngine>();
        // }

        // Guessing resourceToVector helps divide cases where 
        // multiple resources are allocated to a single controller, like
        // panda_left & panda_right
        // so panda_left & panda_right -> ["panda_left","panda_right"]
        std::vector<std::string> resource_vector = resourceToVector(resource);
        std::vector<std::array<double, 7>*> tau_vec;
        std::vector<RobotData*> robot_data_vec;
        for (const auto& res : resource_vector) {
          tau_vec.push_back(&arms_[robot_index_[res]].tau_);
          robot_data_vec.push_back(&robot_data_[robot_index_[res]]); // let's see if the pointers work
        }
        // so for such "complex" cases, you would end up with rd_map_["panda_left&panda_right"] = [robot_data(left), robot_data(right)]
        rd_map_[resource] = robot_data_vec;
        tau_map_[resource] = tau_vec;
      }
    }
  }
  /// Try debugging until here

  std::vector<std::string> start_controllers = get_string_array_param("start_controllers.names");
  if (start_controllers.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "MultiPandaMultiModeController: could not get start_controllers "
              "from parameter server.");
    return false;
  }

  std::vector<ControllerInfo> new_control;
  for (const auto& controller : start_controllers) {
    ControllerInfo info;
    std::vector<std::string> resources = get_string_array_param("start_controllers.resources." + controller);
    if (resources.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "MultiPandaMultiModeController: could not get resources for "
                "start_controller %s from parameter server.",
                controller.c_str());
      return false;
    }
    info.name = controller;
    for (const auto& resource : resources) {
      info.resource = std::move(resource);
      new_control.push_back(info);
    }
  }
  std::unique_ptr<std::vector<Controller>> new_control_set =
      getControlSetFromInfo(new_control);
  if (!new_control_set) {
    return false;
  }
  {
    
    std::lock_guard<std::shared_mutex> lock(control_mtx_);
    // Dirty as hell
    auto node_shared = get_node()->shared_from_this();
    for (auto& re : switch_engines_) {
      if (!re.second->init(rd_map_.at(re.first), node_shared, re.first + "_switch_engine",
                           re.first)) {
        RCLCPP_ERROR(get_node()->get_logger(), "MultiPandaMultiModeController: Could not initialize "
                  "switch_engine for resource %s.", re.first.c_str());
        return false;
      }
    }
    active_control_set_ = std::move(*new_control_set.release());
    for (const auto& control : control_map_) {
      for (const auto& resource : control.second) {
        RCLCPP_INFO_STREAM(get_node()->get_logger(),"MultiPandaMultiModeController: Initializing controller"
                        << " " << control.first << " for resource "
                        << resource.first << ".");
        if (!resource.second->init(rd_map_.at(resource.first), node_shared,
                                   control.first, resource.first)) {
          return false;
        }
      }
    }
  }
  printControlSet();
  return true;

}
//*************** Custom init functions *************//

//*************** Utility functions *************//
std::unique_ptr<std::vector<SingleMultiModeController::Controller>> 
  SingleMultiModeController::getControlSetFromInfo(const std::vector<ControllerInfo>& info){
  
  std::vector<SingleMultiModeController::Controller> control_set;
  
  for (const auto& i : info) {
    Controller tmp;
    if (control_map_.find(i.name) == control_map_.end()) {
      RCLCPP_ERROR(get_node()->get_logger(),"MultiPandaMultiModeController: Requested Controller %s is not "
                "loaded.", i.name.c_str());
      return nullptr;
    } else if (control_map_.at(i.name).find(i.resource) !=
               control_map_.at(i.name).end()) {
      tmp.info.name = i.name;
      tmp.info.resource = i.resource;
      tmp.ptrs.tau = tau_map_.at(i.resource);
      tmp.ptrs.control = control_map_.at(i.name).at(i.resource).get();
      tmp.ptrs.switcher = switch_engines_.at(i.resource).get();
      control_set.push_back(tmp);
    } else {
      RCLCPP_ERROR(get_node()->get_logger(),"MultiPandaMultiModeController: Requested Controller %s is not "
                "available for resource %s.", i.name.c_str(),
                i.resource.c_str());
      return nullptr;
    }
  }
  
  return std::make_unique<std::vector<SingleMultiModeController::Controller>>(std::move(control_set));
};

void SingleMultiModeController::printControlSet(){
  std::stringstream ss;
  ss << "MultiPandaMultiModeController: New active control set:";
  for (const auto& controller : active_control_set_) {
    ss << "\n" << controller.info.name << ": " << controller.info.resource;
  }
  RCLCPP_INFO(get_node()->get_logger(),"%s", ss.str().c_str());
};

std::unique_ptr<std::vector<SingleMultiModeController::Controller>> 
  SingleMultiModeController::setupControlSet(const std::vector<ControllerInfo>& info){
    
  std::unique_ptr<std::vector<Controller>> out = getControlSetFromInfo(info);
  if (!out) {
    return nullptr;
  }
  for (const auto& old : active_control_set_) {
    bool add = true;
    for (const auto& nc : *out) {
      if (std::any_of(old.ptrs.tau.begin(), old.ptrs.tau.end(),
          [&](std::array<double, 7>* n) {
            return std::any_of(nc.ptrs.tau.begin(), nc.ptrs.tau.end(),
                               [&](std::array<double, 7>* o) {
              return n==o;
            });
          })) {
        add = false;
        break;
      }
    }
    if (add) {
      out->push_back(old);
    }
  }
  std::vector<int> resource_usage(arms_.size(), 0);
  for (const auto& nc : *out) {
    for (const auto& res : resourceToVector(nc.info.resource)) {
      ++resource_usage[robot_index_[res]];
    }
  }
  for (auto const& arm : arms_) {
    if (resource_usage[robot_index_[arm.arm_id_]] > 1) {
      RCLCPP_ERROR(get_node()->get_logger(),"MultiPandaMultiModeController: specified control has "
                "conflicting resource specification for resource %s.",
                arm.arm_id_.c_str());
      return nullptr;
    } else if (resource_usage[robot_index_[arm.arm_id_]] < 1) {
      RCLCPP_ERROR(get_node()->get_logger(),"MultiPandaMultiModeController: specified control does "
                "not control resource %s.", arm.arm_id_.c_str());
      return nullptr;
    }
  }
  return out;
};


// the const decorator was causing all that ugly errors?? 
bool SingleMultiModeController::setControllersCallback(const SetControllers::Request::SharedPtr req, 
                                                       __attribute_maybe_unused__ const SetControllers::Response::SharedPtr res){
  std::vector<ControllerInfo> new_control;
  for (const auto& controller : req->controllers) {
    ControllerInfo info;
    info.name = controller.name;
    for (const auto& resource : controller.resources) {
      info.resource = std::move(resource);
      new_control.push_back(info);
    }
  }
  std::unique_ptr<std::vector<Controller>> new_control_set =
      setupControlSet(new_control);
  if (!new_control_set) {
    return false;
  }
  {
    std::lock_guard<std::shared_mutex> lock(control_mtx_);
    for (const auto& control : active_control_set_) {
      control.ptrs.switcher->stop(control.ptrs.control);
    }
    active_control_set_ = std::move(*new_control_set.release());
    for (const auto& control : active_control_set_) {
      control.ptrs.switcher->start(control.ptrs.control);
    }
  }
  printControlSet();
  return true;
}

bool SingleMultiModeController::getControllersCallback(__attribute_maybe_unused__ GetControllers::Request::SharedPtr req, 
                                                       const GetControllers::Response::SharedPtr res){
  //Skeleton
  for (const auto& controller : active_control_set_) {
    multi_mode_control_msgs::msg::Controller c;
    c.name = controller.info.name;
    c.resources.push_back(controller.info.resource);
    res->controllers.push_back(c);
  }
  return true;
}

CallbackReturn SingleMultiModeController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  
  for(auto& state_interface : state_interfaces_){
    std::cout << state_interface.get_name() << std::endl;
  }
  for(auto& rd : robot_data_){
    rd.activate_model(state_interfaces_);
  }
  updateFrankaState();
  for (const auto& control : active_control_set_){
    control.ptrs.switcher->start(control.ptrs.control);
  }
  return CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn SingleMultiModeController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  for(auto& rd : robot_data_){
    rd.deactivate_model();
  }
  return CallbackReturn::SUCCESS;
}

void SingleMultiModeController::updateJointStates() {
  // for(auto& arm_container_pair : arms_){ // the code does not rely on getting joint updates from ros control
  //   auto &arm = arm_container_pair.second;
  //   size_t k = 0;
  //   for (size_t i = 0; i < state_interfaces_.size(); i++) {
  //     const auto& position_interface = state_interfaces_.at(2 * i);
  //     const auto& velocity_interface = state_interfaces_.at(2 * i + 1);
  //     if(position_interface.get_prefix_name().find(arm_container_pair.first) == std::string::npos || 
  //        velocity_interface.get_prefix_name().find(arm_container_pair.first) == std::string::npos ){
  //         // if either position or velocity interface does not contain the ID of the arm, skip
  //         continue;
  //     };

  //     assert(position_interface.get_interface_name() == "position");
  //     assert(velocity_interface.get_interface_name() == "velocity");

  //     arm.q_(k) = position_interface.get_value();
  //     arm.dq_(k) = velocity_interface.get_value();

  //     k++;
  //     if(k == 7){
  //       break;
  //     }
  //   }
  // }
}

void SingleMultiModeController::updateFrankaState(){
  // for(auto& rd_ : robot_data_){ // and this is also useless, since getState/getMass is called in the controller itself by the rd pointer.
  //   rd_.getState();
  //   rd_.getMass();
  // }
  for (auto& rd: robot_data_){
    rd.update();
  }
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(multi_mode_controller::SingleMultiModeController,
                       controller_interface::ControllerInterface)