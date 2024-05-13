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

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/visibility_control.h>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <boost/thread.hpp>
#include "mujoco/mujoco.h"
#include "GLFW/glfw3.h"

#include "franka_hardware/franka_executor.hpp"
#include "franka_hardware/control_mode.h"
#include "franka_hardware/helper_functions.hpp"
#include "franka_hardware/robot_sim.hpp"
#include "franka_hardware/mujoco_visualizer.hpp"
#include "franka_hardware/mujoco_pose_service.hpp"
#include "franka_hardware/gripper_sim_action_server.hpp"
#include "franka_msgs/msg/pose_stamped_array.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_hardware {

struct ObjectContainer{
  std::string obj_name_;
  int obj_body_index_;
  int obj_mocap_index_;
  int obj_jnt_adr_;
  int obj_jnt_num_;
  int obj_jnt_type_;
  std::array<double, 3> obj_position_;
  std::array<double, 4> obj_orientation_;
};

struct ArmContainer {
  std::string robot_name_;
  std::shared_ptr<RobotSim> robot_;
  franka::RobotState hw_franka_robot_state_;

  std::array<double, 7> hw_commands_joint_effort_{0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> hw_commands_joint_position_{0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> hw_commands_joint_velocity_{0, 0, 0, 0, 0, 0, 0};

  // States
  ControlMode control_mode_ = ControlMode::None;
  std::array<double, 7> hw_positions_{0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> hw_velocities_{0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> hw_efforts_{0, 0, 0, 0, 0, 0, 0};
  bool switch_cm_ = false;

};

class FrankaMujocoMultiHardwareInterface : public hardware_interface::SystemInterface {
 public:
  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  static const size_t kNumberOfJoints = 7;
  size_t robot_count_;

 private:
  std::shared_ptr<FrankaExecutor> executor_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  rclcpp::Node::SharedPtr pub_node_;
  
  /* Mujoco variables */
  std::shared_ptr<MujocoVisualizer> mj_visualizer_;
  std::mutex mj_mutex_;
  boost::thread mj_thread_;
  mjModel* m_;
  mjData* d_;
  char error[1000] = {""};  
  mjtNum last_visual_update_;

  std::map<std::string, ArmContainer> arms_;
  std::map<std::string, franka::RobotState*> state_pointers_;
  std::map<std::string, ModelBase*> model_pointers_;
  std::map<std::string, std::shared_ptr<franka_gripper::GripperSimActionServer>> gripper_nodes_;
  std::map<std::string, std::shared_ptr<std::array<double, 3>>> gripper_states_ptrs_; // cmd, width, force
  std::map<std::string, ObjectContainer> mj_objs_;
  rclcpp::Publisher<franka_msgs::msg::PoseStampedArray>::SharedPtr mj_objs_pose_publisher_;
  std::shared_ptr<MujocoPoseServiceServer> mj_pose_service_node_;
  /**
   * Populates the ObjectContainer map mj_objs_ with body index name.
   * No ObjectContainer is created if the object name does not match
   * what is provided in the config file.
  */
  bool populateObjectContainers(std::vector<std::string> obj_names);
  /**
   * Updates all ObjectContainer pose values in the world frame.
  */
  bool updateObjectContainer(std::string obj_name);
  std::array<double, 7> default_arm_qpos_ = {0.0011514965467923919, -0.7849355413286309, 0.0005351744148981226, -2.3558839733056853, -0.00042921391383617395, 1.571927056475186, 0.7850445419811712};
  
  /**
   * Changes the requested object's pose. Calls ...BodyPose or ...MocapPose depending on the object's type.
   * It queries the mj_objs_ map for the given object.
   * It needs to be specified in the mj_objects.yaml file to be updated.
   * Does nothing if the object is not in mj_objs_.
  */
  void changeMjObjPose(std::string obj_name, double x, double y, double z, double qx, double qy, double qz, double qw);
  /**
   * Changes the requested object's pose by modifying its free joint.
   * Does nothing if the requested object does not have a free joint.
  */
  void changeMjBodyPose(std::string obj_name, double x, double y, double z, double qx, double qy, double qz, double qw);
  /**
   * Changes the requested object's pose by modifying the mocap pos and quat.
  */
  void changeMjMocapPose(std::string obj_name, double x, double y, double z, double qx, double qy, double qz, double qw);
  /**
   * Function for publishing the objects in mj_objs_ as a topic.
  */
  void publishObjectContainers();

  rclcpp::Clock clock_;
  static rclcpp::Logger getLogger();

  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};
};
}  // namespace franka_hardware