#pragma once

#include <panda_motion_generators/base/dq_c1_cartesian_motion_generator.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <multi_mode_control_msgs/msg/cartesian_impedance_goal.hpp>

namespace panda_motion_generators {

class PandaDqC1CartesianMotionGenerator :
    public DqC1CartesianMotionGenerator<
    multi_mode_control_msgs::msg::CartesianImpedanceGoal> {
 public:
  PandaDqC1CartesianMotionGenerator() = delete;
  /**
   * Constructor for the PandaDqC1CartesianMotionGenerator.
   * This is the application instantiation of the PandaDqC1CartesianMotionGenerator.
   * Initializes the DqC1CartesianMotionGenerator which it inherits from.
   * @param[in] action_name Name of the action server.
   * @param[in] robot_state_srv_name Fully qualified name of the getRobotState service server, e.g. /panda/get_robot_states
   * @param[in] controller_name Name of the multi-mode controller, e.g. single_multi_mode_controller
   * @param[in] desired_pose_name Fully qualified name of the topic that subscribes to the desired pose. 
   */
  PandaDqC1CartesianMotionGenerator(const std::string& action_name,
                                    const std::string& robot_state_srv_name,
                                    const std::string& controller_name,
                                    const std::string& controllet_name,
                                    const std::string& desired_pose_name);
  ~PandaDqC1CartesianMotionGenerator() override final = default;
 private:
  using CartesianGoalMsg = multi_mode_control_msgs::msg::CartesianImpedanceGoal;
  std::string controllet_name_;
  virtual CartesianGoalMsg convertToMsg(
      const CartesianGoal& goal) override final;
  virtual bool getCurrentPose(DQ_robotics::DQ& out) override final;
  virtual void prepareMotion() override final;
};

}
