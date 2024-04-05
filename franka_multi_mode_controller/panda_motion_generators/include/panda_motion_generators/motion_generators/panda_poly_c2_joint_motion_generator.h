#pragma once

#include <panda_motion_generators/base/poly_c2_joint_motion_generator.h>

#include <multi_mode_control_msgs/msg/joint_goal.hpp>

namespace panda_motion_generators {

class PandaPolyC2JointMotionGenerator :
    public PolyC2JointMotionGenerator<multi_mode_control_msgs::msg::JointGoal, 7> {
 public:
  PandaPolyC2JointMotionGenerator() = delete;
  /**
   * Constructor for the PandaPolyC2JointMotionGenerator.
   * This is the application instantiation of the PolyC2JointMotionGenerator.
   * Initializes the PolyC2JointMotionGenerator which it inherits from.
   * @param[in] action_name Name of the action server.
   * @param[in] robot_state_srv_name Fully qualified name of the getRobotState service server, e.g. /panda/get_robot_states
   * @param[in] controller_name Name of the multi-mode controller, e.g. single_multi_mode_controller
   * @param[in] desired_pose_name Fully qualified name of the topic that subscribes to the desired pose. 
   */
  PandaPolyC2JointMotionGenerator(const std::string& action_name,
                                  const std::string& robot_state_srv_name,
                                  const std::string& controller_name,
                                  const std::string& controllet_name,
                                  const std::string& desired_pose_name);
  ~PandaPolyC2JointMotionGenerator() override final = default;
 private:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using JointGoalMsg = multi_mode_control_msgs::msg::JointGoal;
  std::string controllet_name_;
  virtual JointGoalMsg convertToMsg(const JointGoal<7>& goal) override final;
  virtual bool getCurrentPose(Vector7d& out) override final;
  virtual void prepareMotion() override final;
};

}
