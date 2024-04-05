#pragma once

#include <string>

#include <panda_motion_generators/base/poly_c2_joint_motion_generator_base.h>
#include <panda_motion_generators/base/motion_generator_ros_interface.h>
#include <panda_motion_generator_msgs/action/joint_via_motion.hpp>

namespace panda_motion_generators {

template <typename ControlGoalMsg, int size>
class PolyC2JointMotionGenerator :
    public PolyC2JointMotionGeneratorBase<size>,
    public MotionGeneratorRosInterface<
      Eigen::Matrix<double, size, 1>,
      JointGoal<size>, 
      double, 
      ControlGoalMsg,
      panda_motion_generator_msgs::action::JointViaMotion> {
  using Action   = panda_motion_generator_msgs::action::JointViaMotion;
  using SAResult = panda_motion_generator_msgs::msg::SimpleActionResult;
  using VectorSd = Eigen::Matrix<double, size, 1>;
 public:
  PolyC2JointMotionGenerator() = delete;
    /**
   * Constructor for the PolyC2JointMotionGenerator. 
   * Initializes the PolyC2JointMotionGeneratorBase (MGB) and MotionGeneratorRosInterface (MGRI), 
   * which it inherits from.
   * The min/max args are usually grabbed from multi_mode_controller/utils/panda_limits.h .
   * @param[in] action_name (MGRI) To Name of the action server.
   * @param[in] robot_state_srv_name (MGRI) Fully qualified name of the getRobotState service server, e.g. /panda/get_robot_states
   * @param[in] controller_name (MGRI) Name of the multi-mode controller, e.g. single_multi_mode_controller
   * @param[in] desired_pose_name (MGRI) Fully qualified name of the topic that subscribes to the desired pose. 
   * @param[in] q_min The minimum joint position.
   * @param[in] q_max The maximum joint position.
   * @param[in] qD_max (MGB) Max qdot.
   * @param[in] qDD_max (MGB) Max qddot.
   */
  PolyC2JointMotionGenerator( const std::string& action_name, 
                              const std::string& robot_state_srv_name,
                              const std::string& controller_name,
                              const std::string& desired_pose_name,
                              const VectorSd& q_min, 
                              const VectorSd& q_max, 
                              const VectorSd& qD_max,      
                              const VectorSd& qDD_max) :
      PolyC2JointMotionGeneratorBase<size>(qD_max, qDD_max),
      MotionGeneratorRosInterface<Eigen::Matrix<double, size, 1>,
                                  JointGoal<size>, 
                                  double, 
                                  ControlGoalMsg, 
                                  Action>
                                  ( action_name,
                                    robot_state_srv_name,
                                    controller_name,
                                    desired_pose_name), q_min_(q_min), q_max_(q_max) {};

  virtual ~PolyC2JointMotionGenerator() = default;

 private:
  
  VectorSd q_min_, q_max_;
  virtual bool checkGoal(shared_ptr<const Action::Goal>& goal) override {
    if (goal->v_scale <= 0 || goal->v_scale > 1) {
      this->error_ = "v_scale (%f) out of range, must be inside (0,1].";
      return false;
    }
    for (int i=0;i<goal->via_poses.size();++i) {
      for (int j=0;j<size;++j) {
        if (goal->via_poses[i].q[j] > q_max_[j] ||
            goal->via_poses[i].q[j] < q_min_[j]) {
          this->error_ = "Via Pose " + std::to_string(i) + " violates robot "
              "limits for joint " + std::to_string(j) + ".";
          return false;
        }
      }
    }
    return true;
  }
  virtual shared_ptr<Action::Result> generateAbortedResult() override {
    auto r = std::make_shared<Action::Result>();
    r->result.state = SAResult::ABORTED;
    r->result.error = "";
    return r;
  }
  virtual shared_ptr<Action::Result> generateFailureResult() override {
    auto r = std::make_shared<Action::Result>();
    r->result.state = SAResult::FAILURE;
    r->result.error = this->error_;
    return r;
  }
  virtual shared_ptr<Action::Feedback> generateFeedback(const double& progress,
                                    const double& time_to_completion) override {
    auto f = std::make_shared<Action::Feedback>();
    f->progress = progress;
    f->time_to_completion = time_to_completion;
    return f;
  }
  virtual shared_ptr<Action::Result> generatePreemptedResult() override {
    auto r = std::make_shared<Action::Result>();
    r->result.state = SAResult::PREEMPTED;
    r->result.error = "";
    return r;
  }
  virtual shared_ptr<Action::Result> generateRejectedResult() override {
    auto r = std::make_shared<Action::Result>();
    r->result.state = SAResult::REJECTED;
    r->result.error = this->error_;
    return r;
  }
  virtual shared_ptr<Action::Result> generateSuccessResult() override {
    auto r = std::make_shared<Action::Result>();
    r->result.state = SAResult::SUCCESS;
    r->result.error = "";
    return r;
  }
  virtual double getConfig(shared_ptr<const Action::Goal>& goal) override {
    return goal->v_scale;
  }
  virtual std::vector<VectorSd> getViaPoses(shared_ptr<const Action::Goal>& goal) override {
    std::vector<VectorSd> result;
    for (int i=0;i<goal->via_poses.size();++i) {
      result.push_back((VectorSd() << goal->via_poses[i].q[0],
          goal->via_poses[i].q[1], goal->via_poses[i].q[2],
          goal->via_poses[i].q[3], goal->via_poses[i].q[4],
          goal->via_poses[i].q[5], goal->via_poses[i].q[6]).finished());
    }
    return result;
  }
  virtual void prepareMotion() override {};
  virtual std::string printGoal(shared_ptr<const Action::Goal>& goal) override {
    std::string s("over " + std::to_string(goal->via_poses.size()-1) + " via "
                  "poses to ");
    char buffer[100];
    s += "[";
    for (int i=0;i<goal->via_poses.back().q.size()-1;++i) {
      std::sprintf(buffer, "%5.2f ", goal->via_poses.back().q[i]);
      s += buffer;
    }
    std::sprintf(buffer, "%5.2f]", goal->via_poses.back().q.back());
    s += buffer;
    s += " with v_scale = " + std::to_string(goal->v_scale);
    return s;
  }

};

}
