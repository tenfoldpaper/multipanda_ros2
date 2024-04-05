#pragma once

#include <string>

#include <panda_motion_generators/base/dq_c1_cartesian_motion_generator_base.h>
#include <panda_motion_generators/base/motion_generator_ros_interface.h>
#include <panda_motion_generator_msgs/action/cartesian_via_motion.hpp>

namespace panda_motion_generators {
/*
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
*/
template <typename CartesianGoalMsg>
class DqC1CartesianMotionGenerator :
    public DqC1CartesianMotionGeneratorBase,
    public MotionGeneratorRosInterface<
      DQ_robotics::DQ, 
      CartesianGoal, 
      double,
      CartesianGoalMsg, 
      panda_motion_generator_msgs::action::CartesianViaMotion> {
 private:
  using Action   = panda_motion_generator_msgs::action::CartesianViaMotion;
  using SAResult = panda_motion_generator_msgs::msg::SimpleActionResult;
  using DQ       = DQ_robotics::DQ;
  virtual bool checkGoal(shared_ptr<const Action::Goal>& goal) override {
    if (goal->v_scale <= 0 || goal->v_scale > 1) {
      this->error_ = "v_scale (%f) out of range, must be inside (0,1].";
      return false;
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
  virtual std::vector<DQ> getViaPoses(shared_ptr<const Action::Goal>& goal) override {
    std::vector<DQ> result;
    for (int i=0;i<goal->via_poses.size();++i) {
      DQ r(goal->via_poses[i].orientation.w,
           goal->via_poses[i].orientation.x,
           goal->via_poses[i].orientation.y,
           goal->via_poses[i].orientation.z);
      if (r.norm().q(0) == 0) {
        r = this->current_pose_.P();
      }
      r.normalize();
      const DQ& last = i==0 ? this->current_pose_.P() : result.back().P();
      if (dot(r.Im(), last.Im()).q(0) < 0) {
        r = -r;
      }
      DQ t(0, goal->via_poses[i].position.x,
           goal->via_poses[i].position.y,
           goal->via_poses[i].position.z);
      result.push_back((r + 0.5*DQ::E*t*r).normalize());
    }
    return result;
  }
  virtual void prepareMotion() override {};
  virtual std::string printGoal(shared_ptr<const Action::Goal>& goal) override {
    std::string s("over " + std::to_string(goal->via_poses.size()-1) + " via "
                  "poses to ");
    char buffer[100];
    s += "[pos: ";
    std::sprintf(buffer, "%5.2f ", goal->via_poses.back().position.x);
    s += buffer;
    std::sprintf(buffer, "%5.2f ", goal->via_poses.back().position.y);
    s += buffer;
    std::sprintf(buffer, "%5.2f", goal->via_poses.back().position.z);
    s += buffer;
    s += ", rot: ";
    std::sprintf(buffer, "%5.2f ", goal->via_poses.back().orientation.w);
    s += buffer;
    std::sprintf(buffer, "%5.2f ", goal->via_poses.back().orientation.x);
    s += buffer;
    std::sprintf(buffer, "%5.2f ", goal->via_poses.back().orientation.y);
    s += buffer;
    std::sprintf(buffer, "%5.2f]", goal->via_poses.back().orientation.z);
    s += buffer;
    s += " with v_scale = " + std::to_string(goal->v_scale);
    return s;
  }
 public:
  DqC1CartesianMotionGenerator() = delete;
  DqC1CartesianMotionGenerator(const std::string& action_name,
                               const std::string& robot_state_srv_name,
                               const std::string& controller_name,
                               const std::string& desired_pose_name,
                               const double& v_max, 
                               const double& omega_max) :
      DqC1CartesianMotionGeneratorBase(v_max, omega_max),
      MotionGeneratorRosInterface<DQ, 
                                  CartesianGoal,
                                  double, 
                                  CartesianGoalMsg,
                                  Action>
                                  (action_name, robot_state_srv_name, controller_name, desired_pose_name) {};
  virtual ~DqC1CartesianMotionGenerator() = default;
};

}
