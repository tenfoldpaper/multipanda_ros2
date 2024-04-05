#pragma once

#include <cmath>
#include <mutex>
#include <vector>

namespace panda_motion_generators {

template <typename ViaPose, typename ControlGoal, typename Config>
class MotionGeneratorBase {
 public:
  virtual ~MotionGeneratorBase() = default;
    /***
   * Generates the trajectory given a vector of via poses and config values.
   * Simply assigns the via_poses and config to member variables, and calls the overloaded
   * generateTrajectory() which calls the generateTrajectoryImpl(), which should be
   * overridden by the inherited class.
   * @param[in] via_poses A vector of templated via poses to generate the trajectory around.
   * @param[in] config The templated configuration to use in trajectory generation.
   * @return true if trajectory generation was successful, false otherwise.
  */
  bool generateTrajectory(std::vector<ViaPose>&& via_poses,
                          const Config& config) {
    via_poses_ = std::move(via_poses);
    last_config_ = config;
    return generateTrajectory();
  }
  /* Unused functions, for rf framework */
  bool generateRecoveryTrajectory() {
    return generateTrajectory();
  }
  bool isRecovered() {
    return current_section_ > 0;
  }
  void prepareRecovery() {
    if (current_section_ > 0) {
      via_poses_.erase(via_poses_.begin(), via_poses_.begin()+current_section_);
    }
    via_poses_.insert(via_poses_.begin(), current_pose_);
  }
  /* Unused functions, for rf framework */

  /**
   * Sets the member var current_pose_ to the given templated ViaPose.
   * @param[in] pose The desired pose to set as the current pose.
  */
  void setCurrentPose(const ViaPose& pose) {
    current_pose_ = pose;
  }

  /**
   * Scales the given trajectory according to scaling and steps.
   * @param[in] scaling The double value to scale the trajectory by. 1 will not scale anything.
   * @param[in] steps The number of steps to divide scaling increment by. 1 will not scale anything.
  */
  void setTimeScaling(double scaling, int steps) {
    scaling_goal_ = scaling;
    last_scaling_ = scaling_;
    scaling_progress_ = 0;
    scaling_increment_ = 1.0/steps;
  }
  void start() {
    scaling_progress_ = 1;
    scaling_ = scaling_goal_;
  }
  /**
   * Performs the step function until the end time has reached.
   * @param has_offset Unused. Please leave it as false.
   * @param goal The template ControlGoal to publish to the desired pose sub.
   * @param progress The feedback value, ranging from 0 to 1 to indicate progress of trajectory.
   * @return false if offset was detected (which should never happen), otherwise true.
  */
  bool step(const bool& has_offset, ControlGoal& goal, double& progress,
            double& time_to_completion) {
    if (scaling_progress_ < 1) {
      scaling_progress_ += scaling_increment_;
      scaling_ = last_scaling_ +
                 (scaling_goal_-last_scaling_)*scale(scaling_progress_);
    }
    t_ += T*scaling_;
    if (t_ > end_times_[current_section_]) {
      if (has_offset) {
        t_ -= T*scaling_;
        scaling_progress_ -= scaling_increment_;
        return false;
      } else if (current_section_ < via_poses_.size()-1) {
        ++current_section_;
        t_start_ = end_times_[current_section_-1];
      }
    }
    progress = t_/end_times_.back();
    time_to_completion = end_times_.back() - t_;
    goal = stepImpl(current_section_, t_ - t_start_);
    return true;
  }
 protected:
  ViaPose current_pose_;
  std::vector<double> end_times_;
  std::string error_;
  double T = 0.001;
 private:
  std::vector<ViaPose> via_poses_;
  Config last_config_;
  int current_section_;
  double scaling_ = 1;
  double scaling_goal_ = 1;
  double last_scaling_ = 1;
  double scaling_progress_ = 1;
  double scaling_increment_ = 0;
  double t_;
  double t_start_;

  bool generateTrajectory() {
    t_ = 0;
    t_start_ = 0;
    current_section_ = 0;
    end_times_.clear();
    return generateTrajectoryImpl(end_times_, via_poses_, last_config_);
  }

  /**
   * The actual function that will generate the trajectory given via poses and config.
   * Needs to be overridden by the inherited class.
  */
  virtual bool generateTrajectoryImpl(std::vector<double>& end_times,
      const std::vector<ViaPose>& via_poses, const Config& config) = 0;
  double scale(double progress) {
    return 10*std::pow(progress, 3) - 15*std::pow(progress, 4) +
           6*std::pow(progress, 5);
  }
  virtual ControlGoal stepImpl(int section, double t_section) = 0;
};

}
