#pragma once

#include <mutex>

#include <multi_mode_controller/base/panda_controller_interface.h>
#include <multi_mode_controller/utils/buffered.h>

namespace panda_controllers {

template <typename Parameters, typename Pose>
class PandaControllerBase : public PandaControllerInterface {
 public:
  virtual ~PandaControllerBase() = default;
  Pose getCurrentPose() {
    std::lock_guard<std::recursive_mutex> lock(current_pose_mutex_);
    return current_;
  }
  Pose getDesiredPose() {
    return desired_;
  }
  Pose getDesiredPoseBuffered() {
    return desired_.get();
  }
  Pose getLastDesiredPose() {
    return last_desired_;
  }
  Pose getOffset() {
    return offset_;
  }
  Parameters getParameters() {
    return p_;
  }
  Parameters getParametersBuffered() {
    return p_.get();
  }
  void setDesiredPoseBuffered(const Pose& p) {
    desired_.setBuffer(p);
  }
  void setError(const bool& error) {
    error_ = error;
  }
  void setOffset(const Pose& p) {
    offset_ = p;
  }
  void setParametersBuffered(const Parameters& p) {
    p_.setBuffer(p);
  }
  void updateCurrentPose() {
    std::lock_guard<std::recursive_mutex> lock(current_pose_mutex_);
    current_ = getCurrentPoseImpl();
  }
 protected:
  virtual bool init_(const std::vector<RobotData*>& robot_data, rclcpp_lifecycle::LifecycleNode::SharedPtr& node, std::string name, std::string resource) override {
    p_.setBuffer(defaultParameters());
    return initImpl(robot_data, node, name, resource);
  }
  void start_() override {
    error_ = false;
    move_timeout_ = 3;
    updateCurrentPose();
    resetOffset();
    has_offset_ = false;
    desired_.setBuffer(current_);
    desired_.update();
    startImpl();
  }
  void stop_() override {
    error_ = false;
    move_timeout_ = 3;
    stopImpl();
  }
 private:
  std::recursive_mutex current_pose_mutex_;
  Buffered<Parameters> p_;
  Buffered<Pose> desired_;
  Pose current_, last_desired_, offset_;
  int move_timeout_;
  bool has_offset_;
  void computeTau_(const std::vector<std::array<double, 7>*>& tau)
      override final {
    updateCurrentPose();
    p_.update();
    ++move_timeout_;
    has_offset_ = hasOffsetImpl();
    if (desired_.updatePending()) {
      last_desired_ = desired_;
      desired_.update();
      move_timeout_ = 0;
    }
    computeTauImpl(tau, desired_, p_);
  }
  virtual void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
      const Pose& desired, const Parameters& p) = 0;
  virtual Parameters defaultParameters() = 0;
  virtual Pose getCurrentPoseImpl() = 0;
  bool hasOffset_() override final {
    return has_offset_;
  }
  virtual bool hasOffsetImpl() {
    return false;
  }
  virtual bool initImpl(const std::vector<RobotData*>& robot_data, rclcpp_lifecycle::LifecycleNode::SharedPtr& node, std::string name, std::string resource) {
    return true;
  }
  virtual bool moving_() override final {
    return move_timeout_ < 3 || has_offset_;
  }
  virtual void resetOffset() {}
  virtual void startImpl() {}
  virtual void stopImpl() {}
};

}
