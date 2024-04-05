#pragma once

// Instead of inheriting from the panda_controller,
// for RCLCPP the class needs to inherit from the RCLCPP Node class.
// Then this thing could inherit from both of them, 
// or use the composition principle instead and have a member variable for it.
// since this will be a node class, it needs to be put in the main program as a shared pointer.
// Though this also means that the node could be passed on to the other class as another shared pointer...
// In either case, seems like a dedicated node file for the panda controller is needed.

#include <std_msgs/msg/empty.h>
#include <multi_mode_controller/base/panda_controller_base.h>
#include <algorithm> // for std::replace
namespace panda_controllers {


template <typename ServiceParameter,
          typename DesPoseMsg,
          typename Parameters,
          typename Pose>
class ControllerRosInterface : public virtual PandaControllerBase<Parameters, Pose> {

 public:
  virtual ~ControllerRosInterface() = default;
  void startROSCom() {
                    //                  this->
    std::string resource = PandaControllerInterface::resource_;
    std::string amps("&");
    int pos;
    while ((pos = resource.find(amps)) != std::string::npos)
        resource.replace(pos, amps.length(), "_and_");
    des_pose_sub_ = PandaControllerInterface::node_->create_subscription<DesPoseMsg>(
      resource + "/" + 
      PandaControllerInterface::name_ + "/desired_pose", 
      10, std::bind(&ControllerRosInterface::desiredPoseCallback, this, std::placeholders::_1));

    params_server_ = PandaControllerInterface::node_->create_service<ServiceParameter>(
        resource + "/" + 
        PandaControllerInterface::name_ + "/parameters", 
        std::bind(&ControllerRosInterface::setParametersCallback, this, std::placeholders::_1, std::placeholders::_2));    
    startROSComImpl();
  }
  void stopROSCom() {
    // des_pose_sub_.shutdown();
    // params_server_.shutdown();
    stopROSComImpl();
  }
 private:
  
  virtual bool init_(const std::vector<RobotData*>& robot_data, rclcpp_lifecycle::LifecycleNode::SharedPtr& node, std::string name,
      std::string resource) override final {
    return PandaControllerBase<Parameters, Pose>::init_(robot_data, node, name, resource);
  }

  // Subscriber callbacks
  void desiredPoseCallback(const DesPoseMsg& msg) {
    Pose p;
    if (desiredPoseCallbackImpl(p, this->getCurrentPose(), msg)) {
      this->setDesiredPoseBuffered(p);
    } else {
      this->setError(true);
    }
  }

  virtual bool desiredPoseCallbackImpl(Pose& p_d, const Pose& p,
                                       const DesPoseMsg& msg) = 0;

  virtual bool setParametersCallbackImpl(Parameters& p_d, const Parameters& p,
      const std::shared_ptr<typename ServiceParameter::Request>& req, 
      const std::shared_ptr<typename ServiceParameter::Response>& res) = 0;

  // Service callbacks 
  bool setParametersCallback(const typename ServiceParameter::Request::SharedPtr& req, const typename ServiceParameter::Response::SharedPtr& res) {
    Parameters p;
    if (setParametersCallbackImpl(p, this->getParametersBuffered(), req, res)) {
      this->setParametersBuffered(p);
      return true;
    }
    return false;
  }
  
  // Start and stop functions
  virtual void start_() override final {
    PandaControllerBase<Parameters, Pose>::start_();
    startROSCom();
  }
  virtual void stop_() override final {
    stopROSCom();
    PandaControllerBase<Parameters, Pose>::stop_();
  }
  virtual void startROSComImpl() {}
  virtual void stopROSComImpl() {}
  
  typename rclcpp::Subscription<DesPoseMsg>::SharedPtr des_pose_sub_;
  typename rclcpp::Service<ServiceParameter>::SharedPtr params_server_;
  
};

}
