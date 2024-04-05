#pragma once

#include <Eigen/Dense>

#include <multi_mode_controller/base/panda_controller_ros_interface.h>
#include <multi_mode_controller/controllers/comless_dual_joint_impedance_controller.h>
#include <multi_mode_control_msgs/srv/set_joint_impedance.hpp>
#include <multi_mode_control_msgs/msg/joint_goal.hpp>


namespace panda_controllers {
    using GoalMsg = multi_mode_control_msgs::msg::JointGoal;
    using ServiceParameter = multi_mode_control_msgs::srv::SetJointImpedance;
    using Pose = DualJointImpedanceControllerPose;
    
class TwinJointImpedanceController :
    public virtual ComlessDualJointImpedanceController,
    public virtual ControllerRosInterface<ServiceParameter,
                                          GoalMsg,
                                          double, 
                                          Pose> {
public:
   virtual ~TwinJointImpedanceController() = default;

private:
  bool desiredPoseCallbackImpl(Pose& p_d,
                               const Pose& p,
                               const GoalMsg& msg) 
                               override final;
  
  bool setParametersCallbackImpl(double& p_d, const double& p,
        const ServiceParameter::Request::SharedPtr& req,
        const ServiceParameter::Response::SharedPtr& res) 
        override final;
};
}
