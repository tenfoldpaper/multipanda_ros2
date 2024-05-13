#pragma once

#include <Eigen/Dense>

#include <multi_mode_controller/base/panda_controller_ros_interface.h>
#include <multi_mode_controller/controllers/comless_dual_cartesian_impedance_controller.h>
#include <multi_mode_control_msgs/srv/set_cartesian_impedance.hpp>
#include <multi_mode_control_msgs/msg/dual_cartesian_impedance_goal.hpp>

namespace panda_controllers {
    using GoalMsg = multi_mode_control_msgs::msg::DualCartesianImpedanceGoal;
    using ServiceParameter = multi_mode_control_msgs::srv::SetCartesianImpedance;
    using Pose = DualCartesianImpedanceControllerPose;
    using Parameters = DualCartesianImpedanceControllerParams;

class DualCartesianImpedanceController :
    public virtual ComlessDualCartesianImpedanceController,
    public virtual ControllerRosInterface<ServiceParameter, 
                                          GoalMsg, 
                                          Parameters, 
                                          Pose> {
public:
  virtual ~DualCartesianImpedanceController() = default;

private:
  bool desiredPoseCallbackImpl(Pose& p_d,
                               const Pose& p,
                               const GoalMsg& msg)
                               override final;

  bool setParametersCallbackImpl(Parameters& p_d, 
        const Parameters& p,
        const ServiceParameter::Request::SharedPtr& req, 
        const ServiceParameter::Response::SharedPtr& res) 
        override final;

};
}
