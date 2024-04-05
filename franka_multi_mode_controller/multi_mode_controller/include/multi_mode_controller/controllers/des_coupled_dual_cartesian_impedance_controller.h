#pragma once

#include <Eigen/Dense>

#include <multi_mode_controller/base/panda_controller_ros_interface.h>
#include <multi_mode_controller/controllers/comless_dual_cartesian_impedance_controller.h>
#include <multi_mode_control_msgs/srv/set_cartesian_impedance.hpp>
#include <multi_mode_control_msgs/msg/cartesian_impedance_goal.hpp>

namespace panda_controllers {
    using GoalMsg = multi_mode_control_msgs::msg::CartesianImpedanceGoal;
    using ServiceParameter = multi_mode_control_msgs::srv::SetCartesianImpedance;
    using Pose = DualCartesianImpedanceControllerPose;
    using Parameters = DualCartesianImpedanceControllerParams;

/**
 * Class with a special desired callback function, which supposedly receives a
 * CartesianImpedanceGoal that defines the midpoint between the two arms' EEs.
 * Then the arms' desired poses will be a relative position to that desired pose,
 * The relative position will be just defined along the y axis, meant to mimic the ball demo
 * from the Justin 2007 video.
 * 
*/
class DesCoupledDualCartesianImpedanceController :
    public virtual ComlessDualCartesianImpedanceController,
    public virtual ControllerRosInterface<ServiceParameter, 
                                          GoalMsg, 
                                          Parameters, 
                                          Pose> {
public:
  virtual ~DesCoupledDualCartesianImpedanceController() = default;

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
  double y_offset = 0.145;
  double arm_distance = 0.52;
};
}
