#pragma once

#include <Eigen/Dense>
#include <multi_mode_controller/base/panda_controller_base.h>
#include <multi_mode_controller/utils/redundancy_resolution.h>

namespace panda_controllers {

struct PandaCartesianImpedanceControllerPose {
  Eigen::Matrix<double, 7, 1> q_n;
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
};

struct PandaCartesianImpedanceControllerParams {
  Eigen::Matrix<double, 6, 6> stiffness;
  Eigen::Matrix<double, 6, 1> damping_ratio;
  double nullspace_stiffness;
};

struct DualCartesianImpedanceControllerPose {
  DualCartesianImpedanceControllerPose() : poses(2) {};
  std::vector<PandaCartesianImpedanceControllerPose> poses;
};

struct DualCartesianImpedanceControllerParams {
  DualCartesianImpedanceControllerParams() : params(2) {};
  std::vector<PandaCartesianImpedanceControllerParams> params;
};  

class ComlessDualCartesianImpedanceController :
    public virtual PandaControllerBase<DualCartesianImpedanceControllerParams,
                                       DualCartesianImpedanceControllerPose> {
 public:
  virtual ~ComlessDualCartesianImpedanceController() = default;
 protected:
  bool bCollisionAvoidance = true;
  bool bManipulability = false;
 private:
  void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
      const DualCartesianImpedanceControllerPose& desired,
      const DualCartesianImpedanceControllerParams& p) override final;
  DualCartesianImpedanceControllerParams defaultParameters() override final;
  DualCartesianImpedanceControllerPose getCurrentPoseImpl() override final;
  bool hasOffsetImpl() override final;
  void resetOffset() override final;
};

}
