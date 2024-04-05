#pragma once

#include <Eigen/Dense>
#include <multi_mode_controller/base/panda_controller_base.h>

namespace panda_controllers {

struct PandaJointImpedanceControllerPose {
  Eigen::Matrix<double, 7, 1> q;
  Eigen::Matrix<double, 7, 1> qD;
};

struct DualJointImpedanceControllerPose {
  DualJointImpedanceControllerPose() : poses(2) {};
  std::vector<PandaJointImpedanceControllerPose> poses;
};

class ComlessDualJointImpedanceController :
    public virtual PandaControllerBase<double,
                                       DualJointImpedanceControllerPose> {
 public:
  virtual ~ComlessDualJointImpedanceController() = default;
 private:
  void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
      const DualJointImpedanceControllerPose& desired,
      const double& p) override final;
  double defaultParameters() override final;
  DualJointImpedanceControllerPose getCurrentPoseImpl() override final;
  bool hasOffsetImpl() override final;
  void resetOffset() override final;
};

}
