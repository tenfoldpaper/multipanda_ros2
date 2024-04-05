#pragma once

#include <Eigen/Dense>
#include <multi_mode_controller/base/panda_controller_base.h>

namespace panda_controllers {

struct PandaJointImpedanceControllerPose {
  Eigen::Matrix<double, 7, 1> q;
  Eigen::Matrix<double, 7, 1> qD;
};

class ComlessPandaJointImpedanceController :
    public virtual PandaControllerBase<double,
                                       PandaJointImpedanceControllerPose> {
 public:
  virtual ~ComlessPandaJointImpedanceController() = default;
 private:
  void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
      const PandaJointImpedanceControllerPose& desired,
      const double& p) override final;
  double defaultParameters() override final;
  PandaJointImpedanceControllerPose getCurrentPoseImpl() override final;
  bool hasOffsetImpl() override final;
  void resetOffset() override final;
};

}
