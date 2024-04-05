#pragma once

#include <Eigen/Dense>
#include <multi_mode_controller/base/panda_controller_base.h>

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

class ComlessPandaCartesianImpedanceController :
    public virtual PandaControllerBase<PandaCartesianImpedanceControllerParams,
                                       PandaCartesianImpedanceControllerPose> {
 public:
  virtual ~ComlessPandaCartesianImpedanceController() = default;
 private:
  void computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
      const PandaCartesianImpedanceControllerPose& desired,
      const PandaCartesianImpedanceControllerParams& p) override final;
  PandaCartesianImpedanceControllerParams defaultParameters() override final;
  PandaCartesianImpedanceControllerPose getCurrentPoseImpl() override final;
  bool hasOffsetImpl() override final;
  void resetOffset() override final;
};

}
