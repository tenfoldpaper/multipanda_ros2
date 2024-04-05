#pragma once

#include <array>
#include <assert.h>
#include <cmath>

#include <Eigen/Dense>

#include <panda_motion_generators/base/motion_generator_base.h>

namespace panda_motion_generators {

template <int size>
struct JointGoal {
  using VectorSd = Eigen::Matrix<double, size, 1>;
  VectorSd q;
  VectorSd qD;
  VectorSd qDD;
};

template <int size>
class PolyC2JointMotionGeneratorBase :
    public virtual MotionGeneratorBase<Eigen::Matrix<double, size, 1>,
    JointGoal<size>, double> {
 private:
  using VectorSd = Eigen::Matrix<double, size, 1>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;
  using MatrixS6d = Eigen::Matrix<double, size, 6>;
  std::vector<MatrixS6d> motion_coeffs_;
  VectorSd qD_max_, qDD_max_;
  Matrix6d CoeffMatrix(const double& t) {
    return (Matrix6d() <<
        1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 2, 0, 0, 0,
        1, t, std::pow(t,2), std::pow(t,3), std::pow(t,4), std::pow(t,5),
        0, 1, 2*t, 3*std::pow(t,2), 4*std::pow(t,3), 5*std::pow(t,4),
        0, 0, 2, 6*t, 12*std::pow(t,2), 20*std::pow(t,3)).finished();
  }
  virtual bool generateTrajectoryImpl(std::vector<double>& end_times,
      const std::vector<VectorSd>& via_poses, const double& v_scale)
      override final {
    motion_coeffs_.clear();
    if (v_scale <= 0 || v_scale > 1) {
      this->error_ = "v_scale (%f) out of range, must be between 0 and 1.";
      return false;
    }
    VectorSd v_start = VectorSd::Zero();
    end_times.push_back(getTime(via_poses, v_scale, 0));
    for (int i=0;i<via_poses.size();++i) {
      VectorSd v_end = getEndVel(end_times, via_poses, v_scale, i);
      Matrix6d A = CoeffMatrix(end_times[i]);
      MatrixS6d coeffs;
      for (int j=0;j<size;++j) {
        Vector6d b;
        if (i==0) {
          b << this->current_pose_[j], v_start[j], 0, via_poses[i][j], v_end[j],
               0;
        } else {
          b << via_poses[i-1][j], v_start[j], 0, via_poses[i][j], v_end[j], 0;
        }
        coeffs.row(j) = A.colPivHouseholderQr().solve(b).transpose();
      }
      motion_coeffs_.push_back(coeffs);
      v_start = v_end;
    }
    return true;
  }
  VectorSd getEndVel(std::vector<double>& end_times,
      const std::vector<VectorSd>& via_poses, double v_scale, int i) {
    assert(i >= 0 && i<= via_poses.size()-1);
    if (i == via_poses.size()-1) {
      return VectorSd::Zero();
    }
    end_times.push_back(end_times.back() + getTime(via_poses, v_scale, i+1));
    const VectorSd& start = i==0 ? this->current_pose_ : via_poses[i-1];
    const VectorSd& end = via_poses[i];
    const VectorSd& next = via_poses[i+1];
    const double& t_start = i==0 ? 0 : end_times[i-1];
    const double& t_end = end_times[i];
    const double& t_next = end_times[i+1];
    return 0.5*((end-start)/(t_end-t_start) + (next-end)/(t_next-t_end));
  }
  double getTime(const std::vector<VectorSd>& via_poses, double v_scale,
                 int i) {
    assert(i >= 0 && i<= via_poses.size()-1);
    double t = 0.05;
    const VectorSd& start = i<=0 ? this->current_pose_ : via_poses[i-1];
    const VectorSd& end = via_poses[i];
    for (int j=0;j<size;++j) {
      double s = std::abs(end[j]-start[j]);
      double v_max = v_scale*qD_max_[j];
      double a_max = qDD_max_[j];
      t = std::max(t, std::max(15*s/(8*v_max),
                               std::sqrt(10*s/(std::sqrt(3)*a_max))));
    }
    return t;
  }
  virtual JointGoal<size> stepImpl(int section, double t) override final {
    JointGoal<size> g;
    Vector6d t_q = (Vector6d() << 1, t, std::pow(t,2), std::pow(t,3),
                                  std::pow(t,4), std::pow(t,5)).finished();
    Vector6d t_qD = (Vector6d() << 0, 1, 2*t, 3*std::pow(t,2), 4*std::pow(t,3),
                                   5*std::pow(t,4)).finished();
    Vector6d t_qDD = (Vector6d() << 0, 0, 2, 6*t, 12*std::pow(t,2),
                                    20*std::pow(t,3)).finished();
    g.q = motion_coeffs_[section]*t_q;
    g.qD = motion_coeffs_[section]*t_qD;
    g.qDD = motion_coeffs_[section]*t_qDD;
    return g;
  }
 public:
  PolyC2JointMotionGeneratorBase(const VectorSd& qD_max,
                                 const VectorSd& qDD_max) : qD_max_(qD_max),
                                 qDD_max_(qDD_max) {}
  PolyC2JointMotionGeneratorBase() = delete;
  virtual ~PolyC2JointMotionGeneratorBase() = default;
};

}
