#pragma once

#include <array>
#include <assert.h>
#include <cmath>

#include <dqrobotics/DQ.h>

#include <panda_motion_generators/base/motion_generator_base.h>

namespace panda_motion_generators {

struct CartesianGoal {
  DQ_robotics::DQ pos;
  DQ_robotics::DQ vel;
};

class DqC1CartesianMotionGeneratorBase :
    public virtual MotionGeneratorBase<DQ_robotics::DQ, CartesianGoal, double> {
 private:
  using DQ = DQ_robotics::DQ;
  std::vector<std::array<DQ, 4>> key_frames_;
  double v_max_, omega_max_;
  virtual bool generateTrajectoryImpl(std::vector<double>& end_times,
      const std::vector<DQ>& via_poses, const double& v_scale)
      override final {
    key_frames_.clear();
    if (v_scale <= 0 || v_scale > 1) {
      this->error_ = "v_scale (%f) out of range, must be between 0 and 1.";
      return false;
    }
    DQ v_start(0), v_end(0);
    double t_ges = 0;
    for (int i=0;i<via_poses.size();++i) {
      t_ges += getTime(via_poses, v_scale, i);
      end_times.push_back(t_ges);
      if (i==0) {
        key_frames_.push_back({this->current_pose_, this->current_pose_,
                               via_poses[i], via_poses[i]});
      } else {
        key_frames_.push_back({via_poses[i-1], via_poses[i-1], via_poses[i],
                               via_poses[i]});
      }
    }
    return true;
  }
  double getTime(const std::vector<DQ>& via_poses, double v_scale, int i) {
    assert(i >= 0 && i <= via_poses.size()-1);
    const DQ& start = i<=0 ? this->current_pose_ : via_poses[i-1];
    const DQ& end = via_poses[i];
    DQ q_rel = start.inv()*end;
    double theta = q_rel.rotation_angle();
    DQ s = q_rel.rotation_axis();
    if (s.norm().q(0) < 0.9 || theta==0) {
      s = q_rel.D().Im().normalize();
    }
    double d = dot(q_rel.translation(), s).q(0);
    double r;
    if (theta == 0) {
      r = 0;
    } else {
      r = (q_rel.translation()-s*d).norm().q(0) / (2*std::sin(0.5*theta));
    }
    return std::max(std::max(3*theta/(2*omega_max_*v_scale), 0.05),
                    3*std::sqrt(r*r*theta*theta+d*d)/(2*v_max_*v_scale));
  }
  virtual CartesianGoal stepImpl(int section, double t) override final {
    CartesianGoal g;
    double T = section==0 ? end_times_[0] :
                            end_times_[section] - end_times_[section-1];
    DQ b_1_0 = sclerp(key_frames_[section][0], key_frames_[section][1], t/T);
    DQ b_1_1 = sclerp(key_frames_[section][1], key_frames_[section][2], t/T);
    DQ b_1_2 = sclerp(key_frames_[section][2], key_frames_[section][3], t/T);
    DQ b_2_0 = sclerp(b_1_0, b_1_1, t/T);
    DQ b_2_1 = sclerp(b_1_1, b_1_2, t/T);
    g.pos = sclerp(b_2_0, b_2_1, t/T);
    g.vel = DQ(0);
    return g;
  }
  DQ sclerp(const DQ& q1, const DQ& q2, double t) {
    return q1*dq_exp(t*dq_log(q1.inv()*q2));
  }
  DQ dq_log(const DQ& q) {
    double d, theta_half;
    DQ s, s_hat, theta_hat_half, sin_theta_hat_half;
    if (q.rotation_axis().norm().q(0) > 0.9 && q.rotation_angle() > 0) {
      theta_half = std::acos(q.q(0));
      d = -q.q(4)*2/std::sin(theta_half);
      theta_hat_half = theta_half + DQ::E*d*0.5;
      sin_theta_hat_half = std::sin(theta_half) +
                           DQ::E*d*0.5*std::cos(theta_half);
      s_hat = q.Im()*sin_theta_hat_half.inv();
    } else {
      theta_half = 0;
      d = 2*q.D().Im().norm().q(0);
      if (d==0) {
        s_hat = DQ(0);
      } else {
        s_hat = q.D().Im()*(2/d);
      }
      theta_hat_half = theta_half + DQ::E*d*0.5;
    }
    return theta_hat_half*s_hat;
  }
  DQ dq_exp(const DQ& q) {
    double theta_half, d;
    DQ s, s_0_cross_s;
    theta_half = q.P().Im().norm().q(0);
    if (theta_half > 1e3*DQ_robotics::DQ_threshold) {
      s = q.P().Im()*(1/theta_half);
      d = 2*dot(s, q.D().Im()).q(0);
      s_0_cross_s = (2*q.D().Im() - d*s)*(1/(2*theta_half));
    } else {
      d = 2*q.D().Im().norm().q(0);
      if (d==0) {
        s = DQ(0);
      } else {
        s = q.D().Im()*(2/d);
      }
      s_0_cross_s = DQ(0);
    }
    return std::cos(theta_half) + s*std::sin(theta_half) + DQ::E*(
           -d*0.5*std::sin(theta_half) + s*d*0.5*std::cos(theta_half) +
           s_0_cross_s*std::sin(theta_half));
  }
 public:
  DqC1CartesianMotionGeneratorBase(const double& v_max,
      const double& omega_max) : v_max_(v_max), omega_max_(omega_max) {};
  DqC1CartesianMotionGeneratorBase() = delete;
  virtual ~DqC1CartesianMotionGeneratorBase() = default;
};

}
