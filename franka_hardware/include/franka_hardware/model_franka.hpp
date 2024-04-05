// Copyright (c) 2023 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <array>
#include "franka_hardware/model_base.hpp"
#include <franka/model.h>

namespace franka_hardware {

/**
 * This class is a thin wrapper around a @ref franka::Model and delegates all calls to
 * that
 */
class ModelFranka : public virtual ModelBase{  // NOLINT(cppcoreguidelines-pro-type-member-init,
               // cppcoreguidelines-special-member-functions)
 public:
  ModelFranka(franka::Model* model) : model_(model) {}
 private:
  franka::Model* model_;
  /**
   * Create a new Model instance wrapped around a franka::Model
   */

  /**
   * Gets the 4x4 pose matrix for the given frame in base frame.
   *
   * The pose is represented as a 4x4 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] q Joint position.
   * @param[in] F_T_EE End effector in flange frame.
   * @param[in] EE_T_K Stiffness frame K in the end effector frame.
   *
   * @return Vectorized 4x4 pose matrix, column-major.
   */
  virtual std::array<double, 16> poseImpl(
      franka::Frame frame,
      const std::array<double, 7>& q,        // NOLINT(readability-identifier-length)
      const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
      const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
      const override final {
    return model_->pose(frame, q, F_T_EE, EE_T_K);
  }

  /**
   * Gets the 6x7 Jacobian for the given frame, relative to that frame.
   *
   * The Jacobian is represented as a 6x7 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] q Joint position.
   * @param[in] F_T_EE End effector in flange frame.
   * @param[in] EE_T_K Stiffness frame K in the end effector frame.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  virtual std::array<double, 42> bodyJacobianImpl(
      franka::Frame frame,
      const std::array<double, 7>& q,        // NOLINT(readability-identifier-length)
      const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
      const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
      const override final {
    return model_->bodyJacobian(frame, q, F_T_EE, EE_T_K);
  }

  /**
   * Gets the 6x7 Jacobian for the given joint relative to the base frame.
   *
   * The Jacobian is represented as a 6x7 matrix in column-major format.
   *
   * @param[in] frame The desired frame.
   * @param[in] robot_state State from which the pose should be calculated.
   *
   * @return Vectorized 6x7 Jacobian, column-major.
   */
  virtual std::array<double, 42> zeroJacobianImpl(
      franka::Frame frame,
      const std::array<double, 7>& q,        // NOLINT(readability-identifier-length)
      const std::array<double, 16>& F_T_EE,  // NOLINT(readability-identifier-naming)
      const std::array<double, 16>& EE_T_K)  // NOLINT(readability-identifier-naming)
      const override final {
    return model_->zeroJacobian(frame, q, F_T_EE, EE_T_K);
  }

  /**
   * Calculates the 7x7 mass matrix. Unit: \f$[kg \times m^2]\f$.
   *
   * @param[in] q Joint position.
   * @param[in] I_total Inertia of the attached total load including end effector, relative to
   * center of mass, given as vectorized 3x3 column-major matrix. Unit: \f$[kg \times m^2]\f$.
   * @param[in] m_total Weight of the attached total load including end effector.
   * Unit: \f$[kg]\f$.
   * @param[in] F_x_Ctotal Translation from flange to center of mass of the attached total load.
   * Unit: \f$[m]\f$.
   *
   * @return Vectorized 7x7 mass matrix, column-major.
   */
  virtual std::array<double, 49> massImpl(
      const std::array<double, 7>& q,        // NOLINT(readability-identifier-length)
      const std::array<double, 9>& I_total,  // NOLINT(readability-identifier-naming)
      double m_total,
      const std::array<double, 3>& F_x_Ctotal)  // NOLINT(readability-identifier-naming)
      const noexcept override final {
    return model_->mass(q, I_total, m_total, F_x_Ctotal);
  }

  /**
   * Calculates the Coriolis force vector (state-space equation): \f$ c= C \times
   * dq\f$, in \f$[Nm]\f$.
   *
   * @param[in] q Joint position.
   * @param[in] dq Joint velocity.
   * @param[in] I_total Inertia of the attached total load including end effector, relative to
   * center of mass, given as vectorized 3x3 column-major matrix. Unit: \f$[kg \times m^2]\f$.
   * @param[in] m_total Weight of the attached total load including end effector.
   * Unit: \f$[kg]\f$.
   * @param[in] F_x_Ctotal Translation from flange to center of mass of the attached total load.
   * Unit: \f$[m]\f$.
   *
   * @return Coriolis force vector.
   */
  virtual std::array<double, 7> coriolisImpl(
      const std::array<double, 7>& q,        // NOLINT(readability-identifier-length)
      const std::array<double, 7>& dq,       // NOLINT(readability-identifier-length)
      const std::array<double, 9>& I_total,  // NOLINT(readability-identifier-naming)
      double m_total,
      const std::array<double, 3>& F_x_Ctotal)  // NOLINT(readability-identifier-naming)
      const noexcept override final {
    return model_->coriolis(q, dq, I_total, m_total, F_x_Ctotal);
  }

  /**
   * Calculates the gravity vector. Unit: \f$[Nm]\f$.
   *
   * @param[in] q Joint position.
   * @param[in] m_total Weight of the attached total load including end effector.
   * Unit: \f$[kg]\f$.
   * @param[in] F_x_Ctotal Translation from flange to center of mass of the attached total load.
   * Unit: \f$[m]\f$.
   * @param[in] gravity_earth Earth's gravity vector. Unit: \f$\frac{m}{s^2}\f$.
   *
   * @return Gravity vector.
   */
  virtual std::array<double, 7> gravityImpl(
      const std::array<double, 7>& q,  // NOLINT(readability-identifier-length)
      double m_total,
      const std::array<double, 3>& F_x_Ctotal,  // NOLINT(readability-identifier-naming)
      const std::array<double, 3>& gravity_earth) const noexcept override final {
    return model_->gravity(q, m_total, F_x_Ctotal, gravity_earth);
  }

//  protected:
//   ModelFranka() = default;

//  private:
//   franka::Model* model_;
};

}  // namespace franka_hardware