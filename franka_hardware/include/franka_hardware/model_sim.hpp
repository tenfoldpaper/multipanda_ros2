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
#include "mujoco/mujoco.h"
#include <iostream> // for debugging
namespace franka_hardware {
  class FrankaMujocoHardwareInterface; // for friending
// Mujoco is row-major format; Eigen is column-major.
// so need to write a converter to change the order of that.
/* ChatGPT impl:
std::vector<int> row_major_to_column_major(const std::vector<int>& arr, int rows, int cols) {
    if (arr.size() != static_cast<size_t>(rows * cols)) {
        throw std::invalid_argument("Size of the array does not match the provided rows and columns.");
    }

    std::vector<int> column_major_arr;
    column_major_arr.reserve(arr.size());

    for (int j = 0; j < cols; ++j) {
        for (int i = 0; i < rows; ++i) {
            column_major_arr.push_back(arr[i * cols + j]);
        }
    }

    return column_major_arr;
}
*/
/**
 * This class is a wrapper around the respective robot's mjModel and mjData.
 * It calculates the function results from the model and data.
 */
class ModelSim : public virtual ModelBase{  // NOLINT(cppcoreguidelines-pro-type-member-init,
               // cppcoreguidelines-special-member-functions)
 public:
  ModelSim(mjModel* &model, mjData* &data) : model_(model), data_(data) {}
  mjModel* getMjModel(){return model_;};
  mjData* getMjData(){return data_;};
  void setIndices(std::array<int, 9UL> link_indices,
                  std::array<int, 9UL> joint_site_indices,
                  std::array<int, 7UL> joint_indices,
                  std::array<int, 7UL> joint_qpos_indices,
                  std::array<int, 7UL> joint_qvel_indices,
                  std::array<int, 7UL> act_trq_indices,
                  std::array<int, 7UL> act_vel_indices){
    link_indices_ = link_indices;
    joint_site_indices_ = joint_site_indices;
    joint_indices_ = joint_indices;
    joint_qpos_indices_ = joint_qpos_indices;
    joint_qvel_indices_ = joint_qvel_indices;
    act_trq_indices_ = act_trq_indices;
    act_vel_indices_ = act_vel_indices;
  };
  /**
   * Composes the force transform matrix from the pos and quat arguments.
   * It does not perform any pre-transforms on those arguments, meaning
   * they should already be inverted if that is required.
  */
  void composeForceTransform(double (&TMatRes)[36], double (&pos)[3], double(&quat)[4]) const {
    /* dm_robotics
    r = ht[0:3, 0:3]
    p = ht[0:3, 3]
    pcross = cross_mat_from_vec3(p)
    tw = np.vstack([np.hstack([r, np.zeros((3, 3))]),
                    np.hstack([pcross.dot(r), r])])
    */
    double rMat[9] = {0};
    mju_quat2Mat(rMat, quat);
    // [0, -z, y],
    // [z, 0, -x],
    // [-y, x, 0]
    double pCross[9] = {0, -pos[2], pos[1],
                        pos[2], 0, -pos[0],
                        -pos[1], pos[0], 0};
    double dpCr[9] = {0};
    mju_mulMatMat(dpCr, pCross, rMat, 3, 3, 3);
    double temp[36] = {rMat[0], rMat[1], rMat[2], 0, 0, 0,
                       rMat[3], rMat[4], rMat[5], 0, 0, 0,
                       rMat[6], rMat[7], rMat[8], 0, 0, 0,
                       dpCr[0], dpCr[1], dpCr[2], rMat[0], rMat[1], rMat[2],
                       dpCr[3], dpCr[4], dpCr[5], rMat[3], rMat[4], rMat[5],
                       dpCr[6], dpCr[7], dpCr[8], rMat[6], rMat[7], rMat[8]};
    for(int i = 0; i<36; i++){
      TMatRes[i] = temp[i];
    }

  }
  /**
   * Returns the pos and quat of the given link.
  */
  void getXPosQuatbyLink(double (&posres)[3], double (&quatres)[4], 
                        const int l) const {
    posres[0] = data_->xpos[3*l];
    posres[1] = data_->xpos[3*l+1];
    posres[2] = data_->xpos[3*l+2];
    quatres[0] = data_->xquat[4*l];
    quatres[1] = data_->xquat[4*l+1];
    quatres[2] = data_->xquat[4*l+2];
    quatres[3] = data_->xquat[4*l+3];
  }

  void getXPosQuatbySite(double (&posres)[3], double (&quatres)[4],
                         const int s) const{
    posres[0] = data_->site_xpos[3*s];
    posres[1] = data_->site_xpos[3*s+1];
    posres[2] = data_->site_xpos[3*s+2];
    double rMat[9] = {0};
    for(int i = 0; i < 9; i++){
      rMat[i] = data_->site_xmat[9*s+i];
    }
    mju_mat2Quat(quatres, rMat);
  }
  /**
   * Returns the pos and quat of B in A's frame.
   * Both needs to be in the same frame, i.e. world frame of mujoco.
  */

  void getBinAframe(double (&posres)[3], double (&quatres)[4], 
                    const int A, const int B) const {
    // int& l7 = link_indices_[7];
    // int& l0 = link_indices_[0];
    // double posres[3] = {0};
    // double quatres[4] = {0}
    double Apos[3] = {0};
    double Apos_I[3] = {0};
    double Bpos[3] = {0};
    double Aquat[4] = {0};
    double Aquat_I[4] = {0};
    double Bquat[4] = {0};
    getXPosQuatbyLink(Apos, Aquat, A);
    mju_negPose(Apos_I, Aquat_I, Apos, Aquat);
    getXPosQuatbyLink(Bpos, Bquat, B);
    
    // multiply: inv(link0_pose) * link7_pose
    mju_mulPose(posres, quatres,
                Apos_I, Aquat_I,
                Bpos, Bquat);
  }
  void getBSiteInALinkframe(double (&posres)[3], double (&quatres)[4], 
                    const int A, const int B) const {
    // int& l7 = link_indices_[7];
    // int& l0 = link_indices_[0];
    // double posres[3] = {0};
    // double quatres[4] = {0}
    double Apos[3] = {0};
    double Apos_I[3] = {0};
    double Bpos[3] = {0};
    double Aquat[4] = {0};
    double Aquat_I[4] = {0};
    double Bquat[4] = {0};
    getXPosQuatbyLink(Apos, Aquat, A);
    mju_negPose(Apos_I, Aquat_I, Apos, Aquat);
    getXPosQuatbySite(Bpos, Bquat, B);
    
    // multiply: inv(link0_pose) * link7_pose
    mju_mulPose(posres, quatres,
                Apos_I, Aquat_I,
                Bpos, Bquat);
  }
  void getStackedRotationMat(double (&rMatStacked)[36], 
                             double (&rMat)[9]) 
                             const{
    // assigning the RMat block at upper left
    int i = 0;
    int j = 0;
    for(i = 0; i < 3; i++){
      for(j = 0; j < 3; j++){
        rMatStacked[i*6 + j] = rMat[i*3 + j];
      }
    }
    // assigning the zeros at upper right
    for(i = 0; i < 3; i++){
      for(j = 3; j < 6; j++){
        rMatStacked[i*6 + j] = 0;
      }
    }
    // assigning the zeros at lower left
    for(i = 3; i < 6; i++){
      for(j = 0; j < 3; j++){
        rMatStacked[i*6 + j] = 0;
      }
    }
    // assigning the RMat block at lower right
    for(i = 3; i < 6; i++){
      for(j = 3; j < 6; j++){
        rMatStacked[i*6 + j] = rMat[(i-3)*3 + (j-3)];
      }
    }
  }
  /**
   * Gets the 6x7 jacobian (r x c) of the robot, based on the site # l.
  */
  void get6x7Jacobian(double (&jacRes)[42], int l) const {
    double *jacp = new double[3*model_->nv];
    double *jacr = new double[3*model_->nv];
    mj_jacSite(model_, data_, jacp, jacr, l);
    // compose the 6*7
    // mujoco is row-major, so 3 rows, nv cols for the jacp
    for(int i = 0; i < 3; i++){
      for(int j = 0; j < 7; j++){
        jacRes[i*7 + j] = jacp[i*model_->nv + joint_qvel_indices_[j]];
      }
    }
    for(int i = 3; i < 6; i++){
      for(int j = 0; j < 7; j++){
        jacRes[i*7 + j] = jacr[(i-3)*model_->nv + joint_qvel_indices_[j]];
      }
    }
    delete jacp;
    delete jacr;
  }

 private:
  mjModel* model_;
  mjData* data_;
  std::array<int, 9UL> link_indices_;
  std::array<int, 9UL> joint_site_indices_;
  std::array<int, 7UL> joint_indices_;
  std::array<int, 7UL> joint_qpos_indices_;
  std::array<int, 7UL> joint_qvel_indices_;
  std::array<int, 7UL> act_trq_indices_;
  std::array<int, 7UL> act_vel_indices_;

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
      const std::array<double, 7>& /*q*/,        // NOLINT(readability-identifier-length)
      const std::array<double, 16>& /*F_T_EE*/,  // NOLINT(readability-identifier-naming)
      const std::array<double, 16>& /*EE_T_K*/)  // NOLINT(readability-identifier-naming)
      const override final {
    // return model_->pose(frame, q, F_T_EE, EE_T_K);
    const int base = link_indices_[0];
    int s = 0;
    
    switch(frame){
      case franka::Frame::kJoint1:
        s = joint_site_indices_[0];
        break;
      case franka::Frame::kJoint2:
        s = joint_site_indices_[1];
        break;
      case franka::Frame::kJoint3:
        s = joint_site_indices_[2];
        break;
      case franka::Frame::kJoint4:
        s = joint_site_indices_[3];
        break;
      case franka::Frame::kJoint5:
        s = joint_site_indices_[4];
        break;
      case franka::Frame::kJoint6:
        s = joint_site_indices_[5];
        break;
      case franka::Frame::kJoint7:
        s = joint_site_indices_[6];
        break;
      case franka::Frame::kFlange:
        s = joint_site_indices_[7];
        break;
      case franka::Frame::kEndEffector:
        s = joint_site_indices_[8];
        break;
      default:
        s = joint_site_indices_[8];
        break;
    }
    std::array<double, 16> result{0};
    double posres[3] = {0};
    double quatres[4] = {0};
    getBSiteInALinkframe(posres, quatres, base, s);
    double rMat[9] = {0};
    mju_quat2Mat(rMat, quatres);
    // mujoco is row-major
    double TMat[16] = {rMat[0], rMat[1], rMat[2], posres[0], 
                       rMat[3], rMat[4], rMat[5], posres[1], 
                       rMat[6], rMat[7], rMat[8], posres[2], 
                             0,       0,       0,         1};
    // convert to col-major
    // convertToColMajor
    int k = 0;
    for (int col = 0; col < 4; ++col) {
      for (int row = 0; row < 4; ++row) {
        result[k] = TMat[row * 4 + col];
        k++;
      }
    }
    return result;
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
      const std::array<double, 7>& /*q*/,        // NOLINT(readability-identifier-length)
      const std::array<double, 16>& /*F_T_EE*/,  // NOLINT(readability-identifier-naming)
      const std::array<double, 16>& /*EE_T_K*/)  // NOLINT(readability-identifier-naming)
      const override final {
    
    int l = 0;
    switch(frame){
      case franka::Frame::kJoint1:
        l = joint_site_indices_[0];
        break;
      case franka::Frame::kJoint2:
        l = joint_site_indices_[1];
        break;
      case franka::Frame::kJoint3:
        l = joint_site_indices_[2];
        break;
      case franka::Frame::kJoint4:
        l = joint_site_indices_[3];
        break;
      case franka::Frame::kJoint5:
        l = joint_site_indices_[4];
        break;
      case franka::Frame::kJoint6:
        l = joint_site_indices_[5];
        break;
      case franka::Frame::kJoint7:
        l = joint_site_indices_[6];
        break;
      case franka::Frame::kFlange:
        l = joint_site_indices_[7];
        break;
      case franka::Frame::kEndEffector:
        l = joint_site_indices_[8];
        break;
      default:
        l = joint_site_indices_[8];
        break;
    }
    std::array<double, 42> result{0};
    double posres[3] = {0};
    double posres_I[3] = {0};
    double quatres[4] = {0};
    double quatres_I[4] = {0};
    // First get the joint transform and invert it
    getXPosQuatbyLink(posres, quatres, l);
    mju_negPose(posres_I, quatres_I, posres, quatres);

    // get the force transform matrix
    double rMatStacked[36] = {0};
    composeForceTransform(rMatStacked, posres_I, quatres_I);
    
    double Jac[42] = {0};
    double rJac[42] = {0};
    // Calculate the body jacobian
    get6x7Jacobian(Jac, l);
    // Multiply by the force transform mat to change the frame
    mju_mulMatMat(rJac, rMatStacked, Jac, 6, 6, 7);

    // convert to col major format
    // convertToColMajor
    int k = 0;
    for (int col = 0; col < 7; ++col) {
      for (int row = 0; row < 6; ++row) {
        result[k] = rJac[row * 7 + col];
        k++;
      }
    }
    return result;
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
      const std::array<double, 7>& /*q*/,        // NOLINT(readability-identifier-length)
      const std::array<double, 16>& /*F_T_EE*/,  // NOLINT(readability-identifier-naming)
      const std::array<double, 16>& /*EE_T_K*/)  // NOLINT(readability-identifier-naming)
      const override final {
    
    int l = 0;
    switch(frame){
      case franka::Frame::kJoint1:
        l = joint_site_indices_[0];
        break;
      case franka::Frame::kJoint2:
        l = joint_site_indices_[1];
        break;
      case franka::Frame::kJoint3:
        l = joint_site_indices_[2];
        break;
      case franka::Frame::kJoint4:
        l = joint_site_indices_[3];
        break;
      case franka::Frame::kJoint5:
        l = joint_site_indices_[4];
        break;
      case franka::Frame::kJoint6:
        l = joint_site_indices_[5];
        break;
      case franka::Frame::kJoint7:
        l = joint_site_indices_[6];
        break;
      case franka::Frame::kFlange:
        l = joint_site_indices_[7];
        break;
      case franka::Frame::kEndEffector:
        l = joint_site_indices_[8];
        break;
      default:
        l = joint_site_indices_[8];
        break;
    }
    std::array<double, 42> result{0};
    double posres[3] = {0};
    double posres_I[3] = {0};
    double quatres[4] = {0};
    double quatres_I[4] = {0};
    // First get the base transform and invert it
    getXPosQuatbyLink(posres, quatres, link_indices_[0]);
    mju_negPose(posres_I, quatres_I, posres, quatres);

    // get the force transform matrix
    double rMatStacked[36] = {0};
    composeForceTransform(rMatStacked, posres_I, quatres_I);

    double Jac[42] = {0};
    double rJac[42] = {0};
    // Calculate the body jacobian
    get6x7Jacobian(Jac, l);
    // Multiply by the force transform mat to change the frame
    mju_mulMatMat(rJac, rMatStacked, Jac, 6, 6, 7);

    // convert to col major format
    // convertToColMajor
    int k = 0;
    for (int col = 0; col < 7; ++col) {
      for (int row = 0; row < 6; ++row) {
        result[k] = rJac[row * 7 + col];
        k++;
      }
    }
    return result;
  }

  /**
   * Returns the 7x7 mass matrix. Unit: \f$[kg \times m^2]\f$.
   * The arguments are not necessary (should be dummies) for sim version,
   * since all the data is contained in mjData and mjModel.
   * Iterates through all the links of the given robot as defined in link_indices_
   * to construct the 7x matrix from the dense inertia matrix of mujoco.
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
      const std::array<double, 7>& /*q*/,        // NOLINT(readability-identifier-length)
      const std::array<double, 9>& /*I_total*/,  // NOLINT(readability-identifier-naming)
      double /*m_total*/,
      const std::array<double, 3>& /*F_x_Ctotal*/)  // NOLINT(readability-identifier-naming)
      const noexcept override final {
    // Mass matrix is symmetric, so it shouldn't matter. 
    std::array<double, 49> result{0};
    double *M = new double[model_->nv * model_->nv];
    mj_fullM(model_, M, data_->qM);
    int m_i = 0;
    for(int i=0; i<7; i++){
      for(int j=0; j<7; j++){
        result[m_i] = M[joint_qvel_indices_[i] * model_->nv + joint_qvel_indices_[j]];
        m_i++;
      }
    }
    delete M;
    return result;
  }

  /**
   * Simply returns the qfrc_bias - qfrc_gravcomp vector for now. 
   * This (according to document) should represent the Coriolis + centrifugal terms. 
   * Unit: \f$ c= C \times
   * The arguments are not necessary (should be dummies) for sim version,
   * since all the data is contained in mjData and mjModel.
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
      const std::array<double, 7>& /*q*/,        // NOLINT(readability-identifier-length)
      const std::array<double, 7>& /*dq*/,       // NOLINT(readability-identifier-length)
      const std::array<double, 9>& /*I_total*/,  // NOLINT(readability-identifier-naming)
      double /*m_total*/,
      const std::array<double, 3>& /*F_x_Ctotal*/)  // NOLINT(readability-identifier-naming)
      const noexcept override final {
    std::array<double, 7> result{0};
    for(int i = 0; i < 7; i++){
      result[i] = data_->qfrc_bias[joint_qvel_indices_[i]] - 
                  data_->qfrc_gravcomp[joint_qvel_indices_[i]];
    }
    return result;
  }

  /**
   * Simply returns the qfrc_gravcomp vector for now. Unit: \f$[Nm]\f$.
   * The arguments are not necessary (should be dummies) for sim version,
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
      const std::array<double, 7>& /*q*/,  // NOLINT(readability-identifier-length)
      double /*m_total*/,
      const std::array<double, 3>& /*F_x_Ctotal*/,  // NOLINT(readability-identifier-naming)
      const std::array<double, 3>& /*gravity_earth*/) const noexcept override final {
    std::array<double, 7> result{0};
    for(int i = 0; i < 7; i++){
      result[i] = data_->qfrc_gravcomp[joint_qvel_indices_[i]];
    }
    return result;
  }

  friend class franka_hardware::FrankaMujocoHardwareInterface;

//  protected:
//   ModelFranka() = default;

//  private:
//   franka::Model* model_;
};

}  // namespace franka_hardware