#pragma once

#include <Eigen/Dense>

namespace panda_controllers {

template<int dim>
inline Eigen::Matrix<double, dim, 1> ensureNonNegativeEV(
    const Eigen::MatrixBase<Eigen::Matrix<double, dim, 1>>& in) {
  Eigen::Matrix<double, dim, 1> out = in;
  for (int i=0;i<dim;++i) {
    if (in[i] < 0) {
      out[i] = 0;
    } else {
      break;
    }
  }
  return out;
}

template<class Jacobian, class Inertia,
         class MInertias = Eigen::Matrix<double, 7, 1>,
         class MStiffnesses = Eigen::Matrix<double, 7, 1>>
inline Eigen::Matrix<double, 6, 6> pandaCartesianInertia(
    const Eigen::MatrixBase<Jacobian>& jacobian,
    const Eigen::MatrixBase<Inertia>& inertia,
    const Eigen::MatrixBase<MInertias>& motor_inertias =
        Eigen::Matrix<double, 7, 1>(std::vector<double>(
        {0.6, 0.6, 0.5, 0.5, 0.2, 0.2, 0.2}).data()),
    const Eigen::MatrixBase<MStiffnesses>& motor_stiffnesses =
        Eigen::Matrix<double, 7, 1>(std::vector<double>(
        {1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 2.5}).data())) {
  Eigen::Matrix<double, 7, 7> B, I_K_T_inv;
  B = motor_inertias.asDiagonal();
  I_K_T_inv = (motor_stiffnesses.array()+1).pow(-1).matrix().asDiagonal();
  Eigen::Matrix<double, 7, 7> inertia_hat = inertia + I_K_T_inv * B;
  Eigen::Matrix<double, 6, 6> cartesian_mass_hat_inverse(
      jacobian * inertia_hat.ldlt().solve(jacobian.transpose()));
  return cartesian_mass_hat_inverse.inverse();
}

template<int dim, class Inertia, class Stiffness, class DampingRatio>
inline Eigen::Matrix<double, dim, dim> sqrtDesign(
    const Eigen::MatrixBase<Inertia>& inertia,
    const Eigen::MatrixBase<Stiffness>& stiffness,
    const Eigen::MatrixBase<DampingRatio>& damping_ratio) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, dim, dim>> es;
  es.compute(inertia);
  Eigen::Matrix<double, dim, 1> ev = ensureNonNegativeEV(es.eigenvalues());
  Eigen::Matrix<double, dim, dim> inertia_sqrt = es.eigenvectors() *
      ev.array().sqrt().matrix().asDiagonal() * es.eigenvectors().transpose();
  es.compute(stiffness);
  ev = ensureNonNegativeEV(es.eigenvalues());
  Eigen::Matrix<double, dim, dim> stiffness_sqrt = es.eigenvectors() *
      ev.array().sqrt().matrix().asDiagonal() * es.eigenvectors().transpose();
  return inertia_sqrt * damping_ratio.asDiagonal() * stiffness_sqrt +
         stiffness_sqrt * damping_ratio.asDiagonal() * inertia_sqrt;
}

template<int dim, class Inertia, class Stiffness, class DampingRatio>
inline Eigen::Matrix<double, dim, dim> doubleDiagonalizationDesign(
    const Eigen::MatrixBase<Inertia>& inertia,
    const Eigen::MatrixBase<Stiffness>& stiffness,
    const Eigen::MatrixBase<DampingRatio>& damping_ratio) {
  Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::Matrix<double, dim, dim>> es;
  es.compute(stiffness, inertia);
  Eigen::Matrix<double, dim, 1> KD_0 = ensureNonNegativeEV(es.eigenvalues());
  Eigen::Matrix<double, dim, dim> Q_inv = es.eigenvalues().transpose();
  return 2*Q_inv.ldlt().solve(damping_ratio.asDiagonal()) *
      Q_inv.ldlt().solve(KD_0.array().sqrt().matrix().asDiagonal()).transpose();
}

}
