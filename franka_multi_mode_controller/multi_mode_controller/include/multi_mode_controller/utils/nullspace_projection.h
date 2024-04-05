#pragma once

#include <Eigen/Dense>

namespace panda_controllers {

/*template <int dim>
inline Eigen::Matrix<double, dim, dim> dampedSymmetricInverse(
    const Eigen::Matrix<double, dim, dim>& matrix,
    double threshold = 1e-3) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, dim, dim>> es(matrix);
  Eigen::Matrix<double, dim, 1> eval = es.eigenvalues();
  Eigen::Matrix<double, dim, 1> eval_new;
  for (int i = 0; i < dim; i++) {
    if (eval(i) < threshold) {
      eval(i) = threshold;
    }
    eval_new(i) = 1/eval(i);
  }
  Eigen::Matrix<double, dim, dim> res =
      es.eigenvectors() * eval_new.asDiagonal() * es.eigenvectors().transpose();
  return res;
}*/

template <int dim>
inline Eigen::Matrix<double, dim, dim>
    getDynamicallyConsistentNullspaceProjection(
    const Eigen::Matrix<double, dim, dim>& M_q,
    const Eigen::Matrix<double, 6, dim>& J) {
  Eigen::Matrix<double, dim, 6> M_q_inv_J_T = M_q.ldlt().solve(J.transpose());
  Eigen::Matrix<double, 6, 6> M_x_inv(J*M_q_inv_J_T);
  // option 1: add Identity*0.0... to M_x_inv
  Eigen::Matrix<double, dim, 6> J_consistent_inv =
      M_x_inv.transpose().ldlt().solve(M_q_inv_J_T.transpose()).transpose();
  // option 2: J_consistent_inv = M_q_inv_J_T*dampedSymmetricInverse(M_x_inv);
  return Eigen::Matrix<double, dim, dim>::Identity() -
         J.transpose()*J_consistent_inv.transpose();
}

}
