#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>
#include <cmath>

extern "C" {
#include "multi_mode_controller/utils/openGJK.h"
//#include "dubins.c"
}

using namespace Eigen;

// Type Definitions
typedef Matrix<double, 9, 1> Vector9d;
typedef Matrix<double, 7, 1> Vector7d;

namespace redundancy_resolution{
struct ControlOutput {
    VectorXd tau_wb;  // torque control output
    VectorXd v_wb; // velocity control output -- admittance
};

// Function Declarations
Matrix3d crossOperator(Vector3d vec);
Matrix4d DHmatrix(const VectorXd& q, int i);
Matrix4d FK(const VectorXd& Q, int index, bool isRightArm);
VectorXd pose_arm(const Eigen::VectorXd& Q, int index, bool isRightArm);
MatrixXd getJointsPositions(const VectorXd& Q, bool isRightArm);
MatrixXd JacobArm(const VectorXd& Q, int index, bool isRightArm);
MatrixXd JacobBody(const Eigen::VectorXd& Q, int index, bool isRightArm);
MatrixXd JacobianWholeBody(const Eigen::VectorXd& Q, int index, bool isRightArm);
MatrixXd JacobArm_cg(const Eigen::VectorXd& Q, int index, bool isRightArm);
MatrixXd JacobBody_cg(const Eigen::VectorXd& Q, int index, bool isRightArm);
MatrixXd JacobianWholeBody_cg(const Eigen::VectorXd& Q, int index, bool isRightArm);
void computeDynamics(const VectorXd& Q, const VectorXd& Qdot, MatrixXd& M, MatrixXd& C, VectorXd& G);

double calculateDistance(const MatrixXd& link1, const MatrixXd& link2);

VectorXd CollisionAvoidanceDistances(const Eigen::VectorXd& Q, bool isRightArm);
VectorXd ManipulabilityGradient(const Eigen::VectorXd& Q, bool isRightArm);
VectorXd JointLimitPotentialGradient(const Eigen::VectorXd& Q, bool isRightArm);
VectorXd CollisionAvoidanceGradient(const Eigen::VectorXd& Q, const bool isRightArm);

VectorXd refAdmitanceCalc(const VectorXd& wrench_base, const VectorXd& pose_base, const VectorXd& twist_base);
VectorXd poseNonHolonomic(const VectorXd& pose_base, const VectorXd& twist_base);
VectorXd poseErrorWithRotation(const VectorXd& pose_base_ref, const VectorXd& pose_base);
VectorXd baseController(const VectorXd& twist_base_ref, const VectorXd& e_rotated);
ControlOutput wholeBodyController(const Matrix<double, 6, 2>& X_task, const VectorXd& Q_null,  const VectorXd& Q, const VectorXd& Qdot, double currentTime);
// Eigen::Vector2d DiffDriveVelocity(const Eigen::VectorXd& pose_baseurr, const Eigen::VectorXd& p_goal);
// Eigen::Vector3d dubins_instantaneous_velocity(Eigen::Vector3d& base_pose_init, Eigen::Vector3d& base_pose_goal);
} // namespace redundancy_resolution
