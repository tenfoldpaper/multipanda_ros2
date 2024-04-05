#include <Eigen/Dense>
#include <iostream>
#include <cmath>

extern "C" {
#include "multi_mode_controller/utils/openGJK.h"
// #include "dubins.c"
}

#include "multi_mode_controller/utils/redundancy_resolution.h"

using namespace Eigen;
typedef Matrix<double, 9, 1> Vector9d;
typedef Matrix<double, 7, 1> Vector7d;

namespace redundancy_resolution{
// Cross Product Operator
Matrix3d crossOperator(Vector3d vec) {
    Matrix3d crossMatrix;
    crossMatrix << 0.0, -vec.z(), vec.y(),
                   vec.z(), 0.0, -vec.x(),
                   -vec.y(), vec.x(), 0.0;
    return crossMatrix;
}

// Denavit-Hartenberg Matrix
Matrix4d DHmatrix(const VectorXd& q, int i) {
    double l = 0.1; // Tool length
    VectorXd d(8); d << 0.333, 0, 0.316, 0, 0.384, 0, 0, 0.107 + l;
    VectorXd a(8); a << 0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0;
    VectorXd alpha(8); alpha << 0, -M_PI/2, M_PI/2, M_PI/2, -M_PI/2, M_PI/2, M_PI/2, 0;
    VectorXd q_ext(8);
    q_ext << q, 0;

    double ct = cos(q_ext(i));
    double st = sin(q_ext(i));
    double ca = cos(alpha(i));
    double sa = sin(alpha(i));

    Matrix4d T;
    T << ct, -st, 0, a(i),
         st * ca, ct * ca, -sa, -sa * d(i),
         st * sa, ct * sa, ca, ca * d(i),
         0, 0, 0, 1;
    return T;
}

Matrix4d FK(const VectorXd& Q, int index, bool isRightArm) {  // forward kinematics
    double x = Q(0);
    double y = Q(1);
    double theta = Q(2);
    VectorXd q_arm = (isRightArm) ? Q.segment(3, 7) : Q.segment(10, 7);

    // Parameters Definition
    double width = 0.75; // base plotting
    double height = 0; //0.2; // base plotting
    double depth = 0.75; // base plotting
    double wheelRadius = height / 2; 
    double spineHeight = 0; // 1.05;
    double spineOffset = 0; // 0.1;
    double shoulderWidth = 0.52; // full y-offset of the arms -- place arms 52cm away from each other

    double beta = 0; // M_PI / 2.0;
    double gamma = 0; // isRightArm ? M_PI / 2.0 : -M_PI / 2.0;
    Vector3d shoulderPose = isRightArm ? Vector3d(spineOffset, -shoulderWidth / 2, height + spineHeight) : Vector3d(spineOffset, shoulderWidth / 2, height + spineHeight);

    // Transformation matrix for the base of the robot
    Matrix4d T_mobile_base;
    T_mobile_base << cos(theta), -sin(theta), 0, x,
                     sin(theta), cos(theta), 0, y,
                     0, 0, 1, 0,
                     0, 0, 0, 1;

    // Transformation matrix for the shoulder pose
    Matrix4d Tshoulder;
    Tshoulder << 1, 0, 0, shoulderPose(0),
                 0, 1, 0, shoulderPose(1),
                 0, 0, 1, shoulderPose(2),
                 0, 0, 0, 1;

    // Transformation matrix for the robot's wall-mount arm installation
    Matrix4d Ty, Tz;
    Ty << cos(beta), 0, sin(beta), 0,
          0, 1, 0, 0,
          -sin(beta), 0, cos(beta), 0,
          0, 0, 0, 1;

    Tz << cos(gamma), -sin(gamma), 0, 0,
          sin(gamma), cos(gamma), 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;

    // Transformation of the arm_base to the universal frame
    Matrix4d Ti = T_mobile_base * Tshoulder * Ty * Tz;

    for (int joint_id = 0; joint_id <= index; ++joint_id) {
        Ti = Ti * DHmatrix(q_arm, joint_id); // Apply rotation matrix to compensate for arm configuration
    }

    return Ti;
}

VectorXd pose_arm(const VectorXd& Q, int index, bool isRightArm) {
    VectorXd X(6);
    X.setZero();

    // Compute the transformation matrix for the specified configuration
    MatrixXd T_idx = FK(Q, index, isRightArm);

    // Extract the rotation matrix from the transformation matrix
    Matrix3d R = T_idx.block<3, 3>(0, 0);

    // Convert the rotation matrix to an axis-angle representation
    double angle = std::acos((R.trace() - 1) / 2);
    Vector3d axis;
    if (angle != 0) {
        axis << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
        axis /= axis.norm();
    } else {
        axis << 0, 0, 1; // Default axis if no rotation is present
    }

    // Return pose vector
    X.segment(0, 3) = T_idx.block<3, 1>(0, 3);
    X.segment(3, 3) = angle * axis;
    return X;
}

MatrixXd getJointsPositions(const VectorXd& Q, bool isRightArm) {
    MatrixXd positions(3, 8); // Matrix to hold the positions of up to 8 joints

    for (int joint_id = 0; joint_id <=7 ; ++joint_id) {
        Matrix4d T = FK(Q, joint_id, isRightArm); // Use FK to find the transformation matrix
        positions.col(joint_id) = T.block<3, 1>(0, 3); // Extract the translation part (position) and store it in the matrix
    }
    return positions;
}

MatrixXd JacobArm(const VectorXd& Q, int index, bool isRightArm) {
    // Libfranka has a similar function.
    Matrix<double, 6, 7> jacobian; // Explicitly define Jacobian matrix size
    jacobian.setZero();

    // Compute FK up to the specified joint index
    Matrix4d T_joint_index = FK(Q, index, isRightArm);
    Vector3d P_joint_index = T_joint_index.block<3, 1>(0, 3); // Extract the position vector of the joint at the specified index

    for (int joint_id = 0; joint_id < index; ++joint_id) { // Count joint positions up to one before the target joint
        // Compute the transformation matrix up to the current joint
        Matrix4d Ti = FK(Q, joint_id, isRightArm);

        // Extract the joint axis of the current joint
        Vector3d P_i = Ti.block<3, 1>(0, 3);

        // Calculate the columns of the Jacobian matrix
        Vector3d z_current = Ti.block<3, 1>(0, 2); // z-axis
        Vector3d p_diff = P_joint_index - P_i;

        // Update the Jacobian matrix
        // Linear velocity part
        jacobian.block<3, 1>(0, joint_id) = z_current.cross(p_diff);
        // Angular velocity part
        jacobian.block<3, 1>(3, joint_id) = z_current;
    }

    return jacobian;
}

MatrixXd JacobBody(const VectorXd& Q, int index, bool isRightArm) {
    // Jacobian of the base 

    // Compute FK up to the specified joint index
    Matrix4d T_joint_index = FK(Q, index, isRightArm);

    // Extract the position vector of the joint at the specified index
    // in absolute frame relative to robot base.
    Vector3d P_joint_index = T_joint_index.block<3,1>(0, 3) - Vector3d(Q(0), Q(1), 0); // find the position of base: Q(0:2) = [x,y,theta]'

    // Nonholonomic matrix setup (assuming linear and angular velocity command scenario)
    MatrixXd nonholonomicMatrix(6, 3);
    nonholonomicMatrix << 1, 0, 0,
                          0, 1, 0,
                          0, 0, 0,
                          0, 0, 0,
                          0, 0, 0,
                          0, 0, 1;

    // Calculate the Jacobian for the platform
    Matrix3d crossProdMatrix = crossOperator(P_joint_index);
    MatrixXd JacobianPlatform(6, 6);
    JacobianPlatform.setZero(); // Initialize with zeros
    JacobianPlatform.block<3, 3>(0, 0) = Matrix3d::Identity();
    JacobianPlatform.block<3, 3>(0, 3) = -crossProdMatrix;
    JacobianPlatform.block<3, 3>(3, 3) = Matrix3d::Identity();

    // Apply the nonholonomic constraints to the platform Jacobian
    MatrixXd Jacobian = JacobianPlatform * nonholonomicMatrix;

    return Jacobian;
}

MatrixXd JacobianWholeBody(const VectorXd& Q, int index, bool isRightArm) {
    MatrixXd J_wb(6, 17); // Whole-body Jacobian matrix with 6 rows and 14 columns
    J_wb.setZero();

    MatrixXd J_arm = JacobArm(Q, index, isRightArm);
    MatrixXd J_body = JacobBody(Q, index, isRightArm);

    J_wb.leftCols(J_body.cols()) = J_body;
    if (isRightArm) {
        J_wb.block(0, J_body.cols(), 6, J_arm.cols()) = J_arm;
    } else {
        J_wb.block(0, J_body.cols() + J_arm.cols(), 6, J_arm.cols()) = J_arm;
    }

    return J_wb;
}

MatrixXd JacobArm_cg(const VectorXd& Q, int index, bool isRightArm) {
    MatrixXd jacobian(6, 7); // Pre-allocate for speed
    jacobian.setZero(); // initialize jacobian to zero. 
    MatrixXd linkCoMs(3, 7);
    
    // Updated CoM for each link (expressed as offsets from the link frame)
    linkCoMs << 0,   -0.0256,  0.00652, -0.0472, -0.00402,  0.1263,  0.1,
                -5.844e-9, -0.323,   0.0228,   0.000919, 0.0770, -0.0410, 0.00290,
                -0.391,    0.0706, -0.0701,   0.0287,  -0.376,  -0.0109, 0.0272;

    Matrix4d T_mi = FK(Q, index, isRightArm);
    Matrix3d R_mi = T_mi.block<3,3>(0,0);
    Vector3d joint_position_before_mi = T_mi.block<3,1>(0,3);
    Vector3d r_mi_abs = joint_position_before_mi + R_mi * linkCoMs.col(index); // center of mass position of link i in global frame relative to global frame

    for (int joint_id = 0; joint_id <= index; ++joint_id) {
        Matrix4d Ti = FK(Q, joint_id, isRightArm);
        Vector3d joint_position = Ti.block<3,1>(0,3);
        Vector3d joint_axis = Ti.block<3,1>(0,2); // frame z axis is equivalent to the joint axis vector.
        jacobian.block<3,1>(0, joint_id) = joint_axis.cross(r_mi_abs - joint_position);
        jacobian.block<3,1>(3, joint_id) = joint_axis;
    }

    return jacobian;
}

MatrixXd JacobBody_cg(const VectorXd& Q, int index, bool isRightArm) {
    MatrixXd Jacobian(6, 3);

    // Updated CoM for each link (expressed as offsets from the link frame)
    MatrixXd linkCoMs(3, 7);
    linkCoMs << 0,   -0.0256,  0.00652, -0.0472, -0.00402,  0.1263,  0.1,
                -5.844e-9, -0.323,   0.0228,   0.000919, 0.0770, -0.0410, 0.00290,
                -0.391,    0.0706, -0.0701,   0.0287,  -0.376,  -0.0109, 0.0272;

    Matrix4d T_mi = FK(Q, index, isRightArm);
    Matrix3d R_mi = T_mi.block<3,3>(0,0);
    Vector3d joint_position_before_cg = T_mi.block<3,1>(0,3);
    Vector3d r_mi_abs = joint_position_before_cg + R_mi * linkCoMs.col(index); // center of mass position of link i in global frame

    Vector3d r_mi_abs_rel = r_mi_abs - Vector3d(Q(0), Q(1), 0); // center of mass position of link i relative to mobile base frame Q(0:2) = [x,y,theta]'

    // Nonholonomic matrix setup (assuming linear and angular velocity command scenario)
    MatrixXd nonholonomicMatrix(6, 3);
    nonholonomicMatrix << 1, 0, 0,
                          0, 1, 0,
                          0, 0, 0,
                          0, 0, 0,
                          0, 0, 0,
                          0, 0, 1;

    // Calculate the Jacobian for the platform
    Matrix3d crossProdMatrix = crossOperator(r_mi_abs_rel);
    MatrixXd JacobianPlatform(6, 6);
    JacobianPlatform.setZero(); // Initialize with zeros
    JacobianPlatform.block<3, 3>(0, 0) = Matrix3d::Identity();
    JacobianPlatform.block<3, 3>(0, 3) = -crossProdMatrix;
    JacobianPlatform.block<3, 3>(3, 3) = Matrix3d::Identity();

    // Apply the nonholonomic constraints to the platform Jacobian
    Jacobian = JacobianPlatform * nonholonomicMatrix;

    return Jacobian;
}

MatrixXd JacobianWholeBody_cg(const VectorXd& Q, int index, bool isRightArm) {
    MatrixXd J_wb(6, 17);
    J_wb.setZero();
    
    MatrixXd J_arm_cg = JacobArm_cg(Q, index, isRightArm);
    MatrixXd J_body_cg = JacobBody_cg(Q, index, isRightArm);

    J_wb.leftCols(J_body_cg.cols()) = J_body_cg;
    if (isRightArm) {
        J_wb.block(0, J_body_cg.cols(), 6, J_arm_cg.cols()) = J_arm_cg;
    } else {
        J_wb.block(0, J_body_cg.cols() + J_arm_cg.cols(), 6, J_arm_cg.cols()) = J_arm_cg;
    }
    return J_wb;
}

void computeDynamics(const VectorXd& Q, const VectorXd& Qdot, MatrixXd& M, MatrixXd& C, VectorXd& G) {
    // Dynamic parameters
    Vector3d g(0, 0, -9.81);

    // Updated mass for each link using VectorXd
    VectorXd linkMasses(7);
    linkMasses << 4.07, 5.996, 0.570, 3.083, 3.616, 1.435, 0.453;

    // Initialize inertia matrices for each link, approximated and written inline
    MatrixXd linkInertia(3, 21);
    
    linkInertia.block<3, 3>(0, 0) << 4.47e-6, 0, 0, 0, 4.47e-6, -5.61e-10, 0, -5.61e-10, 2.00e-6; // Link 1 inertia
    linkInertia.block<3, 3>(0, 3) << 0.0465, -0.00338, 0.000748, -0.00338, 0.00243, 0.00980, 0.000748, 0.00980, 0.0445;  // Link 2 inertia
    linkInertia.block<3, 3>(0, 6) << 0.0504, 0.00248, -0.0118, 0.00248, 0.0525, 0.00747, -0.0118, 0.00747, 0.00817;  // Link 3 inertia
    linkInertia.block<3, 3>(0, 9) << 0.0308, 0.00732, 0.000194, 0.00732, 0.00278, 0.00101, 0.000194, 0.00101, 0.0329;  // Link 4 inertia
    linkInertia.block<3, 3>(0, 12) << 0.0511, -0.000984, -0.00223, -0.000984, 0.0472, 0.00346, -0.00223, 0.00346, 0.00475;  // Link 5 inertia
    linkInertia.block<3, 3>(0, 15) << 0.00823, 0.00463, -0.00127, 0.00463, 0.0187, 0.00110, -0.00127, 0.00110, 0.0144;  // Link 6 inertia
    linkInertia.block<3, 3>(0, 18) << 0.00378, -3.27e-5, -0.000200, -3.27e-5, 0.00368, -0.000602, -0.000200, -0.000602, 0.000122;  // Link 7 inertia

    // Mass and Inertia parameters for mobile base
    MatrixXd baseMass = MatrixXd::Zero(3, 3);
    baseMass.diagonal() << 50, 50, 15; // Set the diagonal elements to 10, 10, and 3

    // Initialize matrices M, C, and G
    M = MatrixXd::Zero(17, 17);
    C = MatrixXd::Zero(17, 17);
    G = VectorXd::Zero(17);

    for (int arm = 1; arm <= 2; ++arm) {
        VectorXd q_arm = (arm == 1) ? Q.segment(3, 7) : Q.segment(10, 7);
        for (int joint_id = 0; joint_id < 7; ++joint_id) {
            MatrixXd Tcg = FK(Q, joint_id, arm == 1);
            Matrix3d Rcg = Tcg.block<3, 3>(0, 0);
            MatrixXd Jwb_cg = JacobianWholeBody_cg(Q, joint_id, arm == 1);
            double linkmass = linkMasses(joint_id);
            Matrix3d InertiaMatrix = linkInertia.block<3, 3>(0, 3 * (joint_id));

            MatrixXd mass_link = MatrixXd::Zero(6, 6);
            mass_link.block<3, 3>(0, 0) = Matrix3d::Identity() * linkmass;
            mass_link.block<3, 3>(3, 3) = Rcg * InertiaMatrix * Rcg.transpose();

            MatrixXd Jwb_cg_v = Jwb_cg.block<3, 17>(0, 0);

            M += Jwb_cg.transpose() * mass_link * Jwb_cg;
            G -= linkmass * Jwb_cg_v.transpose() * g;
        }
    }

    MatrixXd jacobBase(3, 17);
    jacobBase << Matrix3d::Identity(), MatrixXd::Zero(3, 14);
    M += jacobBase.transpose() * baseMass * jacobBase;
    // C and G adjustments if necessary (not modified in this example)

}

double calculateDistance(const MatrixXd& link1, const MatrixXd& link2) {
    gkFloat dd;
    gkSimplex s;
    gkPolytope lk1, lk2;

    // Allocate and convert Eigen matrix to the format expected by openGJK
    auto convertMatrix = [](const MatrixXd& mat) -> gkFloat** {
        gkFloat** arr = new gkFloat*[mat.cols()];
        for (int i = 0; i < mat.cols(); ++i) {
            arr[i] = new gkFloat[3];
            for (int j = 0; j < 3; ++j) {
                arr[i][j] = static_cast<gkFloat>(mat(j, i));
            }
        }
        return arr;
    };

    gkFloat** gjkLink1 = convertMatrix(link1);
    gkFloat** gjkLink2 = convertMatrix(link2);

    lk1.coord = gjkLink1;
    lk1.numpoints = link1.cols();

    lk2.coord = gjkLink2;
    lk2.numpoints = link2.cols();

    s.nvrtx = 0;

    dd = compute_minimum_distance(lk1, lk2, &s);

    // Free allocated memory
    for (int i = 0; i < link1.cols(); ++i) {
        delete[] gjkLink1[i];
    }
    delete[] gjkLink1;
    for (int i = 0; i < link2.cols(); ++i) {
        delete[] gjkLink2[i];
    }
    delete[] gjkLink2;

    return static_cast<double>(dd);
}

VectorXd CollisionAvoidanceDistances(const VectorXd& Q, bool isRightArm) {
    Vector7d minDistances;
    minDistances.fill(std::numeric_limits<double>::max());

    MatrixXd positions = getJointsPositions(Q, isRightArm);
    MatrixXd positions_other = getJointsPositions(Q, !isRightArm);


    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < 7; ++j) {
            MatrixXd link(3, 3);
            link.col(0) = positions.col(i);
            link.col(1) = 0.5 * (positions.col(i) + positions.col(i + 1));
            link.col(2) = positions.col(i + 1);

            MatrixXd link_other(3, 3);
            link_other.col(0) = positions_other.col(j);
            link_other.col(1) = 0.5 * (positions_other.col(j) + positions_other.col(j + 1));
            link_other.col(2) = positions_other.col(j + 1);

            double dist = calculateDistance(link, link_other);
            minDistances[i] = std::min(minDistances[i], dist);
        }
    }

    return minDistances;
}

// Function to compute the gradient of manipulability 
VectorXd ManipulabilityGradient(const VectorXd& Q, bool isRightArm) {
    // This should be in the nullspace 
    double epsilon = 1e-6; 
    double k_manipulability = 20;    // gain for the manipulability torque --- avoid high values
    VectorXd gradient(7);

    // Compute the Jacobian(end-effector) and manipulability at q and q_0
    MatrixXd J = JacobArm(Q, 7, isRightArm);

    double manipulability_at_Q = std::sqrt((J * J.transpose()).determinant());
    double manipulability_zero = 1.2;   // the manipulability should be greater than this value to avoid singularity

    k_manipulability = (k_manipulability > 50) ? 50 : k_manipulability; // saturate  gain manipulability to 50
    
    // Compute the potential manipulability at q
    double potential_manipulability_at_Q = manipulability_at_Q <= manipulability_zero
                                           ? k_manipulability*std::pow(manipulability_at_Q - manipulability_zero, 2)
                                           : 0;

    double starting_index = (isRightArm == 1) ? 3 : 10;  // starting index for left or right arm elements

    for (int i = 0; i < 7; ++i) {   // count for all joint number of the right or left arm. 
        // Perturb joint angle by epsilon
        VectorXd Q_epsilon = Q;
        Q_epsilon(i + starting_index) = Q_epsilon(i + starting_index) + epsilon;

        // Compute manipulability at perturbed joint state
        MatrixXd J_epsilon = JacobArm(Q_epsilon,7,isRightArm);
        double manipulability_at_Q_epsilon = std::sqrt((J_epsilon * J_epsilon.transpose()).determinant());

        // Compute the potential manipulability at Q_epsilon
        double potential_manipulability_at_Q_epsilon = manipulability_at_Q_epsilon <= manipulability_zero
                                                       ? k_manipulability*std::pow(manipulability_at_Q_epsilon - manipulability_zero, 2)
                                                       : 0;

        // Compute gradient component
        gradient(i) = (potential_manipulability_at_Q_epsilon - potential_manipulability_at_Q) / epsilon;
    }
    return gradient;
}

VectorXd JointLimitPotentialGradient(const VectorXd& Q, bool isRightArm) {
    // Internal parameters
    double delta = 0.1;
    double epsilon = 1e-6;
    double k_qlim = 500;

    // Hard-coded qmax and qmin values for a 7-joint robotic arm
    VectorXd qmax(7);
    qmax << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
    VectorXd qmin(7);
    qmin << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;

    VectorXd gradient = VectorXd::Zero(7);
    VectorXd qtilda = VectorXd::Zero(7);

    double starting_index = (isRightArm == 1) ? 3 : 10;  // starting index for left or right arm elements

    // Calculate qtilda for the current q
    for (int i = 0; i <  7; ++i) {
        if (Q(i + starting_index) > qmax(i) - delta) {
            qtilda(i) = Q(i + starting_index) - (qmax(i) - delta);
        } else if (Q(i + starting_index) < qmin(i) + delta) {
            qtilda(i) = Q(i + starting_index) - (qmin(i) + delta);
        }
        // qtilda(i) is 0 when not within delta range of limits
    }

    // Calculate the potential at the current q
    double potential_jointlimit_at_Q = 0.5 * qtilda.transpose() * k_qlim * qtilda;

    // Numerically derive the gradient
    for (int i = 0; i < 7 ; ++i) {
        VectorXd Q_epsilon = Q;
        Q_epsilon(i + starting_index) += epsilon;

        // Calculate qtilda for Q_epsilon
        VectorXd qtilda_epsilon = VectorXd::Zero(7);
        for (int j = 0; j < 7; ++j) {
            if (Q_epsilon(j + starting_index) > qmax(j) - delta) {
                qtilda_epsilon(j) = Q_epsilon(j + starting_index) - (qmax(j) - delta);
            } else if (Q_epsilon(j + starting_index) < qmin(j) + delta) {
                qtilda_epsilon(j) = Q_epsilon(j + starting_index) - (qmin(j) + delta);
            }
             // is 0 when not within delta range of limits
        }

        // Calculate the potential at Q_epsilon
        double potential_jointlimit_at_Q_epsilon = 0.5 * qtilda_epsilon.transpose() * k_qlim * qtilda_epsilon;

        // Compute the gradient component
        gradient(i) = (potential_jointlimit_at_Q_epsilon - potential_jointlimit_at_Q) / epsilon;
    }

    return gradient;
}

VectorXd CollisionAvoidanceGradient(const VectorXd& Q, const bool isRightArm) {
    // Internal parameters
    double epsilon = 1e-6;
    double Fmax = 500;
    double collisionRadius = 0.23; // Radius for collision avoidance

    VectorXd gradient = VectorXd::Zero(7);

    // Calculate the minimum distances of the current arm links to the other arm links
    VectorXd minDistances = CollisionAvoidanceDistances(Q, isRightArm);

    // Calculate the potential at the current q
    double potential_collision_at_Q = 0;
    for (int i = 0; i < minDistances.size(); ++i) {
        if (minDistances(i) < collisionRadius) {
            potential_collision_at_Q -= Fmax / pow(collisionRadius, 2) / 3 * pow(minDistances(i) - collisionRadius, 3);
        }
    }

    double starting_index = (isRightArm == 1) ? 3 : 10;  // starting index for left or right arm elements
    // Numerically derive the gradient
    for (int i = 0; i < 7; ++i) {
        VectorXd Q_epsilon = Q;
        Q_epsilon(i + starting_index) += epsilon;

        // Calculate minimum distances for Q_epsilon
        VectorXd minDistances_epsilon = CollisionAvoidanceDistances(Q_epsilon, isRightArm);

        double potential_collision_at_Q_epsilon = 0;
        for (int j = 0; j < minDistances_epsilon.size(); ++j) {
            if (minDistances_epsilon(j) < collisionRadius) {
                potential_collision_at_Q_epsilon -= Fmax / pow(collisionRadius, 2) / 3 * pow(minDistances_epsilon(j) - collisionRadius, 3);
            }
        }

        // Compute the gradient component
        gradient(i) = (potential_collision_at_Q_epsilon - potential_collision_at_Q) / epsilon;
    }

    return gradient;
}

ControlOutput wholeBodyController(const Matrix<double, 6, 2>& X_task, const VectorXd& Q_null, const VectorXd& Q, const VectorXd& Qdot, double currentTime)
{
    // Control parameters for task space
    double M_task_translation = 4;   // Mass gain for translation components -- not used
    double M_task_orientation = 2;   // Mass gain for orientation components -- not used
    double K_task_translation = 100.0; // Stiffness for translation components
    double K_task_orientation = 20.0; // Stiffness for orientation components
    double D_task_translation = 45.0; // Damping for translation components -- don't increase this value
    double D_task_orientation = 4.0; // Damping for orientation components -- don't increase this value

    // Diagonal matrices for K_task, and D_task
    MatrixXd M_task = MatrixXd::Zero(6, 6);
    M_task.diagonal() << VectorXd::Constant(3, M_task_translation), VectorXd::Constant(3, M_task_orientation);
    MatrixXd K_task = MatrixXd::Zero(6, 6);
    K_task.diagonal() << VectorXd::Constant(3, K_task_translation), VectorXd::Constant(3, K_task_orientation);
    MatrixXd D_task = MatrixXd::Zero(6, 6);
    D_task.diagonal() << VectorXd::Constant(3, D_task_translation), VectorXd::Constant(3, D_task_orientation);

    // Base, right arm, and left arm gains
    double K_null_base = 10.0; // Gain for the base
    double D_null_base = 45.0; // Damping for the base
    double K_null_right_arm = 1.0; // Gain for the right arm
    double K_null_left_arm = 1.0; // Gain for the left arm
    double D_null_right_arm = 1.0; // Damping for the right arm
    double D_null_left_arm = 1.0; // Damping for the left arm

    // Diagonal matrices for K_null and D_null
    MatrixXd K_null = MatrixXd::Zero(17, 17);
    K_null.diagonal() << VectorXd::Constant(3, K_null_base), VectorXd::Constant(7, K_null_right_arm), VectorXd::Constant(7, K_null_left_arm);
    MatrixXd D_null = MatrixXd::Zero(17, 17);
    D_null.diagonal() << VectorXd::Constant(3, D_null_base), VectorXd::Constant(7, D_null_right_arm), VectorXd::Constant(7, D_null_left_arm);

    // Dynamics matrices
    MatrixXd M(17, 17);
    MatrixXd C(17, 17);
    VectorXd G(17);

    // Compute sub-tasks
    VectorXd tau_qlim(17);
    tau_qlim << 0.0,0.0,0.0,-JointLimitPotentialGradient(Q,true),-JointLimitPotentialGradient(Q,false);  // apply on right and left arm. no influence on the base
    VectorXd tau_manipulability(17);
    tau_manipulability.setZero();
    // This is really heavy
    //tau_manipulability << 0.0,0.0,0.0,-ManipulabilityGradient(Q,true),-ManipulabilityGradient(Q,false);  // apply on right and left arm. no influence on the base
    VectorXd tau_collision_avoidance(17);
    tau_collision_avoidance.setZero();
    // This is really heavy - collision avoidance should be included though for the two arms to be coupled
    // try calling this every n cycle, retain the value
    //tau_collision_avoidance << 0.0,0.0,0.0,-CollisionAvoidanceGradient(Q,true),-CollisionAvoidanceGradient(Q,false);  // apply on right and left arm. no influence on the base
    
    // compute null-space torques
    MatrixXd Qdot_null = MatrixXd::Zero(17, 1);
    VectorXd tau_null = K_null * (Q_null - Q) + D_null * (Qdot_null - Qdot) + tau_manipulability;

    // Adjustable gains for simulating disturbances in x and theta directions
    double gain_x = 10.0; // Gain for disturbance in the x direction
    double gain_theta = 12.0; // Gain for disturbance in the theta direction

    // Assuming computeDynamics, JacobianWholeBody, and pose_arm are defined elsewhere
    computeDynamics(Q, Qdot, M, C, G); // Compute dynamics

    // Constraint matrix A and its derivative
    MatrixXd A(1, 17); // Nonholonomic constraint matrix
    A << -sin(Q(2)), cos(Q(2)), 0, MatrixXd::Zero(1, 14);
    MatrixXd Adot(1, 17); // Derivative of A
    Adot << -Qdot(2) * cos(Q(2)), -Qdot(2) * sin(Q(2)), 0, MatrixXd::Zero(1, 14);

    // Auxiliary and arbitrary matrix D to make a non-singular square constraint matrix and its derivative
    MatrixXd D = MatrixXd::Zero(16, 17);
    D.block(0, 0, 1, 3) << cos(Q(2)), sin(Q(2)), 0;
    D.block(1, 2, 15, 15) = MatrixXd::Identity(15, 15);
    MatrixXd Ddot = MatrixXd::Zero(16, 17);
    Ddot.block(0, 0, 1, 3) << -Qdot(2) * sin(Q(2)), Qdot(2) * cos(Q(2)), 0;

    // Combine A and D into a square matrix
    MatrixXd A_D(17, 17);
    A_D.block(0, 0, 1, 17) << A;
    A_D.block(1, 0, 16, 17) << D;

    // Inverse of A_D
    MatrixXd inverse_A_D = MatrixXd::Zero(17, 17);
    inverse_A_D.setZero();
    inverse_A_D.block(0, 0, 2, 2) << -sin(Q(2)), cos(Q(2)), cos(Q(2)), sin(Q(2));
    inverse_A_D.block(2, 2, 15, 15) = MatrixXd::Identity(15, 15);

    // Extract E and F from the inverse of A_D
    VectorXd E = inverse_A_D.col(0);
    MatrixXd F = inverse_A_D.rightCols(16);

    // Pseudo velocities
    VectorXd v = D * Qdot;

    // Compute whole body Jacobian for right and left arm
    MatrixXd Jwb_right = JacobianWholeBody(Q, 7, true);
    MatrixXd Jwb_left = JacobianWholeBody(Q, 7, false);

    // Reduced Jacobian for combined arms
    MatrixXd Jreduced(12, 16);
    Jreduced.block(0, 0, 6, 16) = Jwb_right * F;
    Jreduced.block(6, 0, 6, 16) = Jwb_left * F;

    // Define controller variables 
    VectorXd tau_wb(17);
    static VectorXd v_wb(16);
    static MatrixXd prev_Jreduced; // Static variable to store Jreduced from the previous call
    static MatrixXd Xdot_prev(6,2);
    static double prev_time = -1; // Static variable to store the time of the previous call
    MatrixXd Jdot_reduced = MatrixXd::Zero(12, 16); // Variable to store the computed rate of change of Jreduced
    double deltaTime;

    // Check if prev_time is initialized
    if (prev_time >= 0) {
        deltaTime = currentTime - prev_time;
        Jdot_reduced = (Jreduced - prev_Jreduced) / deltaTime;
    }
    else
    {
        tau_wb.setZero(); // Initialize tau_wb
        v_wb  = v; // Initialize v_wb
        Xdot_prev.setZero();
    }

    // Update the static variables for the next call
    prev_Jreduced = Jreduced;
    prev_time = currentTime;

    VectorXd X_right = pose_arm(Q, 7, true);
    VectorXd X_left = pose_arm(Q, 7, false);

    // Concatenate X for both arms
    MatrixXd X(6, 2);
    X.col(0) = X_right;
    X.col(1) = X_left;

    // Compute Xdot for both arms and concatenate
    VectorXd Xdot_right = Jwb_right * Qdot;
    VectorXd Xdot_left = Jwb_left * Qdot;
    MatrixXd Xdot(6, 2), Xdot_task(6, 2);
    Xdot.col(0) = Xdot_right;
    Xdot.col(1) = Xdot_left;
    Xdot_task.setZero();

    VectorXd u_task(16, 1);
    VectorXd u_null(16, 1);
    VectorXd u(16,1);
    u_task.setZero();
    u_null.setZero();
    u.setZero();

    // Reduced mass matrix and its inverse
    MatrixXd M_reduced = F.transpose() * M * F;
    MatrixXd M_reduced_inv = M_reduced.inverse();

    for (int arm_id = 0; arm_id < 2; ++arm_id) {
        MatrixXd Jr = Jreduced.block(6 * arm_id, 0, 6, 16);
        MatrixXd Jd_r = Jdot_reduced.block(6 * arm_id, 0, 6, 16);
        MatrixXd JrT = Jr.transpose();
        MatrixXd lambda = Jr * M_reduced_inv * JrT;  // auxilary parameter that stors the inverse of Mc
        MatrixXd Mc = (lambda).inverse(); // Compute cartesian mass that decouples the dynamics of null space with task space.
        
        // MatrixXd Jr_plus = JrT * (Jr * JrT).inverse(); // Compute pseudo-inverse of Jr
        MatrixXd Jr_plus_M = M_reduced_inv * JrT * Mc;  // Compute weighted pseudo-inverse
        

        VectorXd F_task = lambda * (K_task * (X_task.col(arm_id) - X.col(arm_id)) + D_task * (Xdot_task.col(arm_id) - Xdot.col(arm_id)));
        u_task += M_reduced * Jr_plus_M * (F_task - Jd_r * v);

        MatrixXd Nr = MatrixXd::Identity(16, 16) - Jr_plus_M * Jr;
        u_null += Nr.transpose() * F.transpose() * (tau_null);
    }

    u << u_task + u_null;
    MatrixXd F_p_pseudo = F * (F.transpose() * F).inverse();
    tau_wb = (C * Qdot - M * (E * Adot + F * Ddot) * F * v) + F_p_pseudo * u + tau_qlim + tau_collision_avoidance;   // the robot arm has its own gravity compensation and thus no need to add as feedforward term here.

    VectorXd vdot = M_reduced_inv * (u + F.transpose() * (tau_qlim + tau_collision_avoidance));
    v_wb = v_wb + deltaTime*vdot;

    ControlOutput output;
    output.tau_wb = tau_wb;  // for torque based control
    output.v_wb = v_wb; // for velocity based control

    return output;
}
} // namespace redundancy_resolution
/*
// Computes differential drive velocities to reach a goal pose from a current pose, with constraints on maximum velocity
Eigen::Vector2d DiffDriveVelocity(const VectorXd& currentPosition, const VectorXd& goalPosition) {
    // Constants for velocity calculation
    double velocityGain = 0.5; // Gain for scaling the velocity
    double gainLinearX = 30.0; // Gain for correcting the x-component of position
    double gainLinearY = 30.0; // Gain for correcting the y-component of position
    double gainAngularTheta = 2.0 * sqrt(gainLinearY); // Gain for correcting the orientation
    Vector3d maxVelocity(0.05, 0.05, 0.05); // Maximum allowed velocity for each component

    // Calculate the raw desired velocity in the global frame
    Vector3d rawDesiredVelocityGlobal = velocityGain * (goalPosition - currentPosition);

    // Limit the desired velocity to the maximum values
    Vector3d desiredVelocityGlobal;
    for (int i = 0; i < 3; ++i) {
        desiredVelocityGlobal(i) = std::min(rawDesiredVelocityGlobal(i), maxVelocity(i));
    }

    // Current orientation angle (theta)
    double currentTheta = currentPosition(2); // Assuming the third element is theta

    // Rotation matrix for transforming global velocities to the robot's local frame
    Matrix3d rotationMatrixToRobotFrame;
    rotationMatrixToRobotFrame << cos(currentTheta), -sin(currentTheta), 0,
                                  sin(currentTheta),  cos(currentTheta), 0,
                                  0,                  0,                 1;

    // Transform desired global velocity to the robot's local frame
    Vector3d desiredVelocityRobotFrame = rotationMatrixToRobotFrame.transpose() * desiredVelocityGlobal;

    // Extract linear and angular velocities in the robot frame
    double linearVelocity = desiredVelocityRobotFrame(0);
    double angularVelocity = desiredVelocityRobotFrame(2);

    // Compute the error vector in the robot's local frame
    Vector3d errorVectorRobotFrame = rotationMatrixToRobotFrame.transpose() * (goalPosition - currentPosition);

    // Components of the error vector
    double errorX = errorVectorRobotFrame(0);
    double errorY = errorVectorRobotFrame(1);
    double errorTheta = errorVectorRobotFrame(2);

    // Corrected velocities using the error components
    double correctedLinearVelocity = linearVelocity * cos(errorTheta) + gainLinearX * errorX;
    double correctedAngularVelocity = angularVelocity + linearVelocity * (gainLinearY * errorY + gainAngularTheta * sin(errorTheta));

    // Construct the velocity vector with corrected linear and angular velocities
    Eigen::Vector2d velocities(correctedLinearVelocity, correctedAngularVelocity);
    return velocities;
}

// Function to calculate instantaneous velocities xdot, ydot, and thetadot for one step
// with default rho and time_step, including rotation by yaw angle
Vector3d dubins_instantaneous_velocity(Vector3d& pose_base_init, Vector3d& pose_base_goal) {
    // Default values
    double rho = 0.1;       // Default turning radius
    double time_step = 0.001; // Default time step

    Vector3d velocity; // Vector to store xdot, ydot, and thetadot

    double threshold = 0.05; // Set your desired threshold
    Vector3d pose_difference = pose_base_init - pose_base_goal;
    if (pose_difference.norm() < threshold) {
        return Vector3d::Zero(); // Return zero velocities
    }

    // Use the 2D position and yaw directly from the input vectors
    double pose_init[3] = {pose_base_init[0], pose_base_init[1], pose_base_init[2]};
    double pose_goal[3] = {pose_base_goal[0], pose_base_goal[1], pose_base_goal[2]};

    
    DubinsPath path;
    int err = dubins_shortest_path(&path, pose_init, pose_goal, rho);
    if (err != EDUBOK) {
        return velocity; // Return zero velocity if an error occurs
    }

    // Sample the final pose (at the end of the time_step)
    double sampled_pose[3];
    dubins_path_sample(&path, time_step, sampled_pose);

    // Calculate instantaneous linear velocities
    double xdot = (sampled_pose[0] - pose_init[0]) / time_step;
    double ydot = (sampled_pose[1] - pose_init[1]) / time_step;

    // Rotation matrix for yaw angle
    double yaw = sampled_pose[2];
    Eigen::Matrix2d R;
    R << cos(yaw), -sin(yaw),
         sin(yaw), cos(yaw);

    Eigen::Vector2d linear_velocities(xdot, ydot);
    linear_velocities = R.transpose()* linear_velocities; // Rotate the velocity vector

    // Assign rotated velocities and thetadot to the velocity vector
    velocity(0) = linear_velocities(0); // xdot
    velocity(1) = linear_velocities(1); // ydot
    velocity(2) = (sampled_pose[2] - pose_init[2]) / time_step; // thetadot

    return velocity;
}

// Base stuff - not used? 
// VectorXd poseNonHolonomic( const VectorXd& pose_base, const VectorXd& twist_base) {
//     double orientation = 0;
//     if (!(twist_base(0) == 0 && twist_base(1) == 0)) {
//         orientation = std::atan2(twist_base(1), twist_base(0));
//     }
//     VectorXd pose_nonholonomic(3);
//     pose_nonholonomic << pose_base(0), pose_base(1), orientation;
//     return pose_nonholonomic;
// }

// VectorXd poseErrorWithRotation(const VectorXd& pose_base_ref, const VectorXd& pose_base) {
//     VectorXd e = pose_base_ref - pose_base;
//     double theta = pose_base(2);

//     MatrixXd R_z(3, 3);
//     R_z << std::cos(theta), -std::sin(theta), 0,
//            std::sin(theta),  std::cos(theta), 0,
//            0,                0,                1;

//     VectorXd e_rotated = R_z.transpose() * e;
//     return e_rotated;
// }

// VectorXd baseController(const VectorXd& twist_base_ref, const VectorXd& e_rotated) {
//     double kx = 10.0;
//     double ky = 10.0;
//     double k_theta = 2 * std::sqrt(ky);

//     double x_e = e_rotated(0);
//     double y_e = e_rotated(1);
//     double theta_e = e_rotated(2);

//     double v_r = twist_base_ref.head(2).norm(); // Linear velocity command
//     double w_r = 0; // Angular velocity command, not directly calculated from twist_base in this example

//     double v = v_r * std::cos(theta_e) + kx * x_e;
//     double w = w_r + v_r * (ky * y_e + k_theta * std::sin(theta_e));

//     VectorXd u(2);
//     u << v, w;
//     return u;
// }

*/
