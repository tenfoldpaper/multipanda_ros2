#pragma once

#include <Eigen/Dense>

// see https://frankaemika.github.io/docs/control_parameters.html
namespace panda_limits {
// Joint limits
const Eigen::Matrix<double, 7, 1> q_max = // max joint angles (rad)
    (Eigen::Matrix<double, 7, 1>() <<
    2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973).finished();
const Eigen::Matrix<double, 7, 1> q_min = // min joint angles (rad)
    (Eigen::Matrix<double, 7, 1>() <<
    -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973).finished();
const Eigen::Matrix<double, 7, 1> qD_max = // max joint velocity (rad/s)
    (Eigen::Matrix<double, 7, 1>() <<
    2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100).finished();
const Eigen::Matrix<double, 7, 1> qDD_max = // max joint acceleration (rad/s²)
    (Eigen::Matrix<double, 7, 1>() <<
    15, 7.5, 10, 12.5, 15, 20, 20).finished();
const Eigen::Matrix<double, 7, 1> qDDD_max = // max joint jerk (rad/s³)
    (Eigen::Matrix<double, 7, 1>() <<
    7500, 3750, 5000, 6250, 7500, 10000, 10000).finished();
const Eigen::Matrix<double, 7, 1> tau_max = // max joint torque (Nm)
    (Eigen::Matrix<double, 7, 1>() <<
    87, 87, 87, 87, 12, 12, 12).finished();
const Eigen::Matrix<double, 7, 1> tauD_max = // max joint rotatum (Nm/s)
    (Eigen::Matrix<double, 7, 1>() <<
    1000, 1000, 1000, 1000, 1000, 1000, 1000).finished();
// Cartesian limits
const double tD_max = 1.7; // m/s, max translational velocity
const double tDD_max = 13; // m/s² max translational acceleration
const double tDDD_max = 6500; // m/s³ max translational jerk
const double rD_max = 2.5; // rad/s max rotational velocity
const double rDD_max = 25; // rad/s² max rotational acceleration
const double rDDD_max = 12500; // rad/s³ max rotational jerk
const double eD_max = 2.175; // rad/s max elbow velocity
const double eDD_max = 10; // rad/s² max elbow acceleration
const double eDDD_max = 5000; // rad/s³ max elbow jerk

}
