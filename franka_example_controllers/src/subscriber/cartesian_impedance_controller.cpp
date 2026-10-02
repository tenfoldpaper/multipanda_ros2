#include <franka_example_controllers/subscriber/cartesian_impedance_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <franka/model.h>

// Function to calculate the pseudo-inverse of a matrix using Singular Value Decomposition (SVD)
inline void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true) {
    double lambda_ = damped ? 0.2 : 0.0;

    // Step 1: Perform Singular Value Decomposition (SVD) on the input matrix M_
    // SVD breaks down the matrix M_ into three matrices: U, Σ (Singular Values), and V^T
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // Step 2: Get the singular values (Σ) from the SVD (these are the diagonal values of Σ)
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
    
    // Step 3: Create a new matrix S_ of the same size as M_ (just a copy for dimensions)
    Eigen::MatrixXd S_ = M_;  // copying the dimensions of M_, its content is not needed.
    S_.setZero();

    // Step 4: Calculate the inverse of each singular value (for pseudo-inverse)
    for (int i = 0; i < sing_vals_.size(); i++)
        S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

    // Step 5: Calculate the pseudo-inverse using the SVD formula:
    // M_pinv_ = V * S^T * U^T
    M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}


namespace franka_example_controllers {

// Define the output of the controller
controller_interface::InterfaceConfiguration
CartesianImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

// Define the robot monitoring points
controller_interface::InterfaceConfiguration
CartesianImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // should be model interface
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(franka_robot_model_name);
  }
  return config;
}

// The main update function of the Cartesian Impedance Controller
controller_interface::return_type CartesianImpedanceController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  
  // Getting the current end-effector pose (4x4 transformation matrix)
  Eigen::Map<const Matrix4d> current(franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector).data());

  // Extracting the current position (XYZ coordinates) from the pose matrix
  Eigen::Vector3d current_position(current.block<3,1>(0,3));

  // Extracting the current orientation (3x3 rotation matrix) and converting it to a quaternion
  Eigen::Quaterniond current_orientation(current.block<3,3>(0,0));

  // Getting the robot's inertia matrix (7x7), which represents the mass distribution
  Eigen::Map<const Matrix7d> inertia(franka_robot_model_->getMassMatrix().data());

  // Getting the Coriolis force vector (forces caused by the robot's movement)
  Eigen::Map<const Vector7d> coriolis(franka_robot_model_->getCoriolisForceVector().data());

  // Getting the Jacobian matrix (6x7), which maps joint velocities to end-effector velocity
  Eigen::Matrix<double, 6, 7> jacobian(
      franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector).data());

  // Getting the current joint velocities (7 values, one for each joint)
  Eigen::Map<const Vector7d> qD(franka_robot_model_->getRobotState()->dq.data());

  // Getting the current joint positions (7 values, one for each joint)
  Eigen::Map<const Vector7d> q(franka_robot_model_->getRobotState()->q.data());

  // Defining an error vector for the position and orientation (6D)
  Vector6d error;

  // Storing the desired position (which we will use for comparison)
  auto desired_position_cur = desired_position;

  // Calculating the position error (difference between current and desired)
  error.head(3) << current_position - desired_position_cur;

  // Correcting the orientation to avoid sudden flips
  if (desired_orientation.coeffs().dot(current_orientation.coeffs()) < 0.0) {
    current_orientation.coeffs() << -current_orientation.coeffs();
  }

  // Calculating the orientation error using quaternions
  Eigen::Quaterniond rot_error(
      current_orientation * desired_orientation.inverse());

  // Converting the rotation error to an axis-angle format for better control
  Eigen::AngleAxisd rot_error_aa(rot_error);

  // Storing the orientation error in the last three elements of the error vector
  error.tail(3) << rot_error_aa.axis() * rot_error_aa.angle();

  // Defining torque vectors for the control
  Vector7d tau_task, tau_nullspace, tau_d;

  // Initializing the torque vectors to zero
  tau_task.setZero();
  tau_nullspace.setZero();
  tau_d.setZero();

  // Calculating the task-space torque (effort to correct position and orientation)
  tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * qD));
  // - stiffness * error -> Proportional control (stiffness)
  // - damping * (jacobian * qD) -> Derivative control (damping)

  // Calculating the pseudo-inverse of the Jacobian transpose (for nullspace control)
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // Calculating the nullspace torque (controls redundant joints without affecting end-effector)
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                      jacobian.transpose() * jacobian_transpose_pinv) *
                        (n_stiffness * (desired_qn - q) -
                          (2.0 * sqrt(n_stiffness)) * qD);
  // n_stiffness * (desired_qn - q) -> Proportional control for redundant joints
  // (2.0 * sqrt(n_stiffness)) * qD -> Damping control for redundant joints

  // Calculating the final torque command (combining task and nullspace)
  tau_d << tau_task + coriolis + tau_nullspace;
  // tau_task -> Main control effort (position and orientation)
  // coriolis -> Compensates for dynamic forces
  // tau_nullspace -> Controls redundant degrees of freedom

  // Applying the calculated torques to each joint of the robot
  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_d(i));
  }

  // Indicating that the update was successful
  return controller_interface::return_type::OK;
}


// Initialization function for the Cartesian Impedance Controller
CallbackReturn CartesianImpedanceController::on_init() {
  try {
    // Declaring a parameter "arm_id" of type string with default value "panda"
    auto_declare<std::string>("arm_id", "panda");

    // Declaring a parameter "pos_stiff" (position stiffness) with a default value of 100
    auto_declare<double>("pos_stiff", 100);

    // Declaring a parameter "rot_stiff" (rotation stiffness) with a default value of 10
    auto_declare<double>("rot_stiff", 10);

    // Creating a subscription to the topic "/cartesian_impedance/pose_desired"
    // This means the controller will listen for messages (target positions/orientations) on this topic
    sub_desired_cartesian_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/cartesian_impedance/pose_desired", // Topic name
      1,                                   // Queue size (1 message at a time)
      std::bind(                          // Binding the callback function
        &CartesianImpedanceController::desiredCartesianCallback, // The callback function
        this,                             // Pointer to the current instance of the controller
        std::placeholders::_1             // Placeholder for the incoming message
      )
    );

    param_callback_handle_ = get_node()->add_on_set_parameters_callback(
      std::bind(&CartesianImpedanceController::parameter_callback, this, std::placeholders::_1));

  
  } catch (const std::exception& e) {
    // Error handling: If any exception is thrown during initialization, it will print the error message
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    
    // Returning an ERROR state if initialization fails
    return CallbackReturn::ERROR;
  }

  // If initialization is successful, return SUCCESS
  return CallbackReturn::SUCCESS;
}


// Configuration function for the Cartesian Impedance Controller
CallbackReturn CartesianImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {

  // Retrieving the value of the "arm_id" parameter (previously declared in on_init)
  arm_id_ = get_node()->get_parameter("arm_id").as_string();

  // Retrieving the value of the "pos_stiff" (position stiffness) parameter
  pos_stiff = get_node()->get_parameter("pos_stiff").as_double();

  // Retrieving the value of the "rot_stiff" (rotation stiffness) parameter
  rot_stiff = get_node()->get_parameter("rot_stiff").as_double();


  
  // Initializing the Franka Robot Model (a specialized class for handling the robot model in the controller)
  // This uses the robot model defined by the parameter "arm_id_/robot_model"
  // This is essential for calculating dynamics, Jacobians, and other state data.
  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(
          arm_id_ + "/robot_model",  // Constructing the model namespace (e.g., "panda/robot_model")
          arm_id_                    // Identifying the arm (e.g., "panda")
      ));

  // Listing the first 10 parameters of this node (mostly for debugging or verification)
  auto parameters = get_node()->list_parameters({}, 10);

  // Indicating that configuration was successful
  return CallbackReturn::SUCCESS;
}


// Activation function for the Cartesian Impedance Controller
CallbackReturn CartesianImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {

  // Assigns the state interfaces from the Franka Robot Model to this controller
  // This means the controller now has access to the robot's state information
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  // Initializes the desired end-effector pose using the current pose of the robot
  // This means the robot will start controlling from its current position
  desired = Matrix4d(franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector).data());

  // Extracting the position (XYZ) from the 4x4 pose matrix of the end-effector
  desired_position = Vector3d(desired.block<3,1>(0,3));

  // Extracting the orientation (rotation matrix) from the 4x4 pose matrix
  // Converting it to a quaternion for efficient calculations
  desired_orientation = Quaterniond(desired.block<3,3>(0,0));

  // Storing the initial joint angles (7 joints for the Franka Panda) as the desired joint configuration
  desired_qn = Vector7d(franka_robot_model_->getRobotState()->q.data());

  // Setting the stiffness matrix (controls how rigid the system is in response to external forces)
  // This is a 6x6 matrix:
  // - Top-left 3x3 for position stiffness (X, Y, Z).
  // - Bottom-right 3x3 for orientation stiffness (Roll, Pitch, Yaw).
  stiffness.setIdentity(); // Start with an identity matrix (all diagonal 1s)
  stiffness.topLeftCorner(3, 3) << pos_stiff * Matrix3d::Identity();   // Position stiffness (from parameter)
  stiffness.bottomRightCorner(3, 3) << rot_stiff * Matrix3d::Identity(); // Orientation stiffness (from parameter)

  // Setting the damping matrix (controls how much the system resists motion)
  // This is also a 6x6 matrix:
  // - Top-left 3x3 for position damping.
  // - Bottom-right 3x3 for orientation damping.
  damping.setIdentity();
  damping.topLeftCorner(3, 3) << 2 * sqrt(pos_stiff) * Matrix3d::Identity();   // Position damping (critical damping)
  damping.bottomRightCorner(3, 3) << 0.8 * 2 * sqrt(rot_stiff) * Matrix3d::Identity(); // Orientation damping (slightly underdamped)

  // Setting the nullspace stiffness (affects how the robot behaves in redundant joints)
  // Affects how the arm behaves while still following the end-effector target
  n_stiffness = 10.0;

  // Indicating that activation was successful
  return CallbackReturn::SUCCESS;
}


CallbackReturn CartesianImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/){
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

rcl_interfaces::msg::SetParametersResult CartesianImpedanceController::parameter_callback(
    const std::vector<rclcpp::Parameter>& parameters) {
  
  fprintf(stderr, "Parameter callback called with %zu parameters.\n", parameters.size());
  for (const auto& param : parameters) {
    if (param.get_name() == "pos_stiff") {
      pos_stiff = param.as_double();
      stiffness.topLeftCorner(3, 3) = pos_stiff * Matrix3d::Identity();
      damping.topLeftCorner(3, 3) = 2 * sqrt(pos_stiff) * Matrix3d::Identity();
    }
    if (param.get_name() == "rot_stiff") {
      rot_stiff = param.as_double();
      stiffness.bottomRightCorner(3, 3) = rot_stiff * Matrix3d::Identity();
      damping.bottomRightCorner(3, 3) = 0.8 * 2 * sqrt(rot_stiff) * Matrix3d::Identity();
    }
  }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}


// This function is the callback that is triggered whenever a new message is published
// to the /cartesian_impedance/pose_desired topic. It updates the target position
// and orientation for the end-effector.

void CartesianImpedanceController::desiredCartesianCallback(
    const std_msgs::msg::Float64MultiArray& msg) {

  // Check if the incoming message has valid data (non-zero at the first index)
  if (msg.data[0]) {

    // Update the desired position (X, Y, Z) of the end-effector
    for (auto i = 0; i < 3; ++i) {
      desired_position[i] = msg.data[i]; // msg.data[0] -> X, msg.data[1] -> Y, msg.data[2] -> Z
    }

    // If the message contains an orientation matrix (starting at index 11)
    if (msg.data[11]) { // Check if the 12th value is present (orientation indicator)

      // Create a 3x3 matrix to store the desired orientation
      Matrix3d desired_orientation_mat;

      // Populate the 3x3 matrix using the incoming message data (row by row)
      for (auto i = 0; i < 3; ++i) {        // Loop over rows
        for (auto j = 0; j < 3; ++j) {      // Loop over columns
          // msg.data[3 + 3*i + j] maps the flat array into a 3x3 grid
          desired_orientation_mat(i, j) = msg.data[3 + 3 * i + j];
        }
      }

      // Convert the 3x3 orientation matrix into a quaternion representation
      // This is more efficient for calculations and avoids gimbal lock issues
      desired_orientation = Eigen::Quaterniond(desired_orientation_mat);
    }
  }
}


}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianImpedanceController,
                       controller_interface::ControllerInterface)
