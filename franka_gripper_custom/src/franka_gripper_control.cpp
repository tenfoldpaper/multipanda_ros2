#include "franka_gripper_custom/franka_gripper_control.hpp"

GripperSubscriber::GripperSubscriber() : Node("panda_gripper") {
  // Declare parameters
  this->declare_parameter("robot_ip", "176.16.0.1");
  this->declare_parameter("default_gripper_speed", 0.1);
  this->declare_parameter("joint_names",
                          std::vector<std::string>{"panda_finger_joint1", "panda_finger_joint2"});
  this->declare_parameter("pub_frequency", 50);            // Hz
  this->declare_parameter("gripper_max_effort", 50.0);     // [N]
  this->declare_parameter("default_epsilon_inner", 0.01);  // [m]
  this->declare_parameter("default_epsilon_outer", 0.00);  // [m]

  robot_ip_ = this->get_parameter("robot_ip").as_string();
  default_speed_ = this->get_parameter("default_gripper_speed").as_double();
  joint_names_ = this->get_parameter("joint_names").as_string_array();
  pub_frequency_ = this->get_parameter("pub_frequency").as_int();
  gripper_max_effort_ = this->get_parameter("gripper_max_effort").as_double();
  default_epsilon_inner_ = this->get_parameter("default_epsilon_inner").as_double();
  default_epsilon_outer_ = this->get_parameter("default_epsilon_outer").as_double();

  try {
    gripper_ = std::make_unique<franka::Gripper>(robot_ip_);
    RCLCPP_INFO(this->get_logger(), "Connected to gripper at %s", robot_ip_.c_str());
  } catch (const franka::Exception& e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to connect: %s", e.what());
    throw;
  }

  // Subscriber
  command_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/gripper/command", 1,
      std::bind(&GripperSubscriber::commandCallback, this, std::placeholders::_1));

  stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/stop_sub", 1, std::bind(&GripperSubscriber::stopCallback, this, std::placeholders::_1));

  // Publishers
  joint_state_pub_ =
      this->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);  // joint states
  width_pub_ = this->create_publisher<std_msgs::msg::Float64>("~/width", 10);      // gripper width

  // // Timer for publishing state
  // timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / pub_frequency_),
  //                                  std::bind(&GripperSubscriber::publishGripperState, this));

  RCLCPP_INFO(this->get_logger(), "Listening on /gripper/command");
}

void GripperSubscriber::commandCallback(const std_msgs::msg::Float64::SharedPtr msg) {
  double target_width = msg->data;

  // RCLCPP_INFO(this->get_logger(), "Received gripper command: width = %.3f m", target_width);

  try {
    if (!gripper_->move(target_width, default_speed_)) {
      RCLCPP_WARN(this->get_logger(), "Gripper move to %.3f m failed", target_width);
    }
  } catch (const franka::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Gripper command failed: %s", e.what());
  }
}

void GripperSubscriber::stopCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  bool stop = msg->data;

  // RCLCPP_INFO(this->get_logger(), "Received gripper command: width = %.3f m", target_width);

  if (stop == true) {
    gripper_->stop();
    RCLCPP_INFO(this->get_logger(), "Stop command received");
  }
}

// void GripperSubscriber::commandCallback(const std_msgs::msg::Float64::SharedPtr msg) {
//   double target_width = msg->data;

//   // RCLCPP_INFO(this->get_logger(), "Received gripper command: width = %.3f m", target_width);

//   try {
//     if (!gripper_->grasp(target_width, default_speed_, gripper_max_effort_,
//     default_epsilon_inner_,
//                          default_epsilon_outer_)) {
//       RCLCPP_WARN(this->get_logger(), "Gripper move to %.3f m failed", target_width);
//     }
//   } catch (const franka::Exception& e) {
//     RCLCPP_ERROR(this->get_logger(), "Gripper command failed: %s", e.what());
//   }
// }

void GripperSubscriber::publishGripperState() {
  std::lock_guard<std::mutex> lock(gripper_state_mutex_);
  try {
    current_gripper_state_ = gripper_->readOnce();
  } catch (const franka::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
  }
  sensor_msgs::msg::JointState joint_states;
  joint_states.header.stamp = this->now();
  joint_states.name.push_back(this->joint_names_[0]);
  joint_states.name.push_back(this->joint_names_[1]);
  joint_states.position.push_back(current_gripper_state_.width / 2);
  joint_states.position.push_back(current_gripper_state_.width / 2);
  joint_states.velocity.push_back(0.0);
  joint_states.velocity.push_back(0.0);
  joint_states.effort.push_back(0.0);
  joint_states.effort.push_back(0.0);
  joint_state_pub_->publish(joint_states);

  // Publish width as Float64
  std_msgs::msg::Float64 width_msg;
  width_msg.data = current_gripper_state_.width;
  width_pub_->publish(width_msg);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
