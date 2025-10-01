#include "franka_gripper_custom/franka_gripper_control.hpp"

GripperSubscriber::GripperSubscriber() : Node("panda_gripper") {
  // Declare parameters
  this->declare_parameter("robot_ip", "176.16.0.1");
  this->declare_parameter("joint_names",
                          std::vector<std::string>{"panda_finger_joint1", "panda_finger_joint2"});
  this->declare_parameter("default_gripper_width", 0.01);
  this->declare_parameter("maximum_gripper_width", 0.076);
  this->declare_parameter("default_gripper_speed", 1.0);
  this->declare_parameter("gripper_max_effort", 100.0);   // [N]
  this->declare_parameter("default_epsilon_inner", 0.1);  // [m]
  this->declare_parameter("default_epsilon_outer", 0.1);  // [m]
  this->declare_parameter("pub_frequency", 50);           // actually limited to 15 Hz

  // Get parameters
  robot_ip_ = this->get_parameter("robot_ip").as_string();
  joint_names_ = this->get_parameter("joint_names").as_string_array();
  default_width_ = this->get_parameter("default_gripper_width").as_double();
  maximum_width_ = this->get_parameter("maximum_gripper_width").as_double();
  default_speed_ = this->get_parameter("default_gripper_speed").as_double();
  gripper_max_effort_ = this->get_parameter("gripper_max_effort").as_double();
  default_epsilon_inner_ = this->get_parameter("default_epsilon_inner").as_double();
  default_epsilon_outer_ = this->get_parameter("default_epsilon_outer").as_double();
  pub_frequency_ = this->get_parameter("pub_frequency").as_int();

  try {
    gripper_ = std::make_unique<franka::Gripper>(robot_ip_);
    RCLCPP_INFO(this->get_logger(), "Connected to gripper at %s", robot_ip_.c_str());
  } catch (const franka::Exception& e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to connect: %s", e.what());
    throw;
  }

  // Subscriber
  command_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "~/gripper_command", 1,
      std::bind(&GripperSubscriber::commandCallback, this, std::placeholders::_1));

  // Publishers
  joint_state_pub_ =
      this->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);  // joint states
  width_pub_ = this->create_publisher<std_msgs::msg::Float64>("~/width", 10);      // gripper width

  // Timer for publishing state
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / pub_frequency_),
                                   std::bind(&GripperSubscriber::publishGripperState, this));

  // init gripper memory state
  gripper_->homing();
  gripper_->move(maximum_width_, default_speed_);  // maximum opening
  command_data_bool_prev = false;
}

void GripperSubscriber::commandCallback(const std_msgs::msg::Float64::SharedPtr msg) {
  // get data
  double command_data = msg->data;
  bool command_data_bool = (command_data >= 0.5);

  // decide whether to open or close
  if (command_data_bool != command_data_bool_prev) {
    if (command_data_bool == false)
      try {
        std::lock_guard<std::mutex> lock(gripper_state_mutex_);
        if (!gripper_->move(maximum_width_, default_speed_)) {  // maximum opening
          RCLCPP_WARN(this->get_logger(), "Gripper opening failed");
        }
      } catch (const franka::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Gripper opening command failed: %s", e.what());
      }
    else {
      try {
        std::lock_guard<std::mutex> lock(gripper_state_mutex_);
        if (!gripper_->grasp(default_width_, default_speed_, gripper_max_effort_,  // grasping
                             default_epsilon_inner_, default_epsilon_outer_)) {
          RCLCPP_WARN(this->get_logger(), "Gripper grasping failed");
        }
      } catch (const franka::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Gripper grasping command failed: %s", e.what());
      }
    }
  }
  command_data_bool_prev = command_data_bool;
}

void GripperSubscriber::publishGripperState() {
  try {
    std::lock_guard<std::mutex> lock(gripper_state_mutex_);
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

  // Publish width
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
