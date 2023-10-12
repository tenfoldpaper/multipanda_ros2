
# Full collision behaviour
ros2 service call /param_service_server/set_full_collision_behavior franka_msgs/srv/SetFullCollisionBehavior "{
  lower_torque_thresholds_acceleration: [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0], 
  upper_torque_thresholds_acceleration:[20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],  
  lower_torque_thresholds_nominal: [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0], 
  upper_torque_thresholds_nominal: [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],  
  lower_force_thresholds_acceleration: [20.0, 20.0, 20.0, 25.0, 25.0, 25.0], 
  upper_force_thresholds_acceleration: [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],  
  lower_force_thresholds_nominal: [20.0, 20.0, 20.0, 25.0, 25.0, 25.0], 
  upper_force_thresholds_nominal: [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]}"

# Joint stiffness
ros2 service call /param_service_server/set_joint_stiffness franka_msgs/srv/SetJointStiffness "{
  joint_stiffness: [3000, 3000, 3000, 2500, 2500, 2000, 2000]
}"

