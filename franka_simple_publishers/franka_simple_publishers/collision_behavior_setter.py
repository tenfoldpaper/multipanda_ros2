import sys

import rclpy
from rclpy.node import Node

from franka_msgs.srv import SetForceTorqueCollisionBehavior

class SetCollBehaviorClientAsync(Node):

    def __init__(self):
        super().__init__('set_coll_behavior_client_async')

        # Declare parameters and get their values
        self.declare_parameter('T_lb', 100.0)
        self.declare_parameter('T_ub', 100.0)
        self.declare_parameter('F_lb', 100.0)
        self.declare_parameter('F_ub', 100.0)
        self.T_lb = self.get_parameter('T_lb').get_parameter_value().double_value
        self.T_ub = self.get_parameter('T_ub').get_parameter_value().double_value
        self.F_lb = self.get_parameter('F_lb').get_parameter_value().double_value
        self.F_ub = self.get_parameter('F_ub').get_parameter_value().double_value

        self.cli = self.create_client(
            SetForceTorqueCollisionBehavior, 
            '/panda_param_service_server/set_force_torque_collision_behavior'
        )
        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetForceTorqueCollisionBehavior.Request()

    def send_request(self):
        lower_torque_thresholds_nominal = [self.T_lb, self.T_lb, self.T_lb, self.T_lb, self.T_lb, self.T_lb, self.T_lb]
        upper_torque_thresholds_nominal = [self.T_ub, self.T_ub, self.T_ub, self.T_ub, self.T_ub, self.T_ub, self.T_ub]
        lower_force_thresholds_nominal = [self.F_lb, self.F_lb, self.F_lb, self.F_lb, self.F_lb, self.F_lb]
        upper_force_thresholds_nominal = [self.F_ub, self.F_ub, self.F_ub, self.F_ub, self.F_ub, self.F_ub]

        self.get_logger().info(f"Setting 'lower_torque_thresholds_nominal' to {lower_torque_thresholds_nominal}")
        self.get_logger().info(f"Setting 'upper_torque_thresholds_nominal' to {lower_torque_thresholds_nominal}")
        self.get_logger().info(f"Setting 'lower_force_thresholds_nominal' to {lower_torque_thresholds_nominal}")
        self.get_logger().info(f"Setting 'upper_force_thresholds_nominal' to {lower_torque_thresholds_nominal}")

        self.req.lower_torque_thresholds_nominal = lower_torque_thresholds_nominal
        self.req.upper_torque_thresholds_nominal = upper_torque_thresholds_nominal
        self.req.lower_force_thresholds_nominal = lower_force_thresholds_nominal
        self.req.upper_force_thresholds_nominal = upper_force_thresholds_nominal
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    client = SetCollBehaviorClientAsync()
    future = client.send_request()
    rclpy.spin_until_future_complete(client, future)
    response = future.result()
    client.get_logger().info(f"Success: {response.success}")
    client.get_logger().info(f"Error: {response.error}")

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()