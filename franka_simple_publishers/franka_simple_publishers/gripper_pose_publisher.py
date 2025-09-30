import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class GripperPosePublisher(Node):
    def __init__(self):
        super().__init__("gripper_pose_publisher")

        # Parameters (can be set via launch file or command line)
        self.declare_parameter("pub_frequency", 50.0)  # Hz
        self.declare_parameter("f_sin", 1.0)  # Hz (sinusoid frequency)

        publish_frequency = self.get_parameter("pub_frequency").value
        self.f_sin = self.get_parameter("f_sin").value

        # Publisher
        self.publisher_ = self.create_publisher(Float64, "/gripper/command", 10)

        # Timer for publishing
        timer_period = 1.0 / publish_frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Internal state
        self.start_time = time.time()

        self.get_logger().info(
            f"Publishing sinusoid at {publish_frequency:.2f} Hz, "
            f"sinusoid freq = {self.f_sin} Hz"
        )

    def timer_callback(self):
        elapsed = time.time() - self.start_time

        # Sinusoid
        A_max = 0.076 # [m]
        A_min = 0.0 # [m]
        A = (A_max - A_min)/2
        offset = A_max - A # center

        value = offset + A * math.sin(2.0 * math.pi * self.f_sin * elapsed)

        msg = Float64()
        msg.data = value
        self.publisher_.publish(msg)

        # self.get_logger().debug(f"Publishing: {value:.6f}")


def main(args=None):
    rclpy.init(args=args)
    node = GripperPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
