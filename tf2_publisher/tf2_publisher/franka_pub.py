import sys

from geometry_msgs.msg import PoseStamped, Point, Quaternion

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener



class FrankaPublisher(Node):

    def __init__(self):
        super().__init__('tf_translator')

        # Get frames to translate position to/from
        self.target_frame = self.declare_parameter(
            'target_frame', 'panda_link7').get_parameter_value().string_value

        self.base_frame = self.declare_parameter(
            'base_frame', 'panda_link0').get_parameter_value().string_value

        # Set the output topic
        self.pub_topic = self.declare_parameter(
            'topic', 'tf/pose').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create turtle2 velocity publisher
        self.publisher = self.create_publisher(PoseStamped, self.pub_topic, 1)

        # Log this creation
        self.get_logger().info(f'Publishing from {self.target_frame} to {self.base_frame} on topic {self.pub_topic}')

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel =  self.base_frame

        # Look up for the transformation between target_frame and turtle2 frames
        # and send velocity commands for turtle2 to reach target_frame
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        msg = PoseStamped()
        # Header data
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame
        # Get the pose first
        msg.pose.position.x = t.transform.translation.x
        msg.pose.position.y = t.transform.translation.y
        msg.pose.position.z = t.transform.translation.z
        # Now screw with the rotation
        msg.pose.orientation.x = t.transform.rotation.x
        msg.pose.orientation.y = t.transform.rotation.y
        msg.pose.orientation.z = t.transform.rotation.z
        msg.pose.orientation.w = t.transform.rotation.w

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = FrankaPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


main()