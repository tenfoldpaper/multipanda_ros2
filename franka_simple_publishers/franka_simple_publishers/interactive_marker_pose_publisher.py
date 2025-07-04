#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from geometry_msgs.msg import PoseStamped

from franka_msgs.action import Grasp
from franka_msgs.msg import GraspEpsilon

from tf2_ros import TransformListener, Buffer, LookupException, TimeoutException


class EndEffectorMarkerNode(Node):
    def __init__(self):
        super().__init__('ee_interactive_marker_node')

        # Declare parameters and get their values
        self.declare_parameter('topic_name', 'cartesian_impedance/equilibrium_pose')
        self.declare_parameter('base_link', 'panda_link0')
        self.declare_parameter('ee_link', 'panda_hand_tcp')

        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.base_link = self.get_parameter('base_link').get_parameter_value().string_value
        self.ee_link = self.get_parameter('ee_link').get_parameter_value().string_value

        self.get_logger().info(f"Publishing to topic: {self.topic_name}")
        self.get_logger().info(f"Base link: {self.base_link}, EE link: {self.ee_link}")

        # Callback group for concurrent handling
        self.callback_group = ReentrantCallbackGroup()

        # Publisher
        self.pose_pub = self.create_publisher(
            PoseStamped, self.topic_name, 10, callback_group=self.callback_group
        )

        # Action client

        self.grasp_client = ActionClient(self, Grasp, '/panda_gripper/grasp')
        
        self.grasp_goal_close = Grasp.Goal()
        self.grasp_goal_close.width = 0.0
        self.grasp_goal_close.speed = 0.1
        self.grasp_goal_close.force = 40.0
        self.grasp_goal_close.epsilon = GraspEpsilon()
        self.grasp_goal_close.epsilon.inner = 0.05
        self.grasp_goal_close.epsilon.outer = 0.05

        self.grasp_goal_open = Grasp.Goal()
        self.grasp_goal_open.width = 0.02
        self.grasp_goal_open.speed = 0.1
        self.grasp_goal_open.force = 10.0
        self.grasp_goal_open.epsilon = GraspEpsilon()
        self.grasp_goal_open.epsilon.inner = 0.05
        self.grasp_goal_open.epsilon.outer = 0.05

        self.grasp_available = self.grasp_client.wait_for_server(timeout_sec=5.0)
        if not self.grasp_available:
            self.get_logger().warn(f"Grasp action /panda_gripper/grasp not available!")
            

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Interactive marker server
        self.server = InteractiveMarkerServer(self, 'ee_marker_server')

        self.menu = MenuHandler()
        self.reinit_entry = self.menu.insert("Reset Marker", callback=self.handle_menu_feedback)
        self.grasp_entry = self.menu.insert("Close Gripper", callback=self.handle_menu_feedback)
        self.release_entry = self.menu.insert("Open Gripper", callback=self.handle_menu_feedback)

        self.initialized = False
        self.create_timer(0.5, self.try_initialize_marker, callback_group=self.callback_group)

    def try_initialize_marker(self):
        if self.initialized:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_link, self.ee_link, rclpy.time.Time()
            )
            self.get_logger().info('TF transform acquired, initializing marker')
            self.create_interactive_marker(tf)
            self.initialized = True
        except (LookupException, TimeoutException):
            self.get_logger().warn(f'Waiting for TF from {self.base_link} to {self.ee_link}...')

    def try_reinitialize_marker(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_link, self.ee_link, rclpy.time.Time()
            )
            self.server.clear()
            self.create_interactive_marker(tf)
            # self.get_logger().info("Marker reinitialized.")
        except (LookupException, TimeoutException):
            self.get_logger().warn("Could not reinitialize marker: TF not available.")

    def create_interactive_marker(self, tf):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.base_link
        int_marker.name = 'ee_target'
        int_marker.description = 'Equilibrium Pose'
        int_marker.scale = 0.2

        # Set position and orientation
        int_marker.pose.position.x = tf.transform.translation.x
        int_marker.pose.position.y = tf.transform.translation.y
        int_marker.pose.position.z = tf.transform.translation.z
        int_marker.pose.orientation = tf.transform.rotation

        self.add_6dof_controls(int_marker)

        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.MENU
        menu_control.name = "menu"
        int_marker.controls.append(menu_control)

        self.server.insert(int_marker, feedback_callback=self.process_feedback)
        self.menu.apply(self.server, int_marker.name)
        self.server.applyChanges()

    def add_6dof_controls(self, marker):
        control_axes = [
            ('move_x', InteractiveMarkerControl.MOVE_AXIS, 1.0, 0.0, 0.0),
            ('move_y', InteractiveMarkerControl.MOVE_AXIS, 0.0, 1.0, 0.0),
            ('move_z', InteractiveMarkerControl.MOVE_AXIS, 0.0, 0.0, 1.0),
            ('rotate_x', InteractiveMarkerControl.ROTATE_AXIS, 1.0, 0.0, 0.0),
            ('rotate_y', InteractiveMarkerControl.ROTATE_AXIS, 0.0, 1.0, 0.0),
            ('rotate_z', InteractiveMarkerControl.ROTATE_AXIS, 0.0, 0.0, 1.0),
        ]

        for name, interaction_mode, x, y, z in control_axes:
            control = InteractiveMarkerControl()
            control.name = name
            control.interaction_mode = interaction_mode
            control.orientation.w = 1.0
            control.orientation.x = x
            control.orientation.y = y
            control.orientation.z = z
            marker.controls.append(control)

    def process_feedback(self, feedback: InteractiveMarkerFeedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            pose_stamped = PoseStamped()
            pose_stamped.header = feedback.header
            pose_stamped.pose = feedback.pose
            self.pose_pub.publish(pose_stamped)
            # self.get_logger().info(f"Published EE reference pose: {pose_stamped.pose}")

    def handle_menu_feedback(self, feedback: InteractiveMarkerFeedback):
        if feedback.menu_entry_id == self.reinit_entry:
            # self.get_logger().info("Reinitializing marker at current EE pose")
            self.try_reinitialize_marker()
        if feedback.menu_entry_id == self.grasp_entry:       
            if self.grasp_available:
                # self.get_logger().info("Sending grasp command to the gripper")
                self.grasp_client.send_goal_async(self.grasp_goal_close)
            else:
                self.get_logger().warn(f"Grasp action /panda_gripper/grasp not available!")
        if feedback.menu_entry_id == self.release_entry: 
            if self.grasp_available:
                # self.get_logger().info("Sending release command to the gripper")   
                self.grasp_client.send_goal_async(self.grasp_goal_open)
            else:
                self.get_logger().warn(f"Grasp action /panda_gripper/grasp not available!")


def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorMarkerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
