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

# from control_msgs.action import GripperCommand

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

        # # Action client (control_msgs.action.GripperCommand)
        # self.gripper_client = ActionClient(self, GripperCommand, '/panda_gripper/gripper_action')
        
        # self.gripper_goal_close = GripperCommand.Goal()
        # self.gripper_goal_close.command.position = 0.01 # 0.01 x tape and key, 0.015 x aluminum bar
        # self.gripper_goal_close.command.max_effort = 100.0
        # self.gripper_goal_open = GripperCommand.Goal()
        # self.gripper_goal_open.command.position = 0.038
        # self.gripper_goal_open.command.max_effort = 0.01

        # Action client (franka_msgs.action.Grasp)
        self.gripper_client = ActionClient(self, Grasp, '/panda_gripper/grasp')

        self.gripper_goal_close = Grasp.Goal()
        self.gripper_goal_close.width = 0.010
        self.gripper_goal_close.speed = 1.0
        self.gripper_goal_close.force = 100.0
        self.gripper_goal_close.epsilon = GraspEpsilon()
        self.gripper_goal_close.epsilon.inner = 0.025
        self.gripper_goal_close.epsilon.outer = 0.025

        self.gripper_goal_open = Grasp.Goal()
        self.gripper_goal_open.width = 0.038
        self.gripper_goal_open.speed = 1.0
        self.gripper_goal_open.force = 0.0
        self.gripper_goal_open.epsilon = GraspEpsilon()
        self.gripper_goal_open.epsilon.inner = 0.025
        self.gripper_goal_open.epsilon.outer = 0.025

        self.gripper_available = self.gripper_client.wait_for_server(timeout_sec=5.0)
        if not self.gripper_available:
            self.get_logger().warn(f"Grasp action {self.gripper_client._action_name} not available!")

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
            if self.gripper_available:
                # self.get_logger().info("Sending grasp command to the gripper")
                self.gripper_client.send_goal_async(self.gripper_goal_close)
            else:
                self.get_logger().warn(f"Grasp action {self.gripper_client._action_name} not available!")
        if feedback.menu_entry_id == self.release_entry: 
            if self.gripper_available:
                # self.get_logger().info("Sending release command to the gripper")   
                self.gripper_client.send_goal_async(self.gripper_goal_open)
            else:
                self.get_logger().warn(f"Grasp action {self.gripper_client._action_name} not available!")


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
