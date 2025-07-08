import time
import os
import yaml
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import ament_index_python

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose
from control_msgs.action import GripperCommand
from franka_msgs.action import Grasp
from franka_msgs.msg import GraspEpsilon

from tf2_ros import TransformListener, Buffer, LookupException, TimeoutException

import numpy as np
from scipy.interpolate import CubicSpline, interp1d

class AssistanceExperimentNode(Node):
    def __init__(self):
        super().__init__('assistance_experiment_node')

        self.eq_pose_topic = 'cartesian_impedance/equilibrium_pose'
        self.base_link = 'panda_link0'
        self.ee_link = 'panda_hand_tcp'

        # Callback group for concurrent handling
        self.callback_group = ReentrantCallbackGroup()

        # Publisher
        self.pose_pub = self.create_publisher(
            PoseStamped, self.eq_pose_topic, 1, callback_group=self.callback_group
        )

        # Gripper action clients
        self.grasp_cli = ActionClient(self, Grasp, '/panda_gripper/grasp', callback_group=self.callback_group)
        while not self.grasp_cli.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(f"{self.grasp_cli._action_name} not available, waiting again...")
        self.grasp_msg = Grasp.Goal()
        self.grasp_msg.width = 0.010
        self.grasp_msg.speed = 1.0
        self.grasp_msg.force = 100.0
        self.grasp_msg.epsilon = GraspEpsilon()
        self.grasp_msg.epsilon.inner = 0.025
        self.grasp_msg.epsilon.outer = 0.025

        self.gripper_cli = ActionClient(self, GripperCommand, '/panda_gripper/gripper_action', callback_group=self.callback_group)
        while not self.gripper_cli.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(f"{self.gripper_cli._action_name} not available, waiting again...")
        self.open_grip_msg = GripperCommand.Goal()
        self.open_grip_msg.command.position = 0.038
        self.open_grip_msg.command.max_effort = 0.0

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Open config file
        cfg_path = os.path.join(
            ament_index_python.packages.get_package_share_directory('franka_simple_publishers'), 
            'config',
            'assistance_experiment_cfg.yaml',
        )
        with open(cfg_path) as stream:
            try:
                cfg_dict = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                self.get_logger().error(exc)
                sys.exit(0)

        # Init action done flag
        self.action_done = False

        # Load target poses
        self.poses_dict = cfg_dict["poses"]

        # Load motion params (times, offeset)
        self.interp_dt = cfg_dict["interp_dt"]
        self.go_to_pre_pick_time = cfg_dict["go_to_pre_pick_time"]
        self.go_to_grasp_time = cfg_dict["go_to_grasp_time"]
        self.go_to_pre_place_time = cfg_dict["go_to_pre_place_time"]
        self.go_to_place_time = cfg_dict["go_to_place_time"]
        self.go_to_home_time = cfg_dict["go_to_home_time"]
        self.z_offset = cfg_dict["z_offset"]

        # Homing
        self.get_logger().info("Homing")
        self.home_pose_msg = Pose()
        self.home_pose_msg.position.x = self.poses_dict["home"]["position"]["x"]
        self.home_pose_msg.position.y = self.poses_dict["home"]["position"]["y"]
        self.home_pose_msg.position.z = self.poses_dict["home"]["position"]["z"]
        self.home_pose_msg.orientation.x = self.poses_dict["home"]["orientation"]["x"]
        self.home_pose_msg.orientation.y = self.poses_dict["home"]["orientation"]["y"]
        self.home_pose_msg.orientation.z = self.poses_dict["home"]["orientation"]["z"]
        self.home_pose_msg.orientation.w = self.poses_dict["home"]["orientation"]["w"]
        self.execute_trajectory(self.home_pose_msg, self.go_to_home_time, dt=self.interp_dt)

        # Open gripper
        self.release()

        # Subscribe to trigger topic
        self.idle = True 
        self.available_tasks = ["bar", "screwdriver", "tape"]
        self.trigger_sub = self.create_subscription(
            String,
            'trigger_task',
            self.task_trigger_cb,
            1,
            callback_group=self.callback_group
        )

        self.get_logger().info("Assistance Task Node Ready")


    def task_trigger_cb(self, msg):
        if self.idle:
            if msg.data in self.available_tasks:
                self.get_logger().info(f"Executing task: {msg.data}")
                self.idle = True
                self.execute(msg.data)
                self.get_logger().info(f"Task {msg.data} completed")
            else:
                self.get_logger().warn(f"Requested task {msg.data} not available!")


    def release(self):
        self.action_done = False
        send_goal_future = self.gripper_cli.send_goal_async(self.open_grip_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        while not self.action_done:
            time.sleep(0.1)
        self.get_logger().info('Release done')


    def grasp(self):
        self.action_done = False
        send_goal_future = self.grasp_cli.send_goal_async(self.grasp_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        while not self.action_done:
            time.sleep(0.1)
        self.get_logger().info('Grasp done')


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            # self.get_logger().warn('Goal rejected')
            return
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)


    def result_callback(self, future):
        result = future.result().result
        # self.get_logger().info(f'Result: {result}')
        self.action_done = True


    def execute(self, task_name: str):

        # Get grasp pose
        grasp_pose_dict = self.poses_dict[f"{task_name}_grasp"]
        grasp_pose_msg = Pose()
        grasp_pose_msg.position.x = grasp_pose_dict["position"]["x"]
        grasp_pose_msg.position.y = grasp_pose_dict["position"]["y"]
        grasp_pose_msg.position.z = grasp_pose_dict["position"]["z"]
        grasp_pose_msg.orientation.x = grasp_pose_dict["orientation"]["x"]
        grasp_pose_msg.orientation.y = grasp_pose_dict["orientation"]["y"]
        grasp_pose_msg.orientation.z = grasp_pose_dict["orientation"]["z"]
        grasp_pose_msg.orientation.w = grasp_pose_dict["orientation"]["w"]

        # Compute pre-pick pose
        pre_pick_pose_msg = Pose()
        pre_pick_pose_msg.position.x = grasp_pose_dict["position"]["x"]
        pre_pick_pose_msg.position.y = grasp_pose_dict["position"]["y"]
        pre_pick_pose_msg.position.z = grasp_pose_dict["position"]["z"] + self.z_offset
        pre_pick_pose_msg.orientation.x = grasp_pose_dict["orientation"]["x"]
        pre_pick_pose_msg.orientation.y = grasp_pose_dict["orientation"]["y"]
        pre_pick_pose_msg.orientation.z = grasp_pose_dict["orientation"]["z"]
        pre_pick_pose_msg.orientation.w = grasp_pose_dict["orientation"]["w"]

        # Get place pose
        place_pose_dict = self.poses_dict[f"{task_name}_place"]
        place_pose_msg = Pose()
        place_pose_msg.position.x = place_pose_dict["position"]["x"]
        place_pose_msg.position.y = place_pose_dict["position"]["y"]
        place_pose_msg.position.z = place_pose_dict["position"]["z"]
        place_pose_msg.orientation.x = place_pose_dict["orientation"]["x"]
        place_pose_msg.orientation.y = place_pose_dict["orientation"]["y"]
        place_pose_msg.orientation.z = place_pose_dict["orientation"]["z"]
        place_pose_msg.orientation.w = place_pose_dict["orientation"]["w"]

        # Compute pre-place pose
        pre_place_pose_dict = self.poses_dict[f"{task_name}_place"]
        pre_place_pose_msg = Pose()
        pre_place_pose_msg.position.x = pre_place_pose_dict["position"]["x"]
        pre_place_pose_msg.position.y = pre_place_pose_dict["position"]["y"]
        pre_place_pose_msg.position.z = pre_place_pose_dict["position"]["z"] + self.z_offset
        pre_place_pose_msg.orientation.x = pre_place_pose_dict["orientation"]["x"]
        pre_place_pose_msg.orientation.y = pre_place_pose_dict["orientation"]["y"]
        pre_place_pose_msg.orientation.z = pre_place_pose_dict["orientation"]["z"]
        pre_place_pose_msg.orientation.w = pre_place_pose_dict["orientation"]["w"]

        # Get home pose
        home_pose_dict = self.poses_dict["home"]
        home_pose_msg = Pose()
        home_pose_msg.position.x = home_pose_dict["position"]["x"]
        home_pose_msg.position.y = home_pose_dict["position"]["y"]
        home_pose_msg.position.z = home_pose_dict["position"]["z"]
        home_pose_msg.orientation.x = home_pose_dict["orientation"]["x"]
        home_pose_msg.orientation.y = home_pose_dict["orientation"]["y"]
        home_pose_msg.orientation.z = home_pose_dict["orientation"]["z"]
        home_pose_msg.orientation.w = home_pose_dict["orientation"]["w"]

        # Send commands
        self.execute_trajectory(pre_pick_pose_msg, self.go_to_pre_pick_time, dt=self.interp_dt)
        self.execute_trajectory(grasp_pose_msg, self.go_to_grasp_time, dt=self.interp_dt)
        self.grasp()
        self.execute_trajectory(pre_pick_pose_msg, self.go_to_grasp_time, dt=self.interp_dt)

        self.execute_trajectory(pre_place_pose_msg, self.go_to_pre_place_time, dt=self.interp_dt)
        self.execute_trajectory(place_pose_msg, self.go_to_place_time, dt=self.interp_dt)
        self.release()
        self.execute_trajectory(pre_place_pose_msg, self.go_to_place_time, dt=self.interp_dt)
        self.execute_trajectory(home_pose_msg, self.go_to_home_time, dt=self.interp_dt)


    def set_eq_pose(self, goal: Pose, t_wait: float):
        goal_msg = PoseStamped()
        goal_msg.pose = goal
        self.pose_pub.publish(goal_msg)
        time.sleep(t_wait)


    def execute_trajectory(self, goal: Pose, duration: float, dt:float=0.1, interp: str = "spline"):
        start = self.get_curr_ee_pose()
        values = np.array(
            [
                [start.position.x, start.position.y, start.position.z], 
                [goal.position.x, goal.position.y, goal.position.z], 
            ]
        )
        if interp == "spline":
            interpolator = CubicSpline(np.array([0.0, duration]), values)
        elif interp == "linear":
            interpolator = interp1d(np.array([0.0, duration]), values, axis=0)
        else:
            self.get_logger().warn(f"Unknown requested interpolator {interp}. Falling back to 'spline'")
            interpolator = CubicSpline(np.array([0.0, duration]), values)

        steps = np.arange(0.0, duration + dt, dt)
        traj = interpolator(steps)

        for i in range(traj.shape[0]):
            msg = PoseStamped()
            msg.pose.orientation = goal.orientation  # keep fixed orientation goal
            msg.pose.position.x = traj[i, 0]
            msg.pose.position.y = traj[i, 1]
            msg.pose.position.z = traj[i, 2]
            self.pose_pub.publish(msg)
            time.sleep(dt)
        msg = PoseStamped()
        msg.pose = goal
        self.pose_pub.publish(msg)
        time.sleep(dt)


    def get_curr_ee_pose(self):
        while True:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.base_link, self.ee_link, rclpy.time.Time()
                )
                self.get_logger().info(f"TF transform '{self.base_link}' to '{self.ee_link}' acquired")
                pose = Pose()
                pose.position.x = tf.transform.translation.x
                pose.position.y = tf.transform.translation.y
                pose.position.z = tf.transform.translation.z
                pose.orientation.x = tf.transform.rotation.x
                pose.orientation.y = tf.transform.rotation.y
                pose.orientation.z = tf.transform.rotation.z
                pose.orientation.w = tf.transform.rotation.w
                return pose
            except (LookupException, TimeoutException):
                self.get_logger().warn(f'Waiting for TF from {self.base_link} to {self.ee_link}...')
                time.sleep(5.0)


def main():
    rclpy.init()
    node = AssistanceExperimentNode()

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
