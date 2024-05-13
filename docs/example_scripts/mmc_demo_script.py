# Console menu imports
from consolemenu import *
from consolemenu.items import *

# Messages
from multi_mode_control_msgs.srv import SetControllers, GetRobotStates, SetCartesianImpedance
from multi_mode_control_msgs.msg import Controller
from panda_motion_generator_msgs.action import JointViaMotion, CartesianViaMotion
from panda_motion_generator_msgs.msg import JointPose
from geometry_msgs.msg import Pose

# rclpy stuff
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# System imports
import sys
import time
import csv
from copy import deepcopy

print("Imports working")

# Basic service and action clients
class SetControllerClient(Node):
    def __init__(self):
        super().__init__('set_controller_client_async')
        self.cli = self.create_client(SetControllers, '/set_controllers')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SrvSCC: service not available, waiting again...')
        self.req = SetControllers.Request()

    def send_request(self, ctrl_name, resources):
        ctrl = Controller()
        ctrl.resources=resources
        ctrl.name=ctrl_name
        self.req.controllers = [ctrl]
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

class GetRobotStateClient(Node):
    def __init__(self):
        super().__init__('get_robot_state_client_async')
        self.cli = self.create_client(GetRobotStates, '/left_and_right/get_robot_states')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SrvGRS: service not available, waiting again...')
        self.req = GetRobotStates.Request()

    def send_request(self):
        request = GetRobotStates.Request()
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

class SetDualCartesianImpedanceClient(Node):
    def __init__(self):
        super().__init__('set_cartesian_impedance_client_async')
        self.cli = self.create_client(SetCartesianImpedance, '/left_and_right/dual_cartesian_impedance_controller/parameters')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SrvSCC: service not available, waiting again...')
        self.req = SetCartesianImpedance.Request()

    def send_request(self, mode):
        if(mode == 0):
            '''
            float64[36] stiffness     # column major
            float64[6]  damping_ratio
            float64 nullspace_stiffness
            '''
            self.req.stiffness = [ # translation yes, rotation no
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 20.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 20.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 20.0
            ]
            self.req.nullspace_stiffness = 0.0
        else:
            self.req.stiffness = [
                400.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 400.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 400.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 20.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 20.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 20.0
            ]
            self.req.nullspace_stiffness = 10.0
        self.req.damping_ratio = [0.8, 0.8, 0.8, 0.8, 0.8, 0.8]
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    set_controller_client = SetControllerClient()
    get_robot_state_client = GetRobotStateClient()
    set_cartesian_impedance_client = SetDualCartesianImpedanceClient()
    
    def shutdown_demo():
        set_controller_client.destroy_node()
        get_robot_state_client.destroy_node()
        set_cartesian_impedance_client.destroy_node()
        rclpy.shutdown()

    sleep_duration = 20
    # Activate the joint impedance controllets for left and right, separately
    response = set_controller_client.send_request("panda_joint_impedance_controller", ['left','right'])
    set_controller_client.get_logger().info('Joint impedance Request sent')
    for i in range(0, sleep_duration):
        if(i > sleep_duration-5):
            print("Next stage in ", sleep_duration-i)
        time.sleep(1)
    
    # Activate the Cartesian impedance controllets for left and right, separately
    response = set_controller_client.send_request("panda_cartesian_impedance_controller", ['left','right'])
    set_controller_client.get_logger().info('Cartesian impedance Request sent')
    for i in range(0, sleep_duration):
        if(i > sleep_duration-5):
            print("Next stage in ", sleep_duration-i)
        time.sleep(1)

    # Activate the comless dual cartesian impedance controllets, using left and right
    response = set_controller_client.send_request("comless_dual_cartesian_impedance_controller", ['left&right'])
    set_controller_client.get_logger().info('cmlCDCIC Request sent')
    for i in range(0, sleep_duration):
        if(i > sleep_duration-5):
            print("Next stage in ", sleep_duration-i)
        time.sleep(1)

    # Activate the dual cartesian impedance controllets, using left and right
    response = set_controller_client.send_request("dual_cartesian_impedance_controller",  ['left&right'])
    set_controller_client.get_logger().info('CDCIC Request sent')
    # Additionally, set the stiffness and damping of the controllet to 0
    set_cartesian_impedance_client.send_request(0)
    for i in range(0, sleep_duration):
        if(i > sleep_duration-5):
            print("Next stage in ", sleep_duration-i)
        time.sleep(1)

    # Activate the comless coupled dual cartesian impedance controllets, using left and right
    response = set_controller_client.send_request("comless_coupled_dual_cartesian_impedance_controller")
    set_controller_client.get_logger().info('CCplDCIC Request sent')
    for i in range(0, sleep_duration):
        if(i > sleep_duration-5):
            print("Next stage in ", sleep_duration-i)
        time.sleep(1)
    
    shutdown_demo()
    return


if __name__ == '__main__':
    main()