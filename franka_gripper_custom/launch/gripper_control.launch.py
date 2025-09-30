# Copyright (c) 2021 Franka Emika GmbH
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.




from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node




def generate_launch_description():
    # Parameters
    robot_ip_param = LaunchConfiguration("robot_ip")
    default_speed_param = LaunchConfiguration("default_gripper_speed")
    pub_frequency_param = LaunchConfiguration("pub_frequency")
    gripper_max_effort_param = LaunchConfiguration("gripper_max_effort")
    default_epsilon_inner_param = LaunchConfiguration("default_epsilon_inner")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_ip",
            default_value="176.16.0.1",
            description="Hostname or IP address of the robot (for the gripper connection)."
        ),
        DeclareLaunchArgument(
            "default_gripper_speed",
            default_value="0.1",
            description="Default speed for gripper motion in m/s."
        ),
        DeclareLaunchArgument(
            "pub_frequency",
            default_value="50",
            description='Publisher frequency to publish data for collection.'
        ),
        DeclareLaunchArgument(
            "gripper_max_effort",
            default_value="50.0",
            description='Max tolerated effort for grasping before throwing an error.'
        ),
        DeclareLaunchArgument(
            "default_epsilon_inner",
            default_value="0.01",
            description='Inner tolerance for grasping in meters.'
        ),

        Node(
            package="franka_gripper_custom",  
            executable="franka_gripper_control",   
            name="panda_gripper", 
            output="screen",
            parameters=[{
                "robot_ip": robot_ip_param,
                "default_gripper_speed": default_speed_param,
                "pub_frequency": pub_frequency_param,
                "gripper_max_effort": gripper_max_effort_param,
                "default_epsilon_inner": default_epsilon_inner_param,
            }]
        ),
    ])