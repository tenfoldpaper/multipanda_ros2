#  Copyright (c) 2021 Franka Emika GmbH
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


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_id_1_parameter_name = "arm_id_1"
    arm_id_2_parameter_name = "arm_id_2"
    load_gripper_1_parameter_name = 'load_gripper_1'
    load_gripper_2_parameter_name = 'load_gripper_2'
    use_rviz_parameter_name = 'use_rviz'
    scene_xml_parameter_name = 'scene_xml'
    mj_yaml_parameter_name = 'mj_yaml'

    arm_id_1 = LaunchConfiguration(arm_id_1_parameter_name)
    arm_id_2 = LaunchConfiguration(arm_id_2_parameter_name)
    load_gripper_1 = LaunchConfiguration(load_gripper_1_parameter_name)
    load_gripper_2 = LaunchConfiguration(load_gripper_2_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)
    scene_xml = LaunchConfiguration(scene_xml_parameter_name)
    mj_yaml = LaunchConfiguration(mj_yaml_parameter_name)

    franka_xacro_file = os.path.join(get_package_share_directory('franka_description'), 'robots',
                                     'dual_panda_arm_sim.urdf.xacro')
    default_scene_xml_file = os.path.join(get_package_share_directory('franka_description'), 'mujoco', 'franka', 'dual_scene.xml')
    default_mj_yaml_file = os.path.join(get_package_share_directory('franka_bringup'), 'config', 'mujoco', 'mj_objects.yaml')

    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, 
            ' arm_id_1:=', arm_id_1, 
            ' arm_id_2:=', arm_id_2,
            ' hand_1:=', load_gripper_1,
            ' hand_2:=', load_gripper_2,
            ' scene_xml:=', scene_xml,
            ' mj_yaml:=', mj_yaml])

    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_dual_franka.rviz')

    franka_controllers = PathJoinSubstitution(
        [
            FindPackageShare('franka_bringup'),
            'config',
            'dual_sim_controllers.yaml',
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            use_rviz_parameter_name,
            default_value='false',
            description='Visualize the robot in Rviz'),
        DeclareLaunchArgument(
            arm_id_1_parameter_name,
            default_value='mj_left',
            description='Unique name of robot 1.'
        ),
        DeclareLaunchArgument(
            arm_id_2_parameter_name,
            default_value='mj_right',
            description='Unique name of robot 2.'
        ),
        DeclareLaunchArgument(
            load_gripper_1_parameter_name,
            default_value='true',
            description='Load robot 1 with franka gripper.'),
        DeclareLaunchArgument(
            load_gripper_2_parameter_name,
            default_value='true',
            description='Load robot 2 with franka gripper.'),
        DeclareLaunchArgument(
            scene_xml_parameter_name,
            default_value=default_scene_xml_file,
            description='The path to the mujoco xml file that you want to load.'
        ),
        DeclareLaunchArgument(
            mj_yaml_parameter_name,
            default_value=default_mj_yaml_file,
            description='The path to the mujoco object yaml file that you want to load.'
        ),
        Node( # RVIZ dependency
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node( # RVIZ dependency
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {'source_list': ['franka/joint_states', 
                                '/mj_left_gripper_sim_node/joint_states',
                                '/mj_right_gripper_sim_node/joint_states'],
                 'rate': 30}],
        ),
        Node(
            package='franka_control2',
            executable='franka_control2_node',
            parameters=[{'robot_description': robot_description}, franka_controllers],
            remappings=[('joint_states', 'franka/joint_states')],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
            on_exit=Shutdown(),
        ),
        Node( # RVIZ dependency
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['franka_left_robot_state_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['franka_right_robot_state_broadcaster'],
            output='screen',
        ),
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file],
             condition=IfCondition(use_rviz)
             )

    ])
