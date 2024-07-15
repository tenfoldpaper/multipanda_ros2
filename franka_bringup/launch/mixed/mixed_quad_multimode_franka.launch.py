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
    robot_ip_1_parameter_name = 'robot_ip_1'
    robot_ip_2_parameter_name = 'robot_ip_2'
    scene_xml_parameter_name = 'scene_xml'
    mj_yaml_parameter_name = 'mj_yaml'
    
    load_gripper_1_parameter_name = 'load_gripper_1'
    load_gripper_2_parameter_name = 'load_gripper_2'
    load_gripper_3_parameter_name = 'load_gripper_3'
    load_gripper_4_parameter_name = 'load_gripper_4'

    arm_id_1_parameter_name = 'arm_id_1'
    arm_id_2_parameter_name = 'arm_id_2'
    arm_id_3_parameter_name = 'arm_id_3'
    arm_id_4_parameter_name = 'arm_id_4'

    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    use_rviz_parameter_name = 'use_rviz'

    robot_ip_1 = LaunchConfiguration(robot_ip_1_parameter_name)
    robot_ip_2 = LaunchConfiguration(robot_ip_2_parameter_name)

    arm_id_1 = LaunchConfiguration(arm_id_1_parameter_name)
    arm_id_2 = LaunchConfiguration(arm_id_2_parameter_name)
    arm_id_3 = LaunchConfiguration(arm_id_3_parameter_name)
    arm_id_4 = LaunchConfiguration(arm_id_4_parameter_name)

    load_gripper_1 = LaunchConfiguration(load_gripper_1_parameter_name)
    load_gripper_2 = LaunchConfiguration(load_gripper_2_parameter_name)
    load_gripper_3 = LaunchConfiguration(load_gripper_3_parameter_name)
    load_gripper_4 = LaunchConfiguration(load_gripper_4_parameter_name)

    scene_xml = LaunchConfiguration(scene_xml_parameter_name)
    mj_yaml = LaunchConfiguration(mj_yaml_parameter_name)
    
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)

    franka_xacro_file = os.path.join(get_package_share_directory('franka_description'), 'robots',
                                     'mixed_quad_panda_arm.urdf.xacro')
    default_scene_xml_file = os.path.join(get_package_share_directory('franka_description'), 'mujoco', 'franka', 'dual_scene.xml')
    default_mj_yaml_file = os.path.join(get_package_share_directory('franka_bringup'), 'config', 'mujoco', 'mj_objects.yaml')
    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, 
         ' arm_id_1:=', arm_id_1, ' arm_id_2:=', arm_id_2,
         ' arm_id_3:=', arm_id_3, ' arm_id_4:=', arm_id_4,
         ' hand_1:=', load_gripper_1, ' hand_2:=', load_gripper_2,
         ' hand_3:=', load_gripper_3, ' hand_4:=', load_gripper_4,
         ' robot_ip_1:=', robot_ip_1, ' robot_ip_2:=', robot_ip_2, 
         ' scene_xml:=', scene_xml, ' mj_yaml:=', mj_yaml,
         ' use_fake_hardware:=', use_fake_hardware,
         ' fake_sensor_commands:=', fake_sensor_commands])

    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_dual_franka.rviz')

    franka_controllers = PathJoinSubstitution(
        [
            FindPackageShare('franka_bringup'),
            'config',
            'mixed_quad_multimode.yaml',
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            robot_ip_1_parameter_name,
            default_value='192.168.3.101',
            description='Hostname or IP address of robot 1.'),
        DeclareLaunchArgument(
            robot_ip_2_parameter_name,
            default_value='192.168.3.102',
            description='Hostname or IP address of robot 2.'),
        DeclareLaunchArgument(
            scene_xml_parameter_name,
            default_value=default_scene_xml_file,
            description='Scene xml path'),
        DeclareLaunchArgument(
            mj_yaml_parameter_name,
            default_value=default_mj_yaml_file,
            description='The path to the mujoco object yaml file that you want to load.'
        ),
        DeclareLaunchArgument(
            arm_id_1_parameter_name,
            default_value='rl_left',
            description='Unique arm ID of robot 1.'),
        DeclareLaunchArgument(
            arm_id_2_parameter_name,
            default_value='rl_right',
            description='Unique arm ID of robot 2.'),
        DeclareLaunchArgument(
            arm_id_3_parameter_name,
            default_value='mj_left',
            description='Unique arm ID of robot 3.'),
        DeclareLaunchArgument(
            arm_id_4_parameter_name,
            default_value='mj_right',
            description='Unique arm ID of robot 4.'),
        DeclareLaunchArgument(
            use_rviz_parameter_name,
            default_value='true',
            description='Visualize the robot in Rviz'),
        DeclareLaunchArgument(
            use_fake_hardware_parameter_name,
            default_value='false',
            description='Use fake hardware'),
        DeclareLaunchArgument(
            fake_sensor_commands_parameter_name,
            default_value='false',
            description="Fake sensor commands. Only valid when '{}' is true".format(
                use_fake_hardware_parameter_name)),
        DeclareLaunchArgument(
            load_gripper_1_parameter_name,
            default_value='false',
            description='Use Franka Gripper as an end-effector, otherwise, robot 1 is loaded '
                        'without an end-effector.'),
        DeclareLaunchArgument(
            load_gripper_2_parameter_name,
            default_value='false',
            description='Use Franka Gripper as an end-effector, otherwise, robot 2 is loaded '
                        'without an end-effector.'),
        DeclareLaunchArgument(
            load_gripper_3_parameter_name,
            default_value='true',
            description='Use Franka Gripper as an end-effector, otherwise, robot 3 is loaded '
                        'without an end-effector.'),
        DeclareLaunchArgument(
            load_gripper_4_parameter_name,
            default_value='true',
            description='Use Franka Gripper as an end-effector, otherwise, robot 4 is loaded '
                        'without an end-effector.'),
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
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {'source_list': ['franka/joint_states', 
                                 'panda_gripper/joint_states',
                                 '/mj_left_gripper_sim_node/joint_states',
                                 '/mj_right_gripper_sim_node/joint_states'],
                 'rate': 30}],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        # Real nodes
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['real_multi_mode_controller'],
            output='screen',
            condition=UnlessCondition(use_fake_hardware),
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['rl_left_state_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['rl_right_state_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['rl_left_model_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['rl_right_model_broadcaster'],
            output='screen',
        ),
        Node(
            package="panda_motion_generators",
            executable="panda_poly_c2_joint_motion_generator_node",
            arguments=["rl_left_joint_via_motion",
                       "/rl_left/get_robot_states",
                       "real_multi_mode_controller",
                       "panda_joint_impedance_controller",
                       "/rl_left/panda_joint_impedance_controller/desired_pose"]
        ),
        Node(
            package="panda_motion_generators",
            executable="panda_poly_c2_joint_motion_generator_node",
            arguments=["rl_right_joint_via_motion",
                       "/rl_right/get_robot_states",
                       "real_multi_mode_controller",
                       "panda_joint_impedance_controller",
                       "/rl_right/panda_joint_impedance_controller/desired_pose"]
        ),
        # Sim nodes
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['sim_multi_mode_controller'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['mj_left_state_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['mj_right_state_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['mj_left_model_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['mj_right_model_broadcaster'],
            output='screen',
        ),
        Node(
            package="panda_motion_generators",
            executable="panda_poly_c2_joint_motion_generator_node",
            arguments=["mj_left_joint_via_motion",
                       "/mj_left/get_robot_states",
                       "sim_multi_mode_controller",
                       "panda_joint_impedance_controller",
                       "/mj_left/panda_joint_impedance_controller/desired_pose"]
        ),
        Node(
            package="panda_motion_generators",
            executable="panda_poly_c2_joint_motion_generator_node",
            arguments=["mj_right_joint_via_motion",
                       "/mj_right/get_robot_states",
                       "sim_multi_mode_controller",
                       "panda_joint_impedance_controller",
                       "/mj_right/panda_joint_impedance_controller/desired_pose"]
        ),
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file],
             condition=IfCondition(use_rviz)
             )

    ])
