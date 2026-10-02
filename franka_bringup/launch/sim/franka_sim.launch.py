from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from math import pi
import os

import xacro
def concatenate_ns(ns1, ns2, absolute=False):
    
    if(len(ns1) == 0):
        return ns2
    if(len(ns2) == 0):
        return ns1
    
    # check for /s at the end and start
    if(ns1[0] == '/'):
        ns1 = ns1[1:]
    if(ns1[-1] == '/'):
        ns1 = ns1[:-1]
    if(ns2[0] == '/'):
        ns2 = ns2[1:]
    if(ns2[-1] == '/'):
        ns2 = ns2[:-1]
    if(absolute):
        ns1 = '/' + ns1
    return ns1 + '/' + ns2

def generate_launch_description():
    # Parameters as launch arguments
    arm_id_param = 'arm_id'
    initial_positions_param = 'initial_positions'
    use_rviz_param = 'use_rviz'
    
    arm_id = LaunchConfiguration(arm_id_param)
    initial_positions = LaunchConfiguration(initial_positions_param)
    use_rviz = LaunchConfiguration(use_rviz_param)

    # Fixed variables
    load_gripper = True # We make gripper a fixed variable, mainly because parsing the argument 
                        # within generate_launch_description is a fairly unintuitive process, 
                        # and it's not worth doing just for a single boolean.
    
    if(load_gripper): # mujoco scene file must be manually adjusted since there's no way to pass parameters
        scene_file = 'scene.xml'
    else:
        scene_file = 'scene_ng.xml'
    franka_xacro_file = os.path.join(get_package_share_directory('franka_description'), 'robots', 'sim',
                                     'panda_arm_sim.urdf.xacro')
    xml_file = os.path.join(get_package_share_directory('franka_description'), 'mujoco', 'franka', scene_file)
    mjros_config_file = os.path.join(get_package_share_directory('franka_bringup'), 'config', 'sim',
                                     'single_sim_controllers.yaml')
    franka_bringup_path = get_package_share_directory('franka_bringup')
    ns = ''     # this must match the namespace argument under mujoco_ros2_control in the plugin's parameter yaml file. 
                # See the ros2_control_plugins_example_with_ns.yaml file for more details.

    # Robot state publisher setup
    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, 
            ' arm_id:=', arm_id, 
            ' hand:=', str(load_gripper).lower(),
            ' initial_positions:=', initial_positions])
    
    params = {'robot_description': robot_description}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace= ns,
        parameters=[params]
    )

    # Joint state publisher setup
    jsp_source_list = [concatenate_ns(ns, 'joint_states', True)]
    if(load_gripper):
        jsp_source_list.append(concatenate_ns(ns, 'panda_gripper_sim_node/joint_states', True))

    node_joint_state_publisher = Node( # RVIZ dependency
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace= ns,
            parameters=[
                {'source_list': jsp_source_list,
                 'rate': 30}],
    )

    # Others
    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_franka.rviz')

    # Rosbridge    
    rosbridge_server = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join( get_package_share_directory('rosbridge_server'),
                'launch', 'rosbridge_websocket_launch.xml')
        ),
        launch_arguments={
            'port': '9090',
            # Additional parameters can be added here, for example:
            # 'address': '',
            # 'ssl': 'false'
        }.items()
    )
    

    return LaunchDescription([
        # Launch args
        DeclareLaunchArgument(
            use_rviz_param,
            default_value='false',
            description='Visualize the robot in Rviz'),
        DeclareLaunchArgument(
            arm_id_param,
            default_value='panda',
            description='The name of the robot. Defaults to panda.'),
        DeclareLaunchArgument(
            initial_positions_param,
            default_value='"0.0 -0.785 0.0 -2.356 0.0 1.571 0.785"',
            description='Initial joint positions of the robot. Must be enclosed in quotes, and in pure number.'
                        'Defaults to the "communication_test" pose.'),

        # Mujoco ros2 server launch
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(franka_bringup_path + '/launch/sim/launch_mujoco_ros_server.launch'),
            launch_arguments={
                'use_sim_time': "true",
                'modelfile': xml_file,
                'verbose': "true",
                'ns': ns,
                'mujoco_plugin_config': mjros_config_file
                # 'mujoco_plugin_config': os.path.join(mjr2_control_path, 'example', 'ros2_control_plugins_example.yaml')

            }.items()
        ),

        # Miscellaneous
        node_robot_state_publisher,
        node_joint_state_publisher,

        Node( # RVIZ dependency
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '-c', concatenate_ns(ns, 'controller_manager', True)],
            output='screen',
        ),
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['--display-config', rviz_file],
             condition=IfCondition(use_rviz)
        ),
        
        # Cartesian Controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['cartesian_impedance_controller', '-c', concatenate_ns(ns, 'controller_manager', True)],
            output='screen',
        ),

        # Rosbridge Initialization
        rosbridge_server,

        # Position Publisher
        Node(
            package='tf2_publisher',
            executable='franka_pub',
            name='tf2listener'
        )
    ])
