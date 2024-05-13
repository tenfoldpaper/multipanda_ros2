# franka_bringup
This package serves as the repository for the launch files that you wlil use to start up the framework.
Broadly speaking, there are currently 2 versions of the launch file that you can use:
- the single-arm `franka.launch.py` and its simulation version, `franka_sim.launch.py`
- the two-arm `dual_franka.launch.py` and its simulation version, `dual_franka_sim.launch.py`.
The launch files are where you define the various configurations for starting up the robot. Several parameters are defined as launch arguments, while others are hard-coded.

## The launch files
### Real robot launch files
The argument for the real robot launch files (i.e. the ones without `sim`) are:
- the robot's name `arm_id` -- this is fixed to `panda` for the single-arm version.
- the robot's IP address `robot_ip`, 
- its gripper configuration `hand`,
- RVIZ visualization toggle, `use_rviz`. 
- Fake hardware utilization, `use_fake_hardware` and `fake_sensor_commands` -- although these are just legacy arguments from the original `franka_ros2` repository; you would now simply use the simulation if you do not have access to a real robot.

Hard-coded values for the real robot launch files are:
- the URDF path, 
- `ros2-control`'s `yaml` configuration file.

### Sim robot launch files
The argument for the MuJoCo simulated robot launch files (i.e. `sim`s) are:
- the robot's name `arm_id` -- this is fixed to `panda` for the single-arm version.
- its gripper configuration `hand`,
- RVIZ visualization toggle, `use_rviz`,
- MuJoCo scene file directory, `scene_xml`,
- Optional `yaml` file with a list of objects to publish their poses, `mj_yaml`.

Hard-coded values for the sim robot launch files are:
- the URDF path,
- `ros2-control`'s `yaml` configuration file.

For the single-arm version (`franka_sim.launch.py`), the default `scene.xml` will be loaded when no arguments are given, and the `dual_scene.xml` for the dual-arm version.

For the sim version, the `arm_id` needs special attention -- this should match the name that you gave to the MuJoCo Panda robot (described in [franka_description](./franka_description.md)). In the `TEMPLATE_panda.xml` file, these names are all marked with `{YOUR_NAME}`. Since MuJoCo does not support parameter passing, this needs to be changed manually. For exact reason for why this is necessary, please see [franka_hardware](./franka_hardware.md).

For example, if you decide to use the name `mobile_arm` as your robot's name, you should replace all instances of `{YOUR_NAME}` in a copy of the template file, and also set the `arm_id` argument as `mobile_arm`. Finally, you need to change the included file in the main `scene.xml` file that you provide as the MuJoCo scene.

The optional `mj_yaml` parameter defines the list of objects, whose poses you want to have it published as a `franka_msgs/PoseStampedArray` under the topic `mj_object_poses`. The names that you give to this list should match the ones that you load up into MuJoCo. Currently, this is achieved by either updating the `objects.xml` file or including your own `xml` file defining the objects into your scene, **as a child of the worldbody**. This is because MuJoCo data's `body_pose` and `body_quat` are defined relative to that object's parent. When a valid object is found from the `mj_yaml`, all robot's base link `<robot_name>_link0` pose is also published, to allow for easier relative pose calculation. If left blank, it won't create the publisher, and the simulation will run as usual.

### Structure of the launch files
The different parameters are arguably the only major differences between the real and sim launch files. The rest of the code is structured very similarly. Any controller or code that you write for the real robot should work on the sim robot without requiring any additional changes, and vice-versa.

Nodes are launched via the `Node` class in the long `return LaunchDescription` section. The essential nodes that should be launched for all new launch files (specified via the `executable` argument) are:
- `robot_state_publisher` -- this publishes transform data and RVIZ visualization parameters.
- `joint_state_publisher` -- this publishes the robot's joint position data, also needed for RVIZ.
- `franka_control2_node` -- this starts up the variant of the `franka_hardware` defined in the `ros2-control` `xacro` file.
Gripper node is started if the `hand` argument is set to true. 

You can also choose to launch the framework with some default `ros2-control` controllers, if they have been defined in the corresponding `yaml` file. The following code would start the `joint_state_broadcaster` controller:
``` python
Node(
    package='controller_manager',
    executable='spawner',
    arguments=['joint_state_broadcaster'],
    output='screen',
)
```
The other launch files in the folder besides the ones already mentioned are essentially different variations of this basic format, with different starting controllers and/or different hard-coded parameters.

### yaml configuration files
The `.yaml` configuration files, found under the `config` folder in the package, manages what controllers are loaded up by `ros2-control` when it is started up. By default, they are in unloaded, unconfigured state.

You define the controllers you wish to add (e.g. `my_controller`) by adding them under this line:
``` yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz

    cartesian_impedance_example_controller:
      type: franka_example_controllers/CartesianImpedanceExampleController

    ...
    
    my_controller:
      type: some_controller_package/MyController
```
This controller will first need to be compiled and properly configured for it; please see the `franka_example_controller` section in [main](../main.md) for a brief guide.

Parameters for the new controller can then be added under a new root item, like:
``` yaml
controller_manager:
  ros__parameters:
    update_rate: 1000 # Hz
    ...

my_controller:
  ros__parameters:
    arm_id: panda
    ...
```
These will be then available in the controller's functions.

## Other chapters
[Back to main](../main.md)
[franka_bringup](./franka_bringup.md)
[franka_description](./franka_description.md)
[franka_hardware](./franka_hardware.md)
[franka_multi_mode_controller](./franka_multi_mode_controller.md)