# franka_description

## URDFs and xacros
### panda_arm.xacro and hand.xacro
The `panda_arm.xacro` file provides kinematic description of the robot. In most cases, you should not have to change anything in this file, unless you make some heavy modification to the robot.

Similarly, the `hand.xacro` provides kinematic description of the default Franka Hand. If you change the fingertips of the gripper with some custom component, this file is what you need to change. Since the hand does not have any joints that are directly controlled by `ros2-control`, you can leave the `hand.urdf.xacro` alone.

### arm.urdf.xacro and arm.ros2_control.xacro
When you want to customize the placement of your robot, e.g. in a multi-arm setup, you would make the change by modifying the corresponding `arm.urdf.xacro` file. For example, in the case of the dual-arm setup (`dual_panda_arm.urdf.xacro`), this is described by:
``` xml
<xacro:panda_arm arm_id="$(arg arm_id_1)" connected_to="base_link" rpy="0 0 0"  xyz="0 +0.26 0" safety_distance="0.03"/>
<xacro:panda_arm arm_id="$(arg arm_id_2)" connected_to="base_link" rpy="0 0 0"  xyz="0 -0.26 0" safety_distance="0.03"/>
```
where the `rpy` and `xyz` correspond to the transformation of the robot relative to the `base_link`, which is just a simple box that serve as a common root for the robots.

Another important part is the `arm.ros2_control.xacro` file, which is usually included at the end of the `arm.urdf.xacro` file:
``` xml
<xacro:include filename="$(find franka_description)/robots/dual_panda_arm_sim.ros2_control.xacro"/>
<xacro:panda_arm_ros2_control ns_1="$(arg arm_id_1)" ns_2="$(arg arm_id_2)" hand_1="$(arg hand_1)" hand_2="$(arg hand_2)" scene_xml="$(arg scene_xml)"/>
```
(Notice that the `arm_id_n` parameter is passed to `ns_n`, short for namespace.) This particular file defines which flavor of `franka_hardware` will be run for the given URDF. For the real dual-arm setup (`dual_panda_arm.ros2_control.xacro`), this looks like:
``` xml
<xacro:unless value="${use_fake_hardware}">
    <plugin>franka_hardware/FrankaMultiHardwareInterface</plugin>
    <param name="robot_ip_1">${robot_ip_1}</param>
    <param name="robot_ip_2">${robot_ip_2}</param>
</xacro:unless>
<param name="ns_1">${ns_1}</param>
<param name="ns_2">${ns_2}</param>
</hardware>
``` 
Additionally, the `<param>` tag allows you to add parameters that can be accessed by `ros2-control`. As a comparison, here is the sim version of the dual-arm (`dual_panda_arm_sim.ros2_control.xacro`):
``` xml
<hardware>
    <param name="robot_count">2</param>
    <plugin>franka_hardware/FrankaMujocoMultiHardwareInterface</plugin>
    <param name="ns_1">${ns_1}</param>
    <param name="ns_2">${ns_2}</param>
    <param name="hand_1">${hand_1}</param>
    <param name="hand_2">${hand_2}</param>
    <param name="scene_xml">${scene_xml}</param>
    <param name="mj_yaml">${mj_yaml}</param>
</hardware>
```
Notice that there are 4 additional parameters here: the `hand_1/2`, `scene_xml`, and `mj_yaml`. The `hand_n` parameter indicates whether there is a hand attached to the robot; for now, the model is loaded with it by default, and all this does is to control the creation of the gripper action servers. The `scene_xml` param tells the sim `franka_hardware` about the location of the `scene.xml` at runtime so that it can initialize the MuJoCo simulation. Finally, the `mj_yaml` param tells the `HardwareInterface` about where the file containing list of objects whose poses must be published, so that they can be processed accordingly.

The rest of the `arm.ros2_control.xacro` file defines the kind of state and command interfaces that the robot has.
For both sim and real robots, all 3 major interfaces are defined: joint torque, position and velocity. Typically, you would not need to modify anything in this section.

## MuJoCo files
The MuJoCo files can be found under the `mujoco/franka` folder of the package. By default, the robots have gravity compensation enabled, which is toggled by the `gravcomp="1"` argument in each of the bodies. This is because the real Panda robot also has gravity compensation by default.

The `scene.xml` is the default MuJoCo scene that is loaded up by the one-arm simulation. This is where you would place objects other than the robot. For the single-arm case, please take a look at `panda.xml` and `scene.xml`; it should be fairly straightforward what it does.

For the multi-arm case, you need to define all the arms that you plan to use in a single `xml` file, like in the case of `mj_dual`. In this file, Each robot's section is commented accordingly. 
You can define the absolute transform of the robots in the world space by modifying the `pos` and `quat` of this line:
``` xml
<body name="mj_left_link0" childclass="panda" pos="0 0.26 0" quat="1 0 0 0" gravcomp="1">
```

Additional objects can be included in the scene by including a separate file, such as the `objects.xml`, into your `scene.xml` directly.

MuJoCo does not provide a convenient way to define variables that can be used for other parts, unlike URDFs. As such, it can be quite a pain to track down all the robot name instances. The `TEMPLATE_panda.xml` file was created for this purpose.
All the robot name-dependent part of the `xml` file is marked with `{YOUR_NAME}`. You can simply do a find-and-replace on this string with your own name to quickly create a new instance. **Importantly, you must ensure that the name you provide here is the same as the name that you provide to the `arm_id` argument in the robot's launch file.** For exact reason for why this is necessary, please see [franka_hardware](./franka_hardware.md).

For multi-arm simulation, you would then create a new version of the robot description `xml` file, and copy-paste the parts of your custom `TEMPLATE_panda.xml` to the corresponding part; please see `mj_dual.xml` for an example.

## Other chapters
[Back to main](../main.md)
[franka_bringup](./franka_bringup.md)
[franka_description](./franka_description.md)
[franka_hardware](./franka_hardware.md)
[franka_multi_mode_controller](./franka_multi_mode_controller.md)