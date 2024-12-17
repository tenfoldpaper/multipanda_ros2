# Custom Franka Controllers

## Description

This README is a tutorial for creating a custom controller by yourself. The joint_impedance_controller and cartesian_impedacne_controller are already implemented.
Please make sure you are able to build the bimanual_archetecture before starting customize your own controllers :)
Also, please make sure you have installed the `rqt_controller_manager` with `sudo apt-get install ros-humble-rqt-controller-manager`.

## Create your own controller Step by Step
1. Implement the header file in `/include/franka_controllers` as `<name_your_controller>.hpp` and source file in `/src` as `<name_your_controller>.cpp`. Templates of both header file and cpp file are created, please refer to the templates for more detailed notice on implementation. (The custom controller is only registered in `/franka_bringup/config/sim_controller.yaml` for bring up.)

2. Edit `CMakeLists.txt`. Add your controller in `add_library`:
```
add_library(
        ${PROJECT_NAME}
        SHARED
        src/joint_impedance_controller.cpp
        src/<name_your_controller>.cpp
)
```
Edit `franka_controller.xml`. Add your controller under `<library path="franka_controllers">...</library>`:
```
<class name="franka_controllers/<NameYourController>"
           type="franka_controllers::<NameYourController>" base_class_type="controller_interface::ControllerInterface">
	<description>
	    Description of your controller...
	</description>
</class>
```
Try to build the `franka_controllers` pkg with `colcon build --packages-select franka_controllers` after you finish the implementation. If there is no problem, let's move on to the last step :)

3. Load necessary parameters and register your controller at `rqt_controller_manager`. Go to `franka_bringup` pkg. Under `/config`, add your controller into `controllers.yaml`/`sim_controller.yaml`/`dual_controllers.yaml`/`dual_sim_controllers.yaml`. Choose the yaml file you need according to the launch file your are using (can be found in `launch` under `franka_bringup`). 

Register your controller at controller manager:
```
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz
    
    joint_impedance_controller:
      type: franka_controllers/JointImpedanceController
      
    <name_your_controller>:
      type: franka_controllers/<NameYourController>
```

Then edit necessary parameters to be loaded under like:
```
<name_your_controller>:
  ros__parameters:
    arm_id: panda
    <parameter_1>: <xxx>
    <parameters_2>:
      - <xxx>
      - <xxx>
```
4. BINGO! You have finished customizing your controller. Launch the `bimaunual_archecture`. Run the `rqt_controller_manager` with `ros2 run rqt_controller_manager rqt_controller_manager`. You should be able see your custom controller in the controller list. GOOD LUCK!

## Adapted Impedance Controller
The joint & cartesian impedance controllers are already adapted to general purpose controller from the example given by Franka. You could directly use them or modify them as you like.
### Joint Impedance Controller
The controller subsribes to the desired joint position under the topic `/joint_impedance/joints_desired`. The msg type is `std_msgs::msg::Float64MultiArray`. To use it, you need to publish a 1d array containing the 7-Dof joint position under this topic. Example of the msg:
```
from std_msgs.msg import Float64MultiArray
msg = Float64MultiArray()
msg.data = [0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0]
```
This controller is registered in `controllers.yaml` and `sim_controllers.yaml`.
### Joint Impedance Controller
The controller subsribes to the desired cartesian position under the topic `/cartesian_impedance/pose_desired`. The msg type is `std_msgs::msg::Float64MultiArray`. To use it, you need to publish a 1d array containing the 3-Dof position array and 9-Dof orientation matrix under this topic. The position array is in the first three entries while the last nie entries contain the orientation matrix. The way to reformat the orientation matrix is shown in the following example. Example of the msg:
```
from std_msgs.msg import Float64MultiArray
msg = Float64MultiArray()
position = [p1, p2, p3]
orientation = [
	[a11, a12, a13],
	[a21, a22, a23],
	[a31, a32, a33]
]
msg.data = [p1, p2, p3, a11, a12, a13, a21, a22, a23, a31, a32, a33]
```
This controller is registered in `sim_controllers.yaml`.

P.S.: It was tried that to also directly receive a quaternion for orientation. However, it will lead to instability. It seems that the orientation quaternion has to be converted from matrix by `Eigen::Quaterniond` inside the controller. Since the user can also transform a quaternion into matrix outside the controller, the feature to receive quaternion is abandoned. If you still need such feature, feel free to contact the maintainer and have a discussion about it :)


