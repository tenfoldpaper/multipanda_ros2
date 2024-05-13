# franka_multi_mode_controller

## Overview
The multi-mode controller (MMC) is a meta-package that consists of 5 packages, each with a specific role:
- `_msg` packages provide the message definitions,
- `multi_mode_controller` is where the core logic of the multi-mode controller (MMC) is implemented,
- `multi_mode_controller_impl` is the `ros2-control`-controller instantiation of MMC,
- `panda_motion_generators` implement two example motion generators to be used with the MMC.

We will first begin with the `multi_mode_controller` package, and go through the code structure and how you can implement a new controller in this framework.

## multi_mode_controller
MMC provides its own structure for implementing new controllers, dubbed _controllets_ to distinguish it from the typical `ros2-control` controllers. 

### How to implement a new controllet

#### Implementing the base (comless) controllet
For implementing a new controllet in the MMC framework, you must first implement a "comless" version of the class, then inherit that class to write the version that provides interfaces with ROS, i.e. subscribers and services.

The comless version serves as the core computation class of your controllet. The inherited "com" version simply adds the interfaces, meaning that one good controller can be inherited to multiple "com" classes for a wide range of different goal messages and use cases.

We will take `comless_panda_cartesian_impedance_controller.cpp/h` as the example here. This controllet implements a standard Cartesian impedance controller for a single arm. 

The comless class inherits from `PandaControllerBase` with a templated `Pose` and `Param` struct that you can customize. Instances of these will be used in the main 1khz torque computation loop.
In our example, this is defined like:
``` cpp
// in the header file
struct PandaCartesianImpedanceControllerPose {
  Eigen::Matrix<double, 7, 1> q_n;
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
};
struct PandaCartesianImpedanceControllerParams {
  Eigen::Matrix<double, 6, 6> stiffness;
  Eigen::Matrix<double, 6, 1> damping_ratio;
  double nullspace_stiffness;
};
```
Then you can use that as the template argument for declaring the inherited class:
``` cpp
// in the header file
class ComlessPandaCartesianImpedanceController :
    public virtual PandaControllerBase<PandaCartesianImpedanceControllerParams,
                                       PandaCartesianImpedanceControllerPose> 
```
Once you have defined the template structs, you can now move onto overriding the functions from the inherited `PandaControllerBase` class. The 3 functions that you need to override are:
- `defaultParameters`
This function is executed when the controller is activated to set the default values of the controller parameters, defined in `Param`. For our example, this is simply setting the stiffness and damping values to some desired initial values.

- `getCurrentPoseImpl`
This function is where you define how to process the model and state data that you get from the robot to the custom `Pose` format that you defined. The example's `Pose` requires joint nullspace values and the end effector position and orientation. This is implemented like this:
``` cpp
Pose Controller::getCurrentPoseImpl() {
  Pose p;
  Eigen::Map<const Vector7d> q(robot_data_[0]->state().q.data());
  Eigen::Affine3d transform(
      Eigen::Matrix4d::Map(robot_data_[0]->state().O_T_EE.data()));
  p.position = transform.translation();
  p.orientation = Eigen::Quaterniond(transform.linear());
  p.q_n = q;
  return p;
}
```
You may notice that the `robot_data_` variable is indexed at 0. This is because the controllet cares about only 1 arm, and as such, the size of the `robot_data_` is always 1. A controllet that controls 2 arms would have a `robot_data_` of size 2.

- `computeTauImpl`
This implements the function that will be executed at 1khz, using a desired `Pose` and current `Param` of the controller. It takes in a reference to the torque array that is passed to it from the MMC instantiation, where the final calculated torque values are written to.

In our example, we begin the calculation by first getting all the data we need in this cycle:
``` cpp
Pose current = getCurrentPose(); 
current.position -= getOffset().position;
Eigen::Map<const Matrix7d> inertia(robot_data_[0]->mass().data());
Eigen::Map<const Vector7d> coriolis(robot_data_[0]->coriolis().data());
Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(
    robot_data_[0]->eeZeroJacobian().data());
Eigen::Map<const Vector7d> qD(robot_data_[0]->state().dq.data());
```

In the next chunk, we use the desired `Pose` arg, and do the impedance control calculation.
Notice that we use the `Param` to obtain the current stiffness and damping values. 
After the calculations are done, we simply add the different torques up, and add it to the final desired tau, `tau_d`.
``` cpp
Vector6d error;
error.head(3) << current.position - desired.position;
if (desired.orientation.coeffs().dot(current.orientation.coeffs()) < 0.0) {
current.orientation.coeffs() << -current.orientation.coeffs();
}
Eigen::Quaterniond rot_error(
    current.orientation * desired.orientation.inverse());
Eigen::AngleAxisd rot_error_aa(rot_error);
error.tail(3) << rot_error_aa.axis() * rot_error_aa.angle();
Vector7d tau_task, tau_nullspace, tau_d;
Matrix6d D = sqrtDesign<6>(pandaCartesianInertia(jacobian, inertia),
    p.stiffness, p.damping_ratio);
tau_task << jacobian.transpose() * (-p.stiffness*error - D*(jacobian*qD));
tau_nullspace <<
    getDynamicallyConsistentNullspaceProjection<7>(inertia, jacobian) *
    (p.nullspace_stiffness * (desired.q_n - current.q_n) -
    (2.0 * std::sqrt(p.nullspace_stiffness)) * qD);
tau_d << tau_task + tau_nullspace + coriolis;
``` 

Finally, we write the value of `tau_d` to the command torque reference that is passed as an argument:
``` cpp
for (size_t i = 0; i < 7; ++i) {
(*tau[0])[i] = tau_d[i];
}
```

Putting this all together, it looks like this:
``` cpp
void Controller::computeTauImpl(const std::vector<std::array<double, 7>*>& tau,
    const Pose& desired, const Params& p) {
  Pose current = getCurrentPose();
  current.position -= getOffset().position;
  Eigen::Map<const Matrix7d> inertia(robot_data_[0]->mass().data());
  Eigen::Map<const Vector7d> coriolis(robot_data_[0]->coriolis().data());
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(
      robot_data_[0]->eeZeroJacobian().data());
  Eigen::Map<const Vector7d> qD(robot_data_[0]->state().dq.data());
  Vector6d error;
  error.head(3) << current.position - desired.position;
  if (desired.orientation.coeffs().dot(current.orientation.coeffs()) < 0.0) {
    current.orientation.coeffs() << -current.orientation.coeffs();
  }
  Eigen::Quaterniond rot_error(
      current.orientation * desired.orientation.inverse());
  Eigen::AngleAxisd rot_error_aa(rot_error);
  error.tail(3) << rot_error_aa.axis() * rot_error_aa.angle();
  Vector7d tau_task, tau_nullspace, tau_d;
  Matrix6d D = sqrtDesign<6>(pandaCartesianInertia(jacobian, inertia),
      p.stiffness, p.damping_ratio);
  tau_task << jacobian.transpose() * (-p.stiffness*error - D*(jacobian*qD));
  tau_nullspace <<
      getDynamicallyConsistentNullspaceProjection<7>(inertia, jacobian) *
      (p.nullspace_stiffness * (desired.q_n - current.q_n) -
       (2.0 * std::sqrt(p.nullspace_stiffness)) * qD);
  tau_d << tau_task + tau_nullspace + coriolis;
  for (size_t i = 0; i < 7; ++i) {
    (*tau[0])[i] = tau_d[i];
  }
}
```

Once you've finished implementing the functions, you need to register it to the `ControllerFactory` by giving it a unique name. The factory maps the name you give to the controllet to the controllet itself, so that it can be loaded up:
``` cpp
static auto registration = ControllerFactory::registerClass<Controller>(
    "comless_panda_cartesian_impedance_controller");
    // or whatever name you want
```

See the example code for the full implementation, including headers.

Once you've saved the code, you can now compile it by adding it to the MMC's `CMakeLists.txt`. Unlike other `ros2-control` controllers, there's no additional `xml` file that you need to add a description of.
``` cmake
add_library(multi_mode_controller SHARED 
  src/robot_data.cpp # This is required!
  src/controller_factory.cpp # This is required!
  src/redundancy_resolution.cpp # This is required!
  src/your_controllet.cpp
)
```
Make the adjustment, and run `colcon build` in your workspace root.

Now for the good bit: to load the controller, you simply need to adjust the corresponding `.yaml` file under the MMC instantiation in `franka_bringup`, just like a typical `ros2-control` scenario. The name you give here needs to match the one you provided for the registration step.
``` yaml
single_multi_mode_controller:
  ros__parameters:
    arm_count: 1
    arm_1:
      arm_id: panda
    controllers: ["your_controller"]
```

#### Implementing the com controllet
Once you've written the comless version, now it's time to define the interfaces to ROS2 so that you can affect your controllet.

Typically, you would need a subscriber to receive new desired `Pose`s for the control loop, and a way to adjust the `Param` at runtime, which is implemented as a service.

Both the subscriber and service callbacks are inherited from ` panda_controller_ros_interface.h`. At the moment, you can only define one callback function each for those.

Taking the `panda_cartesian_impedance_controller.cpp/h` as the example, the com class needs to inherit from two classes: the comless version that provides the 1khz torque calculation loop, and the `ControllerRosInterface` class that defines the virtual callback functions that you need to override:
``` cpp
namespace panda_controllers {
    using GoalMsg = multi_mode_control_msgs::msg::CartesianImpedanceGoal;
    using ServiceParameter = multi_mode_control_msgs::srv::SetCartesianImpedance;
    using Pose = PandaCartesianImpedanceControllerPose;
    using Parameters = PandaCartesianImpedanceControllerParams;

class PandaCartesianImpedanceController :
    public virtual ComlessPandaCartesianImpedanceController,
    public virtual ControllerRosInterface<ServiceParameter, 
                                          GoalMsg, 
                                          Parameters, 
                                          Pose> {
// ...
};
}
```

There are only 2 functions you need to override:
- desiredPoseCallbackImpl
This is not the callback function per se, but rather the function that that is called by the generic callback function that is already implemented in the inherited interface class.
It is defined as:
``` cpp
bool desiredPoseCallbackImpl(Pose& p_d, 
                             const Pose& p,
                             const GoalMsg& msg) 
```
where `p_d` is the reference to the desired `Pose` that you saw earlier in the `computeTaulImpl`, `p` is the robot's current `Pose`, and `msg` is the message you received.
`p` is provided to allow for implementing safety checks before setting the desired pose. In the example, this is used to check for whether `p_d` is too far away from the current pose, as that would result in too high of a torque value.

- setParametersCallbackImpl
This function is similar to the above: it provides you with the reference to the desired `Param` `p_d`, the current `Param` `p`, and the typical ROS2 service request and response variables.
``` cpp
  bool setParametersCallbackImpl(Parameters& p_d, 
        const Parameters& p,
        const ServiceParameter::Request::SharedPtr& req, 
        const ServiceParameter::Response::SharedPtr& res) 
```

After you're done, you need to register it to the factor just like the comless version:
``` cpp
static auto registration = ControllerFactory::registerClass<Controller>(
    "panda_cartesian_impedance_controller");
```

You can then compile and load it up in the exact same way as the comless version.

## multi_mode_controller_impl
More extensive tutorial to come.

This package implements a `ros2_control` controller that makes use of the MMC.

Similarly to the `HardwareInterface`, the MMC does not directly immplement any control logic; it manages controllets. Whenever a new controllet is loaded, it first unloads whatever active controller from the affected robots, and then activates the new controllet.

The MMC executes the currently active controllets' `computeTauImpl` in its `update` function.

### Services
#### set_controllers
Changing controllets via MMC is implemented as a ROS2 service. It is advertised under `~/set_controllers`, and takes in the `SetControllers.srv` in `multi_mode_control_msgs` as the message type. This `srv` comprises of an array of `Controller`s, which in turn is structured as:
```
string name
string[] resources
```
where `name` refers to the controllet you want to load, e.g. `panda_cartesian_impedance_controller`, and the `resource` is an array of strings that you want to execute the controllet on. In a single-arm scenario with a robot called `panda`, to start the `panda_cartesian_impedance_controller`, your `srv` arguments would look something like this:
```
# SetControllers.srv
Controllers[0] = {name: 'panda_cartesian_impedance_controller', resource: ['panda']}
```

With the MMC, you can start a single-arm controllet on multiple arms at the same time. This would amount to each of the arms running that particular controllet simultaneously. If we have 2 robots called `left` and `right` and we want to start the same controllet, the `srv` arguments would look like:
```
# SetControllers.srv
Controllers[0] = {name: 'panda_cartesian_impedance_controller', resource: ['left', 'right']}
```

On the other hand, if you have a controllet that takes control of 2 robot arms, e.g. the `dual_panda_cartesian_impedance_controller`, you can define which arms it needs to control by concatenating the names separated by `&`, like:
```
# SetControllers.srv
Controllers[0] = {name: 'dual_panda_cartesian_impedance_controller', resource: ['left&right']}
```

Once the request is received, the MMC handles the resource management, and your new controllet will start in ~2ms (i.e. 2 control cycles).

#### Loaded controllet
The loaded up controllet's goal pose subscriber and param setter service become available under the format `/{robot_name}/{controllet_name}/(desired_pose/parameters)`. For multi-arm controllets, the `&` is replaced by `_and_`. So for the above single- and dual-arm cases, they would be:
- `/panda/panda_cartesian_impedance_controller/(desired_pose/parameters)`
- `/left_and_right/dual_panda_cartesian_impedance_controller/(desired_pose/parameters)`


#### get_controllers
You can easily query what controllets are loaded to the MMC by sending the `GetControllers.srv` to the `~/get_controllers` service. The `srv` is structured like:
```
---
Controller[] controllers
```
where the return argument `controllers` contain the information.

### Configuring the launch yaml
To use the MMC, you need to first add it in the corresponding `.yaml` file that is loaded up by the launch file in `franka_bringup`. A simple example can be found in `multimode.yaml`. The MMC is listed like any other `ros2-control` controller, with additional parameters to define which controllets you want to load up, the resources they need, and what controllet you want the MMC to run by default when it is activated:
``` yaml
single_multi_mode_controller:
  ros__parameters:
    arm_count: 1
    arm_1:
      arm_id: panda
    controllers: ["panda_joint_impedance_controller", "panda_cartesian_impedance_controller"]
    resources:
        panda_joint_impedance_controller: ["panda"]
        panda_cartesian_impedance_controller: ["panda"]
    start_controllers:
        names: ["panda_cartesian_impedance_controller"]
        resources:
            panda_cartesian_impedance_controller: ["panda"]

```
Each loaded controllet has its own set of internal variables, e.g. parameters and desired pose. They are not shared across controllets.

For a multi-arm example, please look at `dual_multimode.yaml`.

### Other resources
An example python script that makes use of the MMC can be found in `mmc_demo_script.py`. It simply sleeps for 20 seconds between each controllet, and then activates the next controllet.

## Other chapters
[Back to main](../main.md)
[franka_bringup](./franka_bringup.md)
[franka_description](./franka_description.md)
[franka_hardware](./franka_hardware.md)
[franka_multi_mode_controller](./franka_multi_mode_controller.md)