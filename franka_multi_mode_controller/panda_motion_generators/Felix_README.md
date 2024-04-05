## Building and testing
1. For building, refer to the `CMakeLists.txt` and add your files accordingly.
    - Then, run `colcon build --packages-select panda_motion_generators`
    - If you need to define your own message definition, you find that in `panda_motion_generator_msgs/action`. Refer to the existing files there, and update the `CMkaeLists.txt` then run `colcon build --packages-select panda_motion_generator_msgs`.
2. For running, you need to run the multimode controller, and the corresponding motion generator node that you built from step 1, as well as the test client.
    - To run the main control software (one arm case): `source ~/bimanual_ws/install/setup.bash & ros2 launch franka_bringup multimode_franka.launch.py robot_ip_1:=192.168.3.101 arm_id_1:=panda`
    This is alias'd in `~/.bashrc` as `mm_go`.
    - Then run the motion generator node, after sourcing the workspace: `ros2 run panda_motion_generators your_circular_field_node`
    - Then run the test client, which will publish the action that the robot needs to take: `ros2 run panda_motion_generators your_client_node`
3. For resetting the robot to the Franka init pose, you can either run:
    - `comtest` or `/home/bimanual/Libraries/libfranka/build/examples/communication_test 192.168.3.101` -- this requires you to shut down the `ros2_control` node you start with `mm_go`.
    - With `mm_go` running, run: 
    `ros2 run panda_motion_generators panda_poly_c2_joint_motion_generator_node&ros2 run panda_motion_generators panda_to_init_pose_client`
    This basically runs the joint motion generator and the init pose client that queries the robot's current joint pose and interpolates it to the init pose.
    
     
