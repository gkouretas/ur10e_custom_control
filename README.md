Available controllers
---------------------

forward_position_controller[position_controllers/JointGroupPositionController]
scaled_joint_trajectory_controller[ur_controllers/ScaledJointTrajectoryController]
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster]
io_and_status_controller[ur_controllers/GPIOController]
speed_scaling_state_broadcaster[ur_controllers/SpeedScalingStateBroadcaster]
force_torque_sensor_broadcaster[force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster]    

ros2 run ur_client_library start_ursim.sh -a 192.168.57 -m ur10e -v 5.9.4 -d 1
ros2 launch ur_robot_driver ur10e.launch.py robot_ip:=192.168.57.101 headless_mode:=true

- `use_fake_hardware`: true -> uses ROS2 commands
- `use_fake_hardware`: false -> uses UrSim
ros2 control load_controller --set-state {configured} forward_velocity_controller
ros2 control set_controller_state forward_velocity_controller active
ros2 launch ur_robot_driver test_forward_velocity_controller.launch.py
ros2 control set_controller_state forward_velocity_controller inactive

ros2 control set_controller_state scaled_joint_trajectory_controller active
ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py
ros2 control set_controller_state scaled_joint_trajectory_controller inactive

docker network rm ursim_net
systemctl status docker
sudo systemctl start docker

Cleanup docker (fallback in race condition...)
```bash
sudo docker network disconnect -f ursim_net ursim
sudo systemctl restart docker.socket docker.service; sudo docker rm $(sudo docker ps -a -q) -f
```

Other docker issues:
- Network conflicts. Resolution: https://stackoverflow.com/questions/40524602/error-creating-default-bridge-network-cannot-create-network-docker0-confli
- https://superuser.com/questions/1741326/how-to-connect-to-docker-daemon-if-unix-var-run-docker-sock-is-not-available
- https://stackoverflow.com/questions/51857634/cannot-connect-to-the-docker-daemon-at-unix-var-run-docker-sock-is-the-docke
- Uninstall docker: https://askubuntu.com/questions/935569/how-to-completely-uninstall-docker
- Install docker: https://www.cherryservers.com/blog/install-docker-ubuntu-22-04

ros2 run ur_client_library start_ursim.sh -a 192.168.57 -m ur10e -v 5.9.4 headless:=true && ros2 launch ur_robot_driver ur10e.launch.py robot_ip:=192.168.57.101 headless_mode:=true && ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py