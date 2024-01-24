```
pip install numpy-quaternion
sudo apt-get install ros-${ROS_DISTRO}-ur
sudo apt install ros-humble-moveit
git clone https://github.com/kemjensak/imu_teleop.git
git clone https://github.com/kemjensak/xsens_mtw_driver-release
git clone https://github.com/kemjensak/moveit_servo_imu.git

ros2 run xsens_mtw_driver mt_w_manager
(allign all 4 IMUs in same orientation)
ros2 launch connect_imu_launch.py
(센서부착)
ros2 run imu_teleop calc_bf_quat.py
(stand stoop)
(mount sensors to specified position and maintain start pose) 
ros2 run imu_teleop fbf_imu.py
(pose to control ARM)

ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=172.17.0.2 launch_rviz:=true
(change controller to forward position controller from trajectory controller)
ros2 launch imu_teleop servo_example.launch.py ur_type:=ur5e launch_rviz:=false
```

