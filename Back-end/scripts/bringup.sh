#! /bin/bash
#export ROS_MASTER_URI=http://10.31.107.138:11311
#export ROS_IP=10.31.107.138
#export ROS_MASTER_URI=http://192.168.1.42:11311
#export ROS_IP=192.168.1.42
source /opt/ros/humble/setup.bash
source /home/robot22/microros_ws/install/setup.bash
source /home/robot22/ros2robot/install/setup.bash
ros2 launch ctrobot robot_bringup.launch.py 
